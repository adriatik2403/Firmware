/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drone_aqua_test.cpp
 * Implementation of a perching wall controller.
 *
 * @author Thomas Courteau 	<thomas.robichaud.courteau@gmail.com>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
//#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
//#include <uORB/topics/fw_virtual_rates_setpoint.h>
//#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/distance_sensor.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <ecl/attitude_fw/ecl_wheel_controller.h>
#include <platforms/px4_defines.h>

// on inclu le nouveau UORB topic spécifique pour le driver charging i2c
#include <uORB/topics/charging_info.h>
#include <uORB/topics/charging_info_2.h>
#include <uORB/topics/wake_up_slave_info_2.h>
#include <uORB/topics/wake_up_slave_info.h>

//#include "control_wall_landing.h"

#include <vector>

#define SENSOR_COUNT_MAX		3

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int drone_aqua_test_main(int argc, char *argv[]);

//********************************************//
// DECLARATION DE LA CLASSE DU PERCHING WALL////
//********************************************//
class DroneAquaTest
{
public:
	/**
	 * Constructor
	 */
	DroneAquaTest();

	/**
	 * Destructor, also kills the main task.
	 */
	~DroneAquaTest();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

    // ID POUR LA COMMANDE DE LA TETE DU FLYING WING
#define INDEX_SERVO_ROT 1
#define INDEX_WIRE_POS_UP 2
#define INDEX_WIRE_POS_DOWN 3


	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */

	bool		_wall_landing;
	bool		_threshold_accel;
    bool        _landed;
    bool        _take_off;          /**< Take off from the wall */
    bool        _accel_stop;        /**< if true, stop hover control */
    bool        _climb;             /**< climb mode, fly into wall */
    bool        _climb_take_off;    /**< climb mode, fly backwards */

    bool _flag_qd_calculated;
    math::Quaternion _qe;
    math::Quaternion _qd_offset;
    math::Quaternion _qd;
    math::Quaternion _qm;
    math::Vector<3> _euler_error;
    math::Vector<3> _euler_error_old;
    math::Vector<3> _derivate;
    math::Vector<3> _integrate;
    math::Vector<3> _integrate_old;
    std::vector<float> _outputs;
    float _dt;
    float _yaw_sp;
    float _hover_speed_comm;
   // float takeoff_correction;
    //float _angle_correction;
    float _speed_z_axis;
    float _speed_z_axis_old;
    float _speed_error;
    float _speed_error_old;
    float _speed_derivate;
    float _speed_integrate;
    float _speed_integrate_old;


    struct {
        std::vector<float> Kp;
        std::vector<float> Kd;
        std::vector<float> Ki;
    } _gain;


	int		_control_task;			/**< task handle */

    int		_ctrl_state_sub;	/**< control state subscription */
	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
    int 	_params_sub;			/**< notification of parameter updates */
    int 	_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
    //int 	_distance_sub;

        ///////////////////////////////////////////////////////////////////////////
    	int 		_charging_info_sub; /* pour le module de balancing i2c */
    	int 		_wake_up_slave_info_sub;
	///////////////////////////////////////////////////////////////////////////

#if 1
    int _counter;
#endif

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub;		/**< actuator control group 1 setpoint (Airframe) */

    	///////////////////////////////////////////////////////////////////////////
    	orb_advert_t     _charging_info_topic_2;          /* pour publish charging i2c */
     	orb_advert_t     _wake_up_slave_topic_2;    
	///////////////////////////////////////////////////////////////////////////

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure

    //orb_id_t _actuators2_id;    // TEST CONTROL GROUP 2 ****

	orb_id_t _attitude_setpoint_id;

	///////////////////////////////////////////////////////////////////////////
	struct charging_info_s				_charging;	/* charging i2c topic */
	struct wake_up_slave_info_s			_wake_up_slave;	
	///////////////////////////////////////////////////////////////////////////

	struct control_state_s				_ctrl_state;	/**< control state */
	struct accel_report				_accel;			/**< body frame accelerations */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s			_rates_sp;	/* attitude rates setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct vehicle_land_detected_s			_vehicle_land_detected;	/**< vehicle land detected */
    //struct distance_sensor_s			_distance;

	///////////////////////////////////////////////////////////////////////////
	struct charging_info_2_s 				report_charging_2;
	struct wake_up_slave_info_2_s 				report_wake_up_slave_2;	
	///////////////////////////////////////////////////////////////////////////

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	float _flaps_cmd_last;
	float _flaperons_cmd_last;

	float _time;
    float _time_yaw_sp;
    float _time_climb;
	float _g;


	struct {
		float p_tc;
		float p_p;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float r_tc;
		float r_p;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_d;
		float y_ff;
		float y_integrator_max;
		float y_coordinated_min_speed;
		int32_t y_coordinated_method;
		float y_rmax;
		float w_p;
		float w_i;
		float w_ff;
		float w_integrator_max;
		float w_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */
		float man_roll_max;				/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */

		float flaps_scale;				/**< Scale factor for flaps */
		float flaperon_scale;			/**< Scale factor for flaperons */

		int vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t p_tc;
		param_t p_p;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t r_tc;
		param_t r_p;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_d;
		param_t y_ff;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t y_coordinated_method;
		param_t y_rmax;
		param_t w_p;
		param_t w_i;
		param_t w_ff;
		param_t w_integrator_max;
		param_t w_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;

		param_t flaps_scale;
		param_t flaperon_scale;

		param_t vtol_type;

        /* Gain for hover control */
        param_t Kp_aileron;
        param_t Kp_elevation;
        param_t Kp_rudder;
        param_t Kp_vel_thrust;
        param_t Kd_aileron;
        param_t Kd_elevation;
        param_t Kd_rudder;
        param_t Kd_vel_thrust;
        param_t Ki_aileron;
        param_t Ki_elevation;
        param_t Ki_rudder;
        param_t Ki_vel_thrust;


	}		_parameter_handles;		/**< handles for interesting parameters */

	// Rotation matrix and euler angles to extract from control state
	math::Matrix<3, 3> _R;
	float _roll;
	float _pitch;
	float _yaw;

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_WheelController			_wheel_ctrl;

	//Control_wall_landing _control_wall_landing;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle land detected updates.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Check for distance sensor
	 */
    //void 		distance_poll();


	/**
	 *
	 * @param q
	 */
    void		hover_control(const math::Quaternion &q);

	/**
	 * fonction qui effectu la sequence de decollage
	 */
    void 		drone_aqua_take_off(int *compteur);

    void calculation_error(const math::Quaternion &val_q);
    void discrete_derivative();
    void discrete_integrate();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();

};

namespace drone_aqua_test_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

DroneAquaTest	*g_control = nullptr;
}

//********************************************//
// CONSTRUCTOR DE LA CLASSE                 ////
//********************************************//
DroneAquaTest::DroneAquaTest() :

	_task_should_exit(false),
	_task_running(false),
	_wall_landing(false),
	_threshold_accel(false),
    _landed(false),
    _take_off(false),
    _accel_stop(false),
    _climb(false),
    _climb_take_off(false),

    _flag_qd_calculated(false),
    _euler_error(0),
    _euler_error_old(0),
    _derivate(0),
    _integrate(0),
    _integrate_old(0),
    _outputs(4),
    _dt(0.005),
    _yaw_sp(0.0f),
    _hover_speed_comm(0.0f),
    //takeoff_correction(1.0f),
    _speed_z_axis(0.0f),
    _speed_z_axis_old(0.0f),
    _speed_error(0.0f),
    _speed_error_old(0.0f),
    _speed_derivate(0.0f),
    _speed_integrate(0.0f),
    _speed_integrate_old(0.0f),
   // _angle_correction(1.0f),


	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_accel_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),
	_vehicle_status_sub(-1),
	_vehicle_land_detected_sub(-1),
        //_distance_sub(-1),
        _charging_info_sub(-1),
        _wake_up_slave_info_sub(-1),

	/* publications */
	_rate_sp_pub(nullptr),
	_attitude_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_actuators_2_pub(nullptr),

	///////////////////////////////////////////////////////////////////////////
	_charging_info_topic_2(nullptr),
	_wake_up_slave_topic_2(nullptr),
	///////////////////////////////////////////////////////////////////////////

	_rates_sp_id(0),
	_actuators_id(0),
	_attitude_setpoint_id(0),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fwa_dt")),

#if 0
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fwa_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fwa_nano")),
#else
	_nonfinite_input_perf(nullptr),
	_nonfinite_output_perf(nullptr),
#endif
	/* states */
	_setpoint_valid(false),
	_debug(false),
	_flaps_cmd_last(0),
	_flaperons_cmd_last(0),
	_time(0),
        _time_yaw_sp(0),
        _time_climb(0),
	_g(9.81f)
{
	/* safely initialize structs */
	_ctrl_state = {};
	_accel = {};
	_att_sp = {};
	_rates_sp = {};
	_manual = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_global_pos = {};
	_vehicle_status = {};
	_vehicle_land_detected = {};
        //_distance ={};

    _counter = 0;


	_parameter_handles.p_tc = param_find("FW_P_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");

	_parameter_handles.r_tc = param_find("FW_R_TC");
	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");

	_parameter_handles.w_p = param_find("FW_WR_P");
	_parameter_handles.w_i = param_find("FW_WR_I");
	_parameter_handles.w_ff = param_find("FW_WR_FF");
	_parameter_handles.w_integrator_max = param_find("FW_WR_IMAX");
	_parameter_handles.w_rmax = param_find("FW_W_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.y_coordinated_min_speed = param_find("FW_YCO_VMIN");
	_parameter_handles.y_coordinated_method = param_find("FW_YCO_METHOD");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");

	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.vtol_type = param_find("VT_TYPE");



    _qd_offset.from_euler(0.0f, 1.57f, 0.0f);
    _qd.from_euler(0.0f, 0.0f, 0.0f);

    // changed from 4 to 5 (DM)
    _gain.Kp.reserve(5);
    _gain.Kd.reserve(5);
    _gain.Ki.reserve(5);

    _parameter_handles.Kp_aileron = param_find("ALLO");
    _parameter_handles.Kp_elevation = param_find("KP_ELEVATION");
    _parameter_handles.Kp_rudder = param_find("KP_RUDDER");
    _parameter_handles.Kp_vel_thrust = param_find("KP_VEL_THRUST");
    _parameter_handles.Kd_aileron = param_find("KD_AILERON");
    _parameter_handles.Kd_elevation = param_find("KD_ELEVATION");
    _parameter_handles.Kd_rudder = param_find("KD_RUDDER");
    _parameter_handles.Kd_vel_thrust = param_find("KD_VEL_THRUST");
    _parameter_handles.Ki_aileron = param_find("KI_AILERON");
    _parameter_handles.Ki_elevation = param_find("KI_ELEVATION");
    _parameter_handles.Ki_rudder = param_find("KI_RUDDER");
    _parameter_handles.Ki_vel_thrust = param_find("KI_VEL_THRUST");

	/* fetch initial parameter values */
	parameters_update();
}

//********************************************//
// DESTRUCTOR DE LA CLASSE                  ////
//********************************************//
DroneAquaTest::~DroneAquaTest()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	drone_aqua_test_control::g_control = nullptr;
}

//********************************************//
// PARAMETER UPDATE                         ////
//********************************************//
int
DroneAquaTest::parameters_update()
{

	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));

	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_coordinated_min_speed, &(_parameters.y_coordinated_min_speed));
	param_get(_parameter_handles.y_coordinated_method, &(_parameters.y_coordinated_method));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));

	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_ff, &(_parameters.w_ff));
	param_get(_parameter_handles.w_integrator_max, &(_parameters.w_integrator_max));
	param_get(_parameter_handles.w_rmax, &(_parameters.w_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.p_tc);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.r_tc);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_coordinated_method(_parameters.y_coordinated_method);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_parameters.w_p);
	_wheel_ctrl.set_k_i(_parameters.w_i);
	_wheel_ctrl.set_k_ff(_parameters.w_ff);
	_wheel_ctrl.set_integrator_max(_parameters.w_integrator_max);
	_wheel_ctrl.set_max_rate(math::radians(_parameters.w_rmax));


    param_get(_parameter_handles.Kp_aileron, &_gain.Kp[0]);
    param_get(_parameter_handles.Kp_elevation, &_gain.Kp[1]);
    param_get(_parameter_handles.Kp_rudder, &_gain.Kp[2]);
    param_get(_parameter_handles.Kp_vel_thrust, &_gain.Kp[3]);
    param_get(_parameter_handles.Kd_aileron,  &_gain.Kd[0]);
    param_get(_parameter_handles.Kd_elevation, & _gain.Kd[1]);
    param_get(_parameter_handles.Kd_rudder, &_gain.Kd[2]);
    param_get(_parameter_handles.Kd_vel_thrust,  &_gain.Kd[3]);
    param_get(_parameter_handles.Ki_aileron, &_gain.Ki[0]);
    param_get(_parameter_handles.Ki_elevation, &_gain.Ki[1]);
    param_get(_parameter_handles.Ki_rudder, &_gain.Ki[2]);
    param_get(_parameter_handles.Ki_vel_thrust, &_gain.Ki[3]);

	return OK;
}

//********************************************//
// POLLING VEHICULE CONTROL MODE            ////
//********************************************//
void
DroneAquaTest::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

//********************************************//
// POLLING MODE MANUEL                      ////
//********************************************//
void
DroneAquaTest::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

//********************************************//
// POLLING CAPTEUR ACCEL                    ////
//********************************************//
void
DroneAquaTest::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

//********************************************//
// POLLING SETPOINT                         ////
//********************************************//
void
DroneAquaTest::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

//********************************************//
// POLLING VEHICULE POSITION                ////
//********************************************//
void
DroneAquaTest::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

//********************************************//
// POLLING STATUS VEHICULE                  ////
//********************************************//
void
DroneAquaTest::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);

                // TEST CONTROL ACTUATEUR GROUP 2
                //_actuators2_id = ORB_ID(actuator_controls_1);

				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

//********************************************//
// POLLING LANDING DETECTED                 ////
//********************************************//
void
DroneAquaTest::vehicle_land_detected_poll()
{
	/* check if there is new status information */
	bool vehicle_land_detected_updated;
	orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

	if (vehicle_land_detected_updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

// POUR MODULE DU PERCHING WALL -> PAS UTILE
/*
void
DroneAquaTest::distance_poll()
{
	bool distance_updated;
	orb_check(_distance_sub, &distance_updated);

	if (distance_updated) {
		orb_copy(ORB_ID(distance_sensor), _distance_sub, &_distance);
	}
}
*/

//********************************************//
// SEQUENCE DE DÉCOLLAGE	                ////
//********************************************//
void
DroneAquaTest::drone_aqua_take_off(int *compteur)
{

}

// POUR MODULE DU PERCHING WALL -> PAS UTILE
void
DroneAquaTest::hover_control(const math::Quaternion &q)
{
	std::vector<float> outputs(4);

    calculation_error(q);
    discrete_derivative();
    discrete_integrate();

    // Calcul la valeur de chaque sortie
    for (int i = 0; i < (_outputs.size() - 1); ++i) {
        outputs[i] = ((_gain.Kp[i] * _euler_error(i)) + (_derivate(i) * _gain.Kd[i]) + (_integrate(i) * _gain.Ki[i]));

        // Conditions de staturation
        if (outputs[i] >= 1.0f) {
            outputs[i] = 1.0f;
        } else if (outputs[i] <= -1.0f) {
            outputs[i] = -1.0f;
        }
    }

    // Vertical velocity and position controller with switch at given pitch angle
    if (_wall_landing) {
        // Force l'élévation à 100% pour ralentir

        // Thrust feedback

        if (q.to_euler()(1) < 1.39f && _hover_speed_comm > -0.1f) { //&& _hover_speed_comm < 0.0f // making sure to not go back to 0.0f speed command
             _hover_speed_comm = 0.0f;
        } else {
            _hover_speed_comm = -0.2f;
           // outputs[0] = outputs[0] + 0.0f;
        }

        //_hover_speed_comm = 0.0f;

        outputs[3] = ((_gain.Kp[3] * _speed_error) + (_speed_derivate * _gain.Kd[3]) + _speed_integrate * _gain.Ki[3]) + 0.65f;

        if (outputs[3] < 0.23f){
                outputs[3] = 0.23f;
        }
            // Thrust feedback through integration of IMU measurements for vertical velocity
 } else if (_take_off) {
        if (_time < 1.0f) {
            //outputs[3] = 0.68f;
            _hover_speed_comm = 0.8f;
            //outputs[3] = 0.60f;
            outputs[3] = ((_gain.Kp[3] * _speed_error) + (_speed_derivate * _gain.Kd[3]) + _speed_integrate * _gain.Ki[3]) + 0.65f;
            if (outputs[3] > 1.0f)
                outputs[3] = 1.0f;
            //outputs[3] = 0.0f;
            else if (outputs[3] < 0.5f)
                outputs[3] = 0.5f;
            //outputs[3] = 0.0f;
        } else {
            //outputs[3] = 0.60f;
             _hover_speed_comm = 0.2f;
            outputs[3] = ((_gain.Kp[3] * _speed_error) + (_speed_derivate * _gain.Kd[3]) + _speed_integrate * _gain.Ki[3]) + 0.65f;
            if (outputs[3] > 1.0f)
                outputs[3] = 1.0f;
            //outputs[3] = 0.0f;
            else if (outputs[3] < 0.5f)
                outputs[3] = 0.5f;
            //outputs[3] = 0.0f;
        }
}

    // DONNE LES OUTPUTS DU
	_actuators.control[actuator_controls_s::INDEX_ROLL] = outputs[0];
    _actuators.control[actuator_controls_s::INDEX_PITCH] = outputs[1];
	_actuators.control[actuator_controls_s::INDEX_YAW] = outputs[2];
    _actuators.control[actuator_controls_s::INDEX_THROTTLE] = outputs[3];
}

// POUR MODULE DU PERCHING WALL -> PAS UTILE
void
DroneAquaTest::calculation_error(const math::Quaternion &q)
{
    _qm = q.conjugated();
    _qe = _qm * _qd;
    _euler_error = _qe.to_euler();

    /* Speed estimation */
    //AccelZ = -(-PixAccelX.*sin(PixPitch*pi/180) + PixAccelZ.*cos(PixPitch*pi/180) + 9.714);

    //float accelZ = -(-_ctrl_state.x_acc*(float)sin(q.to_euler()(1)) + _ctrl_state.z_acc*(float)cos(q.to_euler()(1)) + 9.714f); // g = 9.714


    float angle2 = q.to_euler()(1);
    float angle3 = q.to_euler()(0);

    float s2 = (float)sin(angle2);
    float c2 = (float)cos(angle2);
    float s3 = (float)sin(angle3);
    float c3 = (float)cos(angle3);

    float ax = _ctrl_state.x_acc;
    float ay = _ctrl_state.y_acc;
    float az = _ctrl_state.z_acc;

    float gi = 9.724f;

    float accelZ = -(-ax*s2 + ay*s3*c2 + az*c2*c3 + gi);


    _speed_z_axis_old = _speed_z_axis;
    _speed_z_axis = _speed_z_axis_old + accelZ * _dt; //

   // if (angle2 > 1.39f) {
        _speed_error = _hover_speed_comm - (_speed_z_axis); // float added ro _speed_z_axis as speed calculation bias
    //} else {
    //    _speed_error = _hover_speed_comm - _speed_z_axis;
   // }
}

void
DroneAquaTest::discrete_derivative()
{
    _derivate = (_euler_error - _euler_error_old) / _dt;
    _euler_error_old = _euler_error;

    _speed_derivate = (_speed_error - _speed_error_old) / _dt;
    _speed_error_old = _speed_error;

}

void
DroneAquaTest::discrete_integrate()
{
    _integrate = (_euler_error * _dt) + _integrate_old;
    _integrate_old = _integrate;

    _speed_integrate = (_speed_error * _dt) + _speed_integrate_old;
    _speed_integrate_old = _speed_integrate;

}

void
DroneAquaTest::task_main_trampoline(int argc, char *argv[])
{
	drone_aqua_test_control::g_control->task_main();
}

//********************************************//
// BOUCLE DE CONTROLE                       ////
//********************************************//
void
DroneAquaTest::task_main()
{

	///////////////////////////////////////////////////////////////////////////
	// pour publish dans wake_up_slave_info_2 / charging_info_2
	///////////////////////////////////////////////////////////////////////////
	_wake_up_slave_topic_2 = orb_advertise(ORB_ID(wake_up_slave_info_2), &report_wake_up_slave_2);
	_charging_info_topic_2 = orb_advertise(ORB_ID(charging_info_2), &report_charging_2);
	///////////////////////////////////////////////////////////////////////////

	/*
	 * do subscriptions ()
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
        //_distance_sub = orb_subscribe(ORB_ID(distance_sensor));

	///////////////////////////////////////////////////////////////////////////
	// pour subscribe a wake_up_slave_info / charging_info
	///////////////////////////////////////////////////////////////////////////
	_wake_up_slave_info_sub = orb_subscribe(ORB_ID(wake_up_slave_info));
	_charging_info_sub = orb_subscribe(ORB_ID(charging_info));	
	///////////////////////////////////////////////////////////////////////////

	// Set l'intervale à 200Hz ( boucle de controle )
	orb_set_interval(_ctrl_state_sub, 5);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();
	vehicle_land_detected_poll();
        //distance_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

        	static int loop_counter = 0;

	        // VARIABLES UTILES À LA MANOEUVRE DE DÉCOLLAGE

	        /*****************************************************/
	    
#if 0
	        static int compteur = 0;

	        static bool mode_seq0 = false;
	        static bool mode_seq1 = false;
	        static bool mode_seq2 = false;
	        static bool mode_seq3 = false;
	        static bool mode_seq4 = false;
	        static bool mode_seq5 = false;       
	        static bool mode_seq6 = false;
	        static bool mode_seq7 = false;
	        static bool mode_seq8 = false;
	        static bool mode_seq9 = false;
	        static bool mode_seq10 = false;
	        static bool mode_seq11 = false;

	        static bool flagidle = false;
#endif
        
	        /*****************************************************/

	        // SI PAS DUPDATE DANS 500ms -> LE PILOTE REPREND LES COMMANDES
		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

        	// UPDATE DES PARAMETRES SI EVENT DU 200 HZ
		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

        	// LORSQUE LUPDATE DES PARAMETRE SURVIENT -> ON VA DANS LE IF
		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();


			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			// uORB des control state (acc, gyro, compass, etc. qui viennent dun autre process)
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

            // POLLING DES PARAMETRES
            vehicle_status_poll();
            vehicle_accel_poll();
            vehicle_manual_poll();

            //****************************************************************************************************//
            //A FARE -> METTRE CETTE SÉQUENCE DANS UNE FONCTION APPROPRIÉE ET RENOMMER LE MODULE "AQUA_TAKE_OFF"
            //****************************************************************************************************//    

            // CES COMMANDES DE LAVION VIENNENT DE LA MANETTE (MODE SEMI MANUEL POUR LINSTANT)
            _actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y; //0.0f + _parameters.trim_roll;
            _actuators.control[actuator_controls_s::INDEX_PITCH] = _manual.x; //0.3f + _parameters.trim_pitch;
            _actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r; // 0.0f + _parameters.trim_yaw
            _actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z; // pass-through de la commande du trigger

#if 0

            // WAIT AVANT LA SEQUENCE (FALCULTATIF)
            if(mode_seq0)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f; 
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;                             	
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f;

                flagidle = true; // pour pas remettre le mode idle pendant la suite de la sequence

                if(compteur++ >= 400)
                {
                   compteur = 0;
                   mode_seq0 = false;
                   mode_seq1 = true;
                }
            }
            
            // COMMENCE À ACTIVER LE MUSCLE WIRE DU VERROU HORIZONTAL
            if(mode_seq1)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;  
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;                                 	
            	_actuators_airframe.control[INDEX_WIRE_POS_DOWN] = 1.0f;
            	_actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f;
				
                if(compteur++ >= 120) //timing optimal entre 140-170
                {
                   compteur = 0;
                   mode_seq1 = false;
                   mode_seq2 = true;             
                }               
            }

            // ACTIVE LE SERVO POUR REMONTER LE PIVOT EN CONTINUANT DACTIVER LE MEME MUSCLE WIRE
            if(mode_seq2)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;  
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;                                	
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = 1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = -0.4f;
               
                if(compteur++ >= 200)
                {
                   compteur = 0;
                   mode_seq2 = false;
                   mode_seq3 = true;
                }               
            }  

            // DÉSACTIVE LE MUSCLE WIRE ET FINI LA SEQUENCE POUR REMONTER LE PIVOT
            if(mode_seq3)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f; 
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;                              	
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = -0.4f;
             
                if(compteur++ >= 27)
                {
                   compteur = 0;
                   mode_seq3 = false;
                   mode_seq4 = true;
                }                              
            }    

          	// DÉSACTIVE TOUT UNE FOIS LE PIVOT REMONTÉ
            if(mode_seq4)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;  
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;                 
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f;
                
                if(compteur++ >= 800)
                {
                   compteur = 0;
                   mode_seq4 = false;
                   mode_seq5 = true;
                }                                     
            }     

            // REMET LE SERVO DU PIVOT DANS SA POSITION INITIALE
            if(mode_seq5)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;  
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;                         	
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.4f;
                
                if(compteur++ >= 158)
                {
                   compteur = 0;
                   mode_seq5 = false;
                   mode_seq6 = true;
                }                                
            }    

            // ARRETE LE SERVO DU PIVOT
            if(mode_seq6)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f;
                
                if(compteur++ >= 200) 
                {
                   compteur = 0;
                   mode_seq6 = false;
                   mode_seq7 = true;
                }                                             
            }           
                  
            // IDLE DU THRUST A 20% PENDANT 2 SEC
            if(mode_seq7)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.20f;
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f;

                if(compteur++ >= 400) // 2 sec (VALEUR = 400)
                {
                   compteur = 0;
                   mode_seq7 = false;
                   mode_seq8 = true;
                }
            }

            // FULL THROTTLE PENDANT 0.12 SEC
            if(mode_seq8)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f;

                if(compteur++ >= 24) // 0.12 sec (VALEUR = 24)
                {
                   compteur = 0;
                   mode_seq8 = false;
                   mode_seq9 = true;
                }
            }

            // ACTIVE LE MUSCLE WIRE UP POUR FAIRE BASCULER LA TETE A LHORIZONTAL
            if(mode_seq9)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = 1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f;

                if(compteur++ >= 16) // 0.20 sec (VALEUR = 40)
                {
                   compteur = 0;
                   mode_seq9 = false;
                   mode_seq10 = true;
                }
            }

            //ÉTEINT LE MUSCLE WIRE ET MAINTIENT FULL THROTTLE POUR UN CERTAIN TEMPS (A DETERMINER)
            if(mode_seq10)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f;

                if(compteur++ >= 400) // 2 sec
                {
                   compteur = 0;
                   mode_seq10 = false;
                   mode_seq11 = true;
                }
            }

            //****************************************************************************************************//
            //DONNE LA COMMANDE DU THROTTLE AU PILOTE (POUR LINSTANT)
            //ÉVENTUELLEMENT ON CHANGERA CETTE SÉQUENCE PAR UN KILL DE LA THREAD OU UN RETOUR EN MODE IDLE EN ATTENTE
            //DUNE COMMANDE DU COMMANDER INDIQUANT QUIL EST TEMPS DE FAIR ELA SÉQUENCE DE DÉCOLLAGE
            //****************************************************************************************************//
            if(mode_seq11)
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z; // pass-through de la commande du trigger
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f;
            }

            // COMMANDE DE ZÉRO SI VÉHICULE PAS ARMÉ
            if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _actuators.control[actuator_controls_s::INDEX_THROTTLE];

                if(flagidle == false)
                    mode_seq0 = true;

			} else {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f; // quant nuttx boot le thrust est a 0
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f; // quand nuttx boot on est certain quaucun muscle wire nest activé
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f; // quand nuttx boot on est certain quaucun muscle wire nest activé
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f; // le servo ne bouge pas

		        mode_seq0 = false;
		        mode_seq1 = false;
		        mode_seq2 = false;
		        mode_seq3 = false;
		        mode_seq4 = false;
		        mode_seq5 = false;       
		        mode_seq6 = false;
		        mode_seq7 = false;
		        mode_seq8 = false;
		        mode_seq9 = false;
        		mode_seq10 = false;
        		mode_seq11 = false;

                flagidle = false;

                compteur = 0;
			}

            // SÉCURITÉ -> MODE MANUEL ENCLENCHÉ
            if(_manual.mode_slot == manual_control_setpoint_s::MODE_SLOT_1) // si mode dans la position 1 (mode manuel pour linstant) -> éteint tout
            {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_UP] = -1.0f;
                _actuators_airframe.control[INDEX_WIRE_POS_DOWN] = -1.0f;
                _actuators_airframe.control[INDEX_SERVO_ROT] = 0.0f; // le servo ne bouge pas

		        mode_seq0 = false;
		        mode_seq1 = false;
		        mode_seq2 = false;
		        mode_seq3 = false;
		        mode_seq4 = false;
		        mode_seq5 = false;       
		        mode_seq6 = false;
		        mode_seq7 = false;
		        mode_seq8 = false;
		        mode_seq9 = false;
        		mode_seq10 = false;
        		mode_seq11 = false;

                compteur = 0;
            }
#endif

            // PUBLIE LA VALEUR DES ACTUATEURS 
            // DANS CETTE SÉQUENCE ON METTRA PROBABLEMENT LENVOIE DAUTRES VARIABLES
			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;
			_actuators_airframe.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp_sample = _ctrl_state.timestamp;

			/* publish the actuator controls */
			if (_actuators_0_pub != nullptr) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

			} else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}

			if (_actuators_2_pub != nullptr) {
				/* publish the actuator controls*/
				orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators_airframe);

			} else {
				/* advertise and publish */
				_actuators_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators_airframe);
			}
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_time = 0;
    _counter = 0;
}

int
DroneAquaTest::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("perching_wall",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1300,
					   (px4_main_t)&DroneAquaTest::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int drone_aqua_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: drone_aqua_test {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (drone_aqua_test_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		drone_aqua_test_control::g_control = new DroneAquaTest;

		if (drone_aqua_test_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != drone_aqua_test_control::g_control->start()) {
			delete drone_aqua_test_control::g_control;
			drone_aqua_test_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (drone_aqua_test_control::g_control == nullptr || !drone_aqua_test_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (drone_aqua_test_control::g_control == nullptr || !drone_aqua_test_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (drone_aqua_test_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete drone_aqua_test_control::g_control;
		drone_aqua_test_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (drone_aqua_test_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
