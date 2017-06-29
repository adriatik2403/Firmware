/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file perching_wall_params.c
 *
 * Parameters defined by the perching_wall control task.
 *
 * @author Thomas Courteau <thomas.robichaud.courteau@gmail.com>
 */

#include <systemlib/param/param.h>

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Proportionnal gain for aileron
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KP_AILERON, 3.701f);

/**
 * Proportionnal gain for elevation
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KP_ELEVATION, 2.228f);

/**
 * Proportionnal gain for rudder
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KP_RUDDER, 2.5f);

/**
 * Proportionnal gain for thrust
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KP_VEL_THRUST, 0.1f);

/**
 * Derivate gain for aileron
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KP_POS_THRUST, 0.1f);

/**
 * Derivate gain for aileron
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KD_AILERON, 0.25f);

/**
 * Derivate gain for elevation
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KD_ELEVATION, 0.19f);

/**
 * Derivate gain for rudder
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KD_RUDDER, 0.2f);

/**
 * Derivate gain for thrust
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KD_VEL_THRUST, 0.0f);

/**
 * Integrate gain for aileron
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KD_POS_THRUST, 0.0f);

/**
 * Integrate gain for aileron
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KI_AILERON, 0.0f);

/**
 * Integrate gain for elevation
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KI_ELEVATION, 0.0f);

/**
 * Integrate gain for rudder
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KI_RUDDER, 0.0f);

/**
 * Integrate gain for thrust
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(KI_VEL_THRUST, 0.0f);

PARAM_DEFINE_FLOAT(KI_POS_THRUST, 0.0f);






