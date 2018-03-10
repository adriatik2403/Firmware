/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file run_cam.c
 * Control for RunCam Split 2
 *
 * @author Adriatik Sermaxhaj <adri.sermax@gmail.com>
 * https://github.com/ManagementCenterInnsbruck/XMC_Cleanflight/blob/1a326ac8096c3eba6d6b2e0b22144bcacce1ecf3/src/main/io/rcsplit.c
 */

#include <px4_tasks.h>
#include <px4_posix.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <termios.h>

// packet header and tail
#define RCSPLIT_PACKET_HEADER    0x55
#define RCSPLIT_PACKET_CMD_CTRL  0x01
#define RCSPLIT_PACKET_TAIL      0xaa

typedef enum {
    RCSPLIT_STATE_UNKNOWN = 0,
    RCSPLIT_STATE_INITIALIZING,
    RCSPLIT_STATE_IS_READY,
} rcsplitState_e;

// the commands of RunCam Split serial protocol
typedef enum {
    RCSPLIT_CTRL_ARGU_INVALID = 0x0,
    RCSPLIT_CTRL_ARGU_WIFI_BTN = 0x1,
    RCSPLIT_CTRL_ARGU_POWER_BTN = 0x2,
    RCSPLIT_CTRL_ARGU_CHANGE_MODE = 0x3,
    RCSPLIT_CTRL_ARGU_WHO_ARE_YOU = 0xFF,
} rcsplit_ctrl_argument_e;

__EXPORT int            run_cam_main        (int argc, char *argv[]);
void                    sendCtrlCommand     (int port,rcsplit_ctrl_argument_e argument);
int                     rcSplitInit         (char *uart_name);
uint8_t                 crc_high_first      (uint8_t *ptr, uint8_t len);
rcsplit_ctrl_argument_e rc_split_cmd        (char *cmd);
int                     enable_flow_control (bool enabled,int _uart_fd);
int                     set_baud            (int _uart_fd, char *_device);

// communicate with camera device variables
rcsplitState_e cameraState = RCSPLIT_STATE_UNKNOWN;


uint8_t crc_high_first(uint8_t *ptr, uint8_t len)
{
    uint8_t i;
    uint8_t crc=0x00;
    while (len--) {
        crc ^= *ptr++;
        for (i=8; i>0; --i) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return (crc);
}


void sendCtrlCommand(int port,rcsplit_ctrl_argument_e argument)
{
    if (!port)
        return ;

    uint8_t uart_buffer[5] = {0};
    uint8_t crc = 0;

    uart_buffer[0] = RCSPLIT_PACKET_HEADER;
    uart_buffer[1] = RCSPLIT_PACKET_CMD_CTRL;
    uart_buffer[2] = argument;
    uart_buffer[3] = RCSPLIT_PACKET_TAIL;
    crc = crc_high_first(uart_buffer, 4);

    // build up a full request [header]+[command]+[argument]+[crc]+[tail]
    uart_buffer[3] = crc;
    uart_buffer[4] = RCSPLIT_PACKET_TAIL;

    // write to device
    write(port, uart_buffer, 5);
}


int rcSplitInit(char *uart_name)
{
	// assuming NuttShell is on UART1 (/dev/ttyS0) /
	int _uart_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY); //

	if (_uart_fd < 0) {
		printf("ERROR opening UART %s, aborting..\n", uart_name);
		return _uart_fd;

	} else {
		printf("Writing to UART %s\n", uart_name);
	}

    _uart_fd = set_baud(_uart_fd,uart_name);
    if (_uart_fd < 0) {
        return _uart_fd;
    }

    cameraState = RCSPLIT_STATE_IS_READY;

    return _uart_fd;
}


int set_baud(int _uart_fd, char *_device)
{
    // set baud rate
    int speed = B115200;
    struct termios uart_config;
    tcgetattr(_uart_fd, &uart_config);
    // clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    /* Set baud rate */
    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
        warnx("ERR SET BAUD %s\n", _device);
        close(_uart_fd);
        return -1;
    }

    if ((tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
        PX4_WARN("ERR SET CONF %s\n", _device);
        px4_close(_uart_fd);
        return -1;
    }

    /* setup output flow control */
    if (enable_flow_control(false,_uart_fd)) {
        PX4_WARN("hardware flow disable failed");
    }

    return _uart_fd;
}

int enable_flow_control(bool enabled,int _uart_fd)
{
    struct termios uart_config;

    int ret = tcgetattr(_uart_fd, &uart_config);

    if (enabled) {
        uart_config.c_cflag |= CRTSCTS;

    } else {
        uart_config.c_cflag &= ~CRTSCTS;
    }

    ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

    /*
    if (!ret) {
        _flow_control_enabled = enabled;
    }
    */

    return ret;
}

rcsplit_ctrl_argument_e rc_split_cmd(char *cmd){
    /*
    * RCSPLIT_CTRL_ARGU_WIFI_BTN = 0x1,
    */
    if (!strcmp(cmd, "wifi")) {
        return RCSPLIT_CTRL_ARGU_WIFI_BTN;
    }
    /*
    * RCSPLIT_CTRL_ARGU_POWER_BTN = 0x2,
    */
    if (!strcmp(cmd, "power") ||
        !strcmp(cmd, "start") ||
        !strcmp(cmd, "stop") ||
        !strcmp(cmd, "snap")) {
        return RCSPLIT_CTRL_ARGU_POWER_BTN;
    }
    /*
    * RCSPLIT_CTRL_ARGU_CHANGE_MODE = 0x3,
    */
    if (!strcmp(cmd, "mode")) {
        return RCSPLIT_CTRL_ARGU_CHANGE_MODE;
    }
    /*
    * RCSPLIT_CTRL_ARGU_WHO_ARE_YOU = 0xFF,
    */
    if (!strcmp(cmd, "who")) {
        return RCSPLIT_CTRL_ARGU_WHO_ARE_YOU;
    }
    return RCSPLIT_CTRL_ARGU_WHO_ARE_YOU;
}
/*
run_cam who
run_cam wifi
run_cam power
run_cam mode
*/

int run_cam_main(int argc, char *argv[])
{
    // set BR = 115200
    printf("(who | wifi | power | start | stop | snap | mode)");
    // input handling
	char *uart_name = "/dev/ttyS3";
    rcsplit_ctrl_argument_e split_cmd = RCSPLIT_CTRL_ARGU_WHO_ARE_YOU;

	if (argc > 2) { 
        split_cmd = rc_split_cmd(argv[1]);
        uart_name = argv[2]; 
    }else if(argc > 1){ 
        split_cmd = rc_split_cmd(argv[1]);
    }  


	int test_uart = rcSplitInit(uart_name);

	if (test_uart < 0) {
		return test_uart;
	}

	uint64_t start_time = hrt_absolute_time();


    sendCtrlCommand(test_uart,split_cmd);
    //usleep(1000);


	int interval = hrt_absolute_time() - start_time;

	printf("%d ms on UART %s\n", interval / 1000, uart_name);

	close(test_uart);

	PX4_INFO("exiting");

	return 0;
}