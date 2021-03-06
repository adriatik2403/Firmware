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
 * @file icm20602_spi.cpp
 *
 * Driver for the ICM20602 connected via SPI.
 *
 * @author Adriatik Sermaxhaj
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_device.h>

#include "icm20602.h"
#include <board_config.h>

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#if defined(PX4_SPIDEV_ICM20602)
#  ifdef PX4_SPI_BUS_EXT
#    define EXTERNAL_BUS PX4_SPI_BUS_EXT
#  else
#    define EXTERNAL_BUS PX4_SPI_BUS_EXT
#  endif

/*
  The ICM20602 can only handle high SPI bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  SPI speed

 */
#define ICM20602_LOW_SPI_BUS_SPEED	1000*1000
#define ICM20602_HIGH_SPI_BUS_SPEED	11*1000*1000 /* will be rounded to 10.4 MHz, within margins for MPU6K */


device::Device *ICM20602_SPI_interface(int bus, bool external_bus);


class ICM20602_SPI : public device::SPI
{
public:
	ICM20602_SPI(int bus, spi_dev_e device);
	virtual ~ICM20602_SPI();

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

	virtual int	ioctl(unsigned operation, unsigned &arg);
protected:
	virtual int probe();

private:

	int _device_type;
	/* Helper to set the desired speed and isolate the register on return */

	void set_bus_frequency(unsigned &reg_speed_reg_out);
};

device::Device *
ICM20602_SPI_interface(int bus, bool external_bus)
{
	int cs = SPIDEV_NONE;
	device::Device *interface = nullptr;

	printf("BUS ==> %d  external ==> %b\n",bus,external_bus);

	if (external_bus) {
		cs = PX4_SPIDEV_ICM_20602_EXT;
	} else {
		cs = PX4_SPIDEV_ICM20602;
	}

	if (cs != SPIDEV_NONE) {

		interface = new ICM20602_SPI(bus, (spi_dev_e) cs);
	}

	return interface;
}

ICM20602_SPI::ICM20602_SPI(int bus, spi_dev_e device) :
	SPI("ICM20602", nullptr, bus, device, SPIDEV_MODE3, ICM20602_LOW_SPI_BUS_SPEED)
{
}

ICM20602_SPI::~ICM20602_SPI()
{
}

int
ICM20602_SPI::init()
{
	int ret;

	ret = SPI::init();

	if (ret != OK) {
		printf("SPI init failed\n");
		return -EIO;
	}

	return OK;
}

int
ICM20602_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case ACCELIOCGEXTERNAL:
#if defined(PX4_SPI_BUS_EXT)
		return _bus == PX4_SPI_BUS_EXT ? 1 : 0;
#else
		return 0;
#endif

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	default: {
			ret = -EINVAL;
		}
	}

	return ret;
}

void
ICM20602_SPI::set_bus_frequency(unsigned &reg_speed)
{
	/* Set the desired speed */

	set_frequency(ICM20602_IS_HIGH_SPEED(reg_speed) ? ICM20602_HIGH_SPI_BUS_SPEED : ICM20602_LOW_SPI_BUS_SPEED);

	/* Isoolate the register on return */

	reg_speed = ICM20602_REG(reg_speed);
}


int
ICM20602_SPI::write(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd[MPU_MAX_WRITE_BUFFER_SIZE];

	if (sizeof(cmd) < (count + 1)) {
		return -EIO;
	}

	/* Set the desired speed and isolate the register */

	set_bus_frequency(reg_speed);

	cmd[0] = reg_speed | DIR_WRITE;
	cmd[1] = *(uint8_t *)data;

	return transfer(&cmd[0], &cmd[0], count + 1);
}

int
ICM20602_SPI::read(unsigned reg_speed, void *data, unsigned count)
{
	/* We want to avoid copying the data of MPUReport: So if the caller
	 * supplies a buffer not MPUReport in size, it is assume to be a reg or reg 16 read
	 * and we need to provied the buffer large enough for the callers data
	 * and our command.
	 */
	uint8_t cmd[3] = {0, 0, 0};

	uint8_t *pbuff  =  count < sizeof(MPUReport) ? cmd : (uint8_t *) data ;


	if (count < sizeof(MPUReport))  {

		/* add command */

		count++;
	}

	set_bus_frequency(reg_speed);

	/* Set command */

	pbuff[0] = reg_speed | DIR_READ ;

	/* Transfer the command and get the data */

	int ret = transfer(pbuff, pbuff, count);

	if (ret == OK && pbuff == &cmd[0]) {

		/* Adjust the count back */

		count--;

		/* Return the data */

		memcpy(data, &cmd[1], count);

	}

	return ret == OK ? count : ret;
}

int
ICM20602_SPI::probe()
{
    printf("probe ==> ICM_WHOAMI_20602\n");
	uint8_t whoami = 0;

	read(MPUREG_WHOAMI, &whoami, 1);

	if(read(MPUREG_WHOAMI, &whoami, 1) > 0 && (whoami == ICM_WHOAMI_20602))
	{
	    printf("GOOD!!!! this is right\n");
	}
	else
	{
	    printf("NOT GOOD!!!! there is a problem sir!\n");
	}

	return (read(MPUREG_WHOAMI, &whoami, 1) > 0 && (whoami == ICM_WHOAMI_20602)) ? 0 : -EIO;
}

#endif // PX4_SPIDEV_MPU
