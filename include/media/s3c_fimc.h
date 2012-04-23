/*
 * Samsung S3C SoC camera interface driver header
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd
 * Author of original S5P version: Sylwester Nawrocki, <s.nawrocki@samsung.com>
 * Modified for S3C64xx by: Tomasz Figa, <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef S3C_FIMC_H_
#define S3C_FIMC_H_

enum cam_bus_type {
	FIMC_ITU_601 = 1,
	FIMC_ITU_656,
};

struct i2c_board_info;

/**
 * struct s3c_fimc_isp_info - image sensor information required for host
 *			      interace configuration.
 *
 * @board_info: pointer to I2C subdevice's board info
 * @clk_frequency: frequency of the clock the host interface provides to sensor
 * @bus_type: determines bus type, MIPI, ITU-R BT.601 etc.
 * @i2c_bus_num: i2c control bus id the sensor is attached to
 * @flags: the parallel bus flags defining signals polarity (V4L2_MBUS_*)
 * @clk_id: index of the SoC peripheral clock for sensors
 */
struct s3c_fimc_isp_info {
	struct i2c_board_info *board_info;
	unsigned long clk_frequency;
	enum cam_bus_type bus_type;
	u16 i2c_bus_num;
	u16 flags;
	u8 clk_id;
};

/**
 * struct s3c_platform_fimc - camera host interface platform data
 *
 * @isp_info: properties of camera sensor required for host interface setup
 * @num_clients: the number of attached image sensors
 */
struct s3c_platform_fimc {
	struct s3c_fimc_isp_info *isp_info;
	int num_clients;
};

/*
 * v4l2_device notification id. This is only for internal use in the kernel.
 * Sensor subdevs should issue S3C_FIMC_TX_END_NOTIFY notification in single
 * frame capture mode when there is only one VSYNC pulse issued by the sensor
 * at begining of the frame transmission.
 */
#define S3C_FIMC_TX_END_NOTIFY _IO('e', 0)

#endif /* S3C_FIMC_H_ */
