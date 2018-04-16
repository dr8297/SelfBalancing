/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32f4_lis302dl_lis3dsh.h"

/* Private */
extern SPI_HandleTypeDef hspi1; // added by DRM to link this function to the main
extern void TM_LIS302DL_LIS3DSH_INT_WriteSPI(uint8_t* data, uint8_t addr, uint8_t count);
extern void TM_LIS302DL_LIS3DSH_INT_ReadSPI(uint8_t* data, uint8_t* addr, uint8_t count, spi_mode spi1);
//extern void TM_LIS302DL_LIS3DSH_INT_InitPins(void);
extern void TM_LIS302DL_LIS3DSH_INT_InitLIS3DSH(TM_LIS302DL_LIS3DSH_Sensitivity_t Sensitivity, TM_LIS302DL_LIS3DSH_Filter_t Filter);
extern void TM_LIS302DL_LIS3DSH_INT_InitLIS302DL(TM_LIS302DL_LIS3DSH_Sensitivity_t Sensitivity, TM_LIS302DL_LIS3DSH_Filter_t Filter);
extern void TM_LIS3DSH_INT_ReadAxes(TM_LIS302DL_LIS3DSH_t* Axes_Data);
extern void TM_LIS302DL_INT_ReadAxes(TM_LIS302DL_LIS3DSH_t* Axes_Data);
extern void TM_LIS302DL_LIS3DSH_INT_Delay(void);

TM_LIS302DL_LIS3DSH_Device_t TM_LIS302DL_LIS3DSH_INT_Device = TM_LIS302DL_LIS3DSH_Device_Error;
float TM_LIS3DSH_INT_Sensitivity;

/* Public */
TM_LIS302DL_LIS3DSH_Device_t TM_LIS302DL_LIS3DSH_Detect() {
	spi_mode mode=read_spi;
	uint8_t id;
	uint8_t write_buffer=LIS302DL_LIS3DSH_REG_WHO_I_AM;
	/* Delay on power up */
	//TM_LIS302DL_LIS3DSH_INT_Delay(); #I'll use a HAL delay
	HAL_Delay(100);
	/* Init SPI */
	//TM_LIS302DL_LIS3DSH_INT_InitPins(); #The cube MX function should init the pins
	/* Get ID */
	TM_LIS302DL_LIS3DSH_INT_ReadSPI(&id, &write_buffer , 1,mode );


	/* Check device */
	if (id == LIS302DL_ID) {
		/* Set device */
		TM_LIS302DL_LIS3DSH_INT_Device = TM_LIS302DL_LIS3DSH_Device_LIS302DL;
		/* Return device */
		return TM_LIS302DL_LIS3DSH_Device_LIS302DL;
	} else if (id == LIS3DSH_ID) {
		/* Set device */
		TM_LIS302DL_LIS3DSH_INT_Device = TM_LIS302DL_LIS3DSH_Device_LIS3DSH;
		/* Return device */;
		return TM_LIS302DL_LIS3DSH_Device_LIS3DSH;
	}
	
	/* Return Error */
	return TM_LIS302DL_LIS3DSH_Device_Error;
}

TM_LIS302DL_LIS3DSH_Device_t TM_LIS302DL_LIS3DSH_Init(TM_LIS302DL_LIS3DSH_Sensitivity_t Sensitivity, TM_LIS302DL_LIS3DSH_Filter_t Filter) {
	/* Init pinout and SPI */
	//TM_LIS302DL_LIS3DSH_INT_InitPins();
	/* Some delay */
	TM_LIS302DL_LIS3DSH_INT_Delay();
	/* Detect proper device and init it */
	if (TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS302DL) {
		/* Init sequence for LIS302DL */
		TM_LIS302DL_LIS3DSH_INT_InitLIS302DL(Sensitivity, Filter);
		/* Return device */
		return TM_LIS302DL_LIS3DSH_Device_LIS302DL;
	} else if (TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS3DSH) {
		/* Init sequence for LIS3DSH */
		TM_LIS302DL_LIS3DSH_INT_InitLIS3DSH(Sensitivity, Filter);
		/* Return device */
		return TM_LIS302DL_LIS3DSH_Device_LIS3DSH;
	}
	
	/* Error detection */
	TM_LIS302DL_LIS3DSH_INT_Device = TM_LIS302DL_LIS3DSH_Device_Error;
	/* Return Error */
	return TM_LIS302DL_LIS3DSH_Device_Error;
}

TM_LIS302DL_LIS3DSH_Device_t TM_LIS302DL_LIS3DSH_ReadAxes(TM_LIS302DL_LIS3DSH_t* Axes_Data) {
	if (TM_LIS302DL_LIS3DSH_INT_Device == TM_LIS302DL_LIS3DSH_Device_LIS302DL) {
		/* Init sequence for LIS302DL */
		TM_LIS302DL_INT_ReadAxes(Axes_Data);
		/* Return device */
		return TM_LIS302DL_LIS3DSH_Device_LIS302DL;
	} else if (TM_LIS302DL_LIS3DSH_INT_Device == TM_LIS302DL_LIS3DSH_Device_LIS3DSH) {
		/* Init sequence for LIS3DSH */
		TM_LIS3DSH_INT_ReadAxes(Axes_Data);
		/* Return device */
		return TM_LIS302DL_LIS3DSH_Device_LIS3DSH;
	}
	/* Return Error */
	return TM_LIS302DL_LIS3DSH_Device_Error;
}

/* Private */

/*
void TM_LIS302DL_LIS3DSH_INT_InitPins(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	static uint8_t initialized = 0;
	if (initialized) {
		return;
	}
	
	Initialize SPI
	TM_SPI_Init(LIS302DL_LIS3DSH_SPI, LIS302DL_LIS3DSH_SPI_PINSPACK);
	
	 Enable clock for CS port
	RCC_AHB1PeriphClockCmd(LIS302DL_LIS3DSH_CS_RCC, ENABLE);
	
	Configure CS pin
	GPIO_InitStruct.Pin = LIS302DL_LIS3DSH_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.OType = GPIO_OType_PP;
	GPIO_InitStruct.PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.Speed = GPIO_Speed_25MHz;
	GPIO Init
	GPIO_Init(LIS302DL_LIS3DSH_CS_PORT, &GPIO_InitStruct);
	
 	 	CS HIGH
	LIS302DL_LIS3DSH_CS_HIGH;
	
	initialized = 1;
}
*/

void TM_LIS302DL_LIS3DSH_INT_WriteSPI(uint8_t* data, uint8_t addr, uint8_t count) {
	/* Start SPI transmission */
	
	LIS302DL_LIS3DSH_CS_LOW;

	if (count > 1 && TM_LIS302DL_LIS3DSH_INT_Device == TM_LIS302DL_LIS3DSH_Device_LIS302DL) {
		/* Add autoincrement bit */
		/* Only LIS302DL device */
		addr |= 0x40;
	}
	
	/* Send address */
	//TM_SPI_Send(LIS302DL_LIS3DSH_SPI, addr);
	/* Send data */
	//TM_SPI_WriteMulti(LIS302DL_LIS3DSH_SPI, data, count);
	
	/* Stop SPI transmission */
	LIS302DL_LIS3DSH_CS_HIGH;
}

void TM_LIS302DL_LIS3DSH_INT_ReadSPI(uint8_t* data, uint8_t* write_buffer, uint8_t count, spi_mode mode) {
	/* Start SPI transmission */
	uint16_t size = count+1; // SPI interface read at least 16 bits
	uint16_t timeout =100;

	uint8_t  data1[2];
	//uint8_t local_addr=addr;
	data1[0]=0; // This is a dummy byte. HAL might need to be modified
	data1[1]=0;

	LIS302DL_LIS3DSH_CS_LOW;
	//HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);
	/* This step can be written more eficiently by manipulating just the bit7 of the write_buffer[0]  */
	if (mode== write_spi) {
		*write_buffer &= 0x7f; //  This set the the RnW bit to read
	} else {
		*write_buffer |= 0x80;
	}

	if (count > 1) 		*write_buffer |= 0x40; 		/* Add autoincrement bit */
	
	// Transmit and Receive address
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t  *) write_buffer, (uint8_t *) &data1, size, timeout);
	/* Send address */
	/*TM_SPI_Send(LIS302DL_LIS3DSH_SPI, addr);*/
	//LIS302DL_LIS3DSH_CS_HIGH;
	//HAL_Delay(10);
	//LIS302DL_LIS3DSH_CS_LOW;
	/* Receive data */
	//TM_SPI_ReadMulti(LIS302DL_LIS3DSH_SPI, data, 0x00, count);
	//HAL_SPI_Receive(&hspi1, data1, (uint16_t) 1, (uint32_t) 100);
	/* Stop SPI transmission */
	LIS302DL_LIS3DSH_CS_HIGH;
	*data = data1[1]; // data is written on second byte

}

void TM_LIS302DL_LIS3DSH_INT_InitLIS3DSH(TM_LIS302DL_LIS3DSH_Sensitivity_t Sensitivity, TM_LIS302DL_LIS3DSH_Filter_t Filter) {
	// prepare the array of data to be written
	spi_mode mode=write_spi;
	uint8_t data_read[4];
	uint8_t data_write[4];
	data_write[0]= 0x20; // adress 20h define as LIS3DSH_CTRL_REG4_ADDR (however on datasheet is given as CTRL_REG1), RnW and MnS set to zero
	uint8_t count=1; // write 1 byte, but in reality the spi interface will send two bytes.


	/* Set data, I think data rate has been set to 200Hz, but in normal mode BW(Hz) is set to 100 */
	//temp =  LIS3DSH_DATARATE_100 | LIS3DSH_XYZ_ENABLE;
	//setting normal mode, CTRL_REG1[3]=0 and CTRL_REG4[3]=0
	data_write[1] = LIS3DSH_DATARATE_100 | LIS3DSH_XYZ_ENABLE;
	//temp |= LIS3DSH_SERIALINTERFACE_4WIRE | LIS3DSH_SELFTEST_NORMAL;
	count = 1;
	TM_LIS302DL_LIS3DSH_INT_ReadSPI( (uint8_t *) &data_read, (uint8_t *) &data_write, count, mode);
	
	data_write[0]= 0x23; //CTRL_REG4, bit1 and bit0 controls self-test and SPI serial mode
	/* Set sensitivity, this paramter is set by register CTRL_REG4 (0x23) */
	if (Sensitivity == TM_LIS3DSH_Sensitivity_2G) {
		data_write[1] =  LIS3DSH_FULLSCALE_2;
		TM_LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_06G;
	} else if (Sensitivity == TM_LIS3DSH_Sensitivity_4G) {
		data_write [1] = LIS3DSH_FULLSCALE_4;
		TM_LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_12G;
	} else if (Sensitivity == TM_LIS3DSH_Sensitivity_6G) {
		data_write[1] = LIS3DSH_FULLSCALE_6;
		TM_LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_18G;
	} else if (Sensitivity == TM_LIS3DSH_Sensitivity_8G) {
		data_write[1] = LIS3DSH_FULLSCALE_8;
		TM_LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_24G;
	} else if (Sensitivity == TM_LIS3DSH_Sensitivity_16G) {
		data_write[1] = LIS3DSH_FULLSCALE_16;
		TM_LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_73G;
	} else {
		return;
	}
	
	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t *) &data_read,(uint8_t *) &data_write, count,mode);

	// Refer to page 16 of data-sheet. The sensor must operate on one of the theree modes: high, normal, low-power
	/* Set filter
	if (Filter == TM_LIS3DSH_Filter_800Hz) {
		temp |= (LIS3DSH_FILTER_BW_800 << 8);
	} else if (Filter == TM_LIS3DSH_Filter_400Hz) {
		temp |= (LIS3DSH_FILTER_BW_400 << 8);
	} else if (Filter == TM_LIS3DSH_Filter_200Hz) {
		temp |= (LIS3DSH_FILTER_BW_200 << 8);
	} else if (Filter == TM_LIS3DSH_Filter_50Hz) {
		temp |= (LIS3DSH_FILTER_BW_50 << 8);
	} else {
		return;
	}
	*/
	
	/* Configure MEMS: power mode(ODR) and axes enable
	tmpreg = (uint8_t) (temp);

	 Write value to MEMS CTRL_REG4 register, documentation shows that address 0x20h is CTRL_REG1
	TM_LIS302DL_LIS3DSH_INT_WriteSPI(&tmpreg, LIS3DSH_CTRL_REG4_ADDR, 1);

	HAL_SPI_Transmit(&hspi1, &data,  (uint16_t) 2, (uint32_t) 100);

	 Configure MEMS: full scale and self test
	tmpreg = (uint8_t) (temp >> 8);

	 Write value to MEMS CTRL_REG5 register
	TM_LIS302DL_LIS3DSH_INT_WriteSPI(&tmpreg, LIS3DSH_CTRL_REG5_ADDR, 1);
	*/
}

void TM_LIS302DL_LIS3DSH_INT_InitLIS302DL(TM_LIS302DL_LIS3DSH_Sensitivity_t Sensitivity, TM_LIS302DL_LIS3DSH_Filter_t Filter) {
	uint16_t ctrl;
	spi_mode mode=write_spi;
	uint8_t write_buffer = LIS302DL_CTRL_REG2_ADDR;
	/* Reboot */
	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t *)&ctrl, &write_buffer , 1,mode);
	ctrl |= LIS302DL_BOOT_REBOOTMEMORY;
	//TM_LIS302DL_LIS3DSH_INT_WriteSPI((uint8_t *)&ctrl, write_buffer, 1);
	
	/* Init settings */
	ctrl = (uint16_t) (LIS302DL_DATARATE_100 | LIS302DL_LOWPOWERMODE_ACTIVE | LIS302DL_SELFTEST_NORMAL | LIS302DL_XYZ_ENABLE);
	if (Sensitivity == TM_LIS302DL_Sensitivity_2_3G) {
		ctrl |= (uint16_t) LIS302DL_FULLSCALE_2_3;
		TM_LIS3DSH_INT_Sensitivity = LIS302DL_SENSITIVITY_2_3G;
	} else if (Sensitivity == TM_LIS302DL_Sensitivity_9_2G) {
		ctrl |= (uint16_t) LIS302DL_FULLSCALE_9_2;
		TM_LIS3DSH_INT_Sensitivity = LIS302DL_SENSITIVITY_9_2G;
	} else {
		return;
	}
	/* Write settings */
	TM_LIS302DL_LIS3DSH_INT_WriteSPI((uint8_t *)&ctrl, LIS302DL_CTRL_REG1_ADDR, 1);
	
	/* Read filter */
	TM_LIS302DL_LIS3DSH_INT_WriteSPI((uint8_t *)&ctrl, LIS302DL_CTRL_REG2_ADDR, 1);
	ctrl &= (uint8_t) ~(LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER | LIS302DL_HIGHPASSFILTER_LEVEL_3 | LIS302DL_HIGHPASSFILTERINTERRUPT_1_2);
	/* Set filter */
    ctrl |= (uint8_t) (LIS302DL_HIGHPASSFILTERINTERRUPT_1_2 | LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER);
	/* Set filter value */
	if (Filter == TM_LIS302DL_Filter_2Hz) {
		ctrl |= (uint8_t) LIS302DL_HIGHPASSFILTER_LEVEL_0;
	} else if (Filter == TM_LIS302DL_Filter_1Hz) {
		ctrl |= (uint8_t) LIS302DL_HIGHPASSFILTER_LEVEL_1;
	} else if (Filter == TM_LIS302DL_Filter_500mHz) {
		ctrl |= (uint8_t) LIS302DL_HIGHPASSFILTER_LEVEL_2;
	} else if (Filter == TM_LIS302DL_Filter_250mHz) {
		ctrl |= (uint8_t) LIS302DL_HIGHPASSFILTER_LEVEL_3;
	} else {
		return;
	}
	/* Write settings */
	TM_LIS302DL_LIS3DSH_INT_WriteSPI((uint8_t *)&ctrl, LIS302DL_CTRL_REG2_ADDR, 1);
}

void TM_LIS3DSH_INT_ReadAxes(TM_LIS302DL_LIS3DSH_t *Axes_Data) {
	spi_mode mode=write_spi;
	int8_t buffer[6];

	int8_t write_buffer=LIS3DSH_OUT_X_L_ADDR;

	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t *) &buffer[0], (uint8_t *) &write_buffer , 1,mode);

	// you have to edit it later on

	/*TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t*)&buffer[1], LIS3DSH_OUT_X_H_ADDR, 1,mode);
	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t*)&buffer[2], LIS3DSH_OUT_Y_L_ADDR, 1,mode);
	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t*)&buffer[3], LIS3DSH_OUT_Y_H_ADDR, 1,mode);
	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t*)&buffer[4], LIS3DSH_OUT_Z_L_ADDR, 1,mode);
	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t*)&buffer[5], LIS3DSH_OUT_Z_H_ADDR, 1,mode); */
	/* Set axes */
	Axes_Data->X = (int16_t)((buffer[1] << 8) + buffer[0]) * TM_LIS3DSH_INT_Sensitivity;
	Axes_Data->Y = (int16_t)((buffer[3] << 8) + buffer[2]) * TM_LIS3DSH_INT_Sensitivity;
	Axes_Data->Z = (int16_t)((buffer[5] << 8) + buffer[4]) * TM_LIS3DSH_INT_Sensitivity;
}

void TM_LIS302DL_INT_ReadAxes(TM_LIS302DL_LIS3DSH_t* Axes_Data) {
	spi_mode mode=read_spi;
	int8_t buffer[3];
	int16_t SwitchXY;
	int8_t write_buffer=LIS302DL_OUT_X_ADDR;


	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t *) &buffer[0], (uint8_t *) &write_buffer, 1,mode);
	write_buffer=LIS302DL_OUT_Y_ADDR;
	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t *) &buffer[1], (uint8_t *) &write_buffer, 1,mode);
	write_buffer=LIS302DL_OUT_Z_ADDR;
	TM_LIS302DL_LIS3DSH_INT_ReadSPI((uint8_t * )&buffer[2], (uint8_t *) &write_buffer, 1,mode);
	
	/* Set axes */
	Axes_Data->X = (int16_t) (buffer[0]) * TM_LIS3DSH_INT_Sensitivity;
	Axes_Data->Y = (int16_t) (buffer[1]) * TM_LIS3DSH_INT_Sensitivity;
	Axes_Data->Z = (int16_t) (buffer[2]) * TM_LIS3DSH_INT_Sensitivity;	
	/* Switch axes */
	SwitchXY  = Axes_Data->X;
	Axes_Data->X = Axes_Data->Y;
	Axes_Data->X = -SwitchXY;
}

void TM_LIS302DL_LIS3DSH_INT_Delay(void) {
	uint32_t delay = 1000000;
	while (delay--);
}

