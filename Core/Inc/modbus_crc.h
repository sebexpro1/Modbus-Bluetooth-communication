/*
 * modbus_crc.h
 *
 *  Created on: Sep 16, 2022
 *      Author: arunr
 */

#ifndef INC_MODBUS_CRC_H_
#define INC_MODBUS_CRC_H_
#define SLAVE_ID 7
#include "stdint.h"

static uint16_t Holding_Registers_Database[50];
uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);
void modbus_on_compare_array();
void modbus_off_compare_array();
void modbus_on_compare_array_AT();
void modbus_off_compare_array_AT();
int compareArrays(uint8_t arr1[], uint8_t arr2[], int size, int i_start);
int clearArray(uint8_t arr1[], int size);
void BTM222_init_reset();
void ATMode_on();
void ATMode_Pcom();
uint8_t readHoldingRegs (void);
uint8_t writeSingleReg (void);
void sendData (uint8_t *data, int size);

//static uint16_t Holding_Registers_Database[50];
#endif /* INC_MODBUS_CRC_H_ */
