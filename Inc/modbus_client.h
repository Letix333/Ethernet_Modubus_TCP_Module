/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 */
#ifndef __TCP_ECHOSERVER_H__
#define __TCP_ECHOSERVER_H__

#include "inttypes.h"

extern uint8_t *Outputs;
extern uint8_t *Inputs;
extern uint16_t *Ainputs;
extern uint16_t *Registers;
extern uint8_t *Errors;
extern uint8_t Modbus_changed; 

#define MODBUS_CHANGED_WRITE_COILS 					0x01
#define MODBUS_CHANGED_WRITE_REGISTERS 			0x02

#define MODBUS_BROADCAST_ADDRESS    0


/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01 // DONE
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02 // DONE
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03 // DONE
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04 // DONE
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05 // DONE
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06 // DONE
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07 // DONE
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F // DONE
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10 // DONE
#define MODBUS_FC_REPORT_SLAVE_ID           0x11 // DONE
#define MODBUS_FC_MASK_WRITE_REGISTER       0x16 // DONE
#define MODBUS_FC_WRITE_AND_READ_REGISTERS  0x17 // DONE

/* Modbus error codes */
#define MODBUS_ILLEGAL_FUNCTION             0x01 // DONE
#define MODBUS_ILLEGAL_DATA_ADDRESS         0x02 // DONE
#define MODBUS_ILLEGAL_DATA_VALUE           0x03 // DONE
#define MODBUS_SLAVE_DEVICE_FAILURE         0x04 // DONE
#define MODBUS_SLAVE_DEVICE_BUSY 						0x06 // DONE

#define MSG_LENGTH_UNDEFINED 9999

#define MAX_registers 10
#define MAX_input_registers 6
#define MAX_coils 16
#define MAX_discrate_inputs 16
void tcp_echoserver_init(void);

/* REGISTER MAP */
/*
REGISTER[0-3] -> Registers contain refreshment time for discrate inputs,  4 bits for 1 input 0-15 cycles  when 0 means disabled.
REGISTER[4-5] -> Registers contain refreshment time for analog inputs,  4 bits for 1 input 0-15 cycles  when 0 means disabled.
REGISTER[6]   -> Register  contain  last read value from RS232
REGISTER[7]   -> Register  contain   value to write on RS232
REGISTER[8-9] -> DAC channel1 and Channel2 
*/

#endif /* __TCP_ECHOSERVER */
