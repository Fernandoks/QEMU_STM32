/*
 * commander.h
 *
 *  Created on: Apr 14, 2020
 *      Author: fernandoks
 */

#ifndef INC_COMMANDER_H_
#define INC_COMMANDER_H_


typedef enum
{
	COMMAND_LED_CTRL			= 0x50,
	COMMAND_SENSOR_READ       	= 0x51,
	COMMAND_LED_READ          	= 0x52,
	COMMAND_PRINT           	= 0x53,
	COMMAND_ID_READ				= 0x54,
	NUM_COMMANDS
} command_type;



typedef struct
{
	command_type command;
	void (*command_func) (void);
} CMDHeader_type;



void CMD_Led_Ctrl(void);
void CMD_Sensor_Read(void);
void CMD_Led_Read(void);
void CMD_Printl(void);
void CMD_ID_Read(void);


#endif /* INC_COMMANDER_H_ */
