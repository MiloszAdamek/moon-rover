/*
 * ODrive.c
 *
 *  Created on: 07-Jul-2022
 *      Author: sidiyer27
 */


#include "ODrive.h"

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];


void Set_TX_Param(int AXIS_ID, int COMMAND_ID, int id_type, int frame_type, int data_length){
	TxHeader.Identifier = (AXIS_ID << 5) | COMMAND_ID;
	TxHeader.IdType = id_type;
	TxHeader.TxFrameType = frame_type;
	TxHeader.DataLength = data_length;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
}


void Set_Axis_Requested_State(Axis Axis, Axis_State state){
	Set_TX_Param(Axis.AXIS_ID, SET_AXIS_REQUESTED_STATE, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	unsigned int Requested_State = state;
	uint8_t *ptrToFloat;
	ptrToFloat = (uint8_t *)&Requested_State;
	TxData[0] = ptrToFloat[0];
	TxData[1] = ptrToFloat[1];
	TxData[2] = ptrToFloat[2];
	TxData[3] = ptrToFloat[3];
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Set_Axis_Requested_State");
	}
}

void Set_Input_Vel(Axis Axis, float vel, float torqueff){
	Set_TX_Param(Axis.AXIS_ID, SET_INPUT_VEL, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	uint8_t *ptrVel;
	ptrVel = (uint8_t *)&vel;
	uint8_t *ptrTor;
	ptrTor = (uint8_t *)&torqueff;
	TxData[0] = ptrVel[0];
	TxData[1] = ptrVel[1];
	TxData[2] = ptrVel[2];
	TxData[3] = ptrVel[3];
	TxData[4] = ptrTor[0];
	TxData[5] = ptrTor[1];
	TxData[6] = ptrTor[2];
	TxData[7] = ptrTor[3];
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Set_Input_Vel");
	}
}

void Clear_Errors(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, CLEAR_ERRORS, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_0);
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Clear_Errors");
	}
}

void Reboot_ODrive(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, REBOOT_ODRIVE, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_0);
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Reboot_ODrive");
	}
}

void Set_Controller_Modes(Axis Axis, Control_Mode ControlMode, Input_Mode InputMode){
	Set_TX_Param(Axis.AXIS_ID, SET_CONTROLLER_MODES, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	int Control = ControlMode;
	int Input = InputMode;
	uint8_t *ptrControl;
	ptrControl = (uint8_t *)&Control;
	uint8_t *ptrInput;
	ptrInput = (uint8_t *)&Input;
	TxData[0] = ptrControl[0];
	TxData[1] = ptrControl[1];
	TxData[2] = ptrControl[2];
	TxData[3] = ptrControl[3];
	TxData[4] = ptrInput[0];
	TxData[5] = ptrInput[1];
	TxData[6] = ptrInput[2];
	TxData[7] = ptrInput[3];
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Set_Controller_Modes");
	}
}

void Set_Input_Pos(Axis Axis, float Input_Pos, int Vel_FF, int Torque_FF){
	Set_TX_Param(Axis.AXIS_ID, SET_INPUT_POS, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	uint8_t *ptrPos;
	ptrPos = (uint8_t *)&Input_Pos;
	uint8_t *ptrVel;
	ptrVel = (uint8_t *)&Vel_FF;
	uint8_t *ptrTor;
	ptrTor = (uint8_t *)&Torque_FF;
	TxData[0] = ptrPos[0];
	TxData[1] = ptrPos[1];
	TxData[2] = ptrPos[2];
	TxData[3] = ptrPos[3];
	TxData[4] = ptrVel[0];
	TxData[5] = ptrVel[1];
	TxData[6] = ptrTor[0];
	TxData[7] = ptrTor[1];
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Set_Input_Pos");
	}
}

void Get_Encoder_Count(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, GET_ENCODER_COUNT, FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Get_Encoder_Count");
	}
}

void Set_Input_Torque(Axis Axis, float torque){
	Set_TX_Param(Axis.AXIS_ID, SET_INPUT_TORQUE, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
	uint8_t *ptrTor;
	ptrTor = (uint8_t *)&torque;
	TxData[0] = ptrTor[0];
	TxData[1] = ptrTor[1];
	TxData[2] = ptrTor[2];
	TxData[3] = ptrTor[3];
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Set_Input_Torque");
	}
}

void Get_Bus_Voltage_Current(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, GET_BUS_VOLTAGE_CURRENT, FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Get_Bus_Voltage_Current");
	}
}

void Get_IQ(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, GET_IQ, FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Get_IQ");
	}
}

void Set_Position_Gain(Axis Axis, float pos_gain){
	Set_TX_Param(Axis.AXIS_ID, SET_POSITION_GAIN, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
	uint8_t *ptrPos;
	ptrPos = (uint8_t *)&pos_gain;
	TxData[0] = ptrPos[0];
	TxData[1] = ptrPos[1];
	TxData[2] = ptrPos[2];
	TxData[3] = ptrPos[3];
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Set_Position_Gain");
	}
}

void Set_Vel_Gains(Axis Axis, float Vel_Gain, float Vel_Int_Gain){
	Set_TX_Param(Axis.AXIS_ID, SET_VEL_GAINS, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	uint8_t *ptrVelGain;
	ptrVelGain = (uint8_t *)&Vel_Gain;
	uint8_t *ptrVelIntGain;
	ptrVelIntGain = (uint8_t *)&Vel_Int_Gain;
	TxData[0] = ptrVelGain[0];
	TxData[1] = ptrVelGain[1];
	TxData[2] = ptrVelGain[2];
	TxData[3] = ptrVelGain[3];
	TxData[4] = ptrVelIntGain[0];
	TxData[5] = ptrVelIntGain[1];
	TxData[6] = ptrVelIntGain[2];
	TxData[7] = ptrVelIntGain[3];
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Set_Vel_Gains");
	}
}

void Set_Axis_Node_ID(Axis Axis, uint32_t node_id){
	Set_TX_Param(Axis.AXIS_ID, SET_AXIS_NODE_ID, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
	uint8_t *ptrNodeId;
	ptrNodeId = (uint8_t *)&node_id;
	TxData[0] = ptrNodeId[0];
	TxData[1] = ptrNodeId[1];
	TxData[2] = ptrNodeId[2];
	TxData[3] = ptrNodeId[3];
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) !=HAL_OK){
		Serial.println("Failed to send data via CAN, Set_Axis_Node_ID");
	}
}

void Set_Limits(Axis Axis, float vel_lim, float curr_lim){
	Set_TX_Param(Axis.AXIS_ID, SET_LIMITS, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	uint8_t *ptrVelLim;
	ptrVelLim = (uint8_t *)&vel_lim;
	uint8_t *ptrCurrLim;
	ptrCurrLim = (uint8_t *)&curr_lim;
	TxData[0] = ptrVelLim[0];
	TxData[1] = ptrVelLim[1];
	TxData[2] = ptrVelLim[2];
	TxData[3] = ptrVelLim[3];
	TxData[4] = ptrCurrLim[0];
	TxData[5] = ptrCurrLim[1];
	TxData[6] = ptrCurrLim[2];
	TxData[7] = ptrCurrLim[3];
	if(HAL_FDCAN_AddMessageToTxFifoQ(Axis.hfdcan, &TxHeader, TxData) != HAL_OK){
		Serial.println("Failed to send data via CAN, Set_Limits");
	}
}

void ODrive_RX_CallBack(Axis *AXIS,FDCAN_RxHeaderTypeDef* RxHeader, uint8_t* data){
	int32_t ID = 0;
	ID = RxHeader->Identifier;
	int32_t NODE_ID = (ID >> 5);
	int32_t CMD_ID = (ID & 0x01F);


	if(NODE_ID == AXIS->AXIS_ID){

		switch(CMD_ID){

			case ODRIVE_HEARTBEAT_MESSAGE:
				AXIS->AXIS_Error = (data[0] | data[1]<<8 | data[2]<<16 | data[3]<<24);
				AXIS->AXIS_Current_State = data[4];
				AXIS->Controller_Status = data[5];
				break;


			case ENCODER_ESTIMATES:
				uint32_t *ptrEncPos;
				ptrEncPos = (uint32_t *)&(AXIS->AXIS_Encoder_Pos);
				*ptrEncPos = (data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24));
				uint32_t *ptrEncVel;
				ptrEncVel = (uint32_t *)&(AXIS->AXIS_Encoder_Vel);
				*ptrEncVel = (data[4] + (data[5]<<8) + (data[6]<<16) + (data[7]<<24));
				break;

			case GET_ENCODER_COUNT:
				AXIS->AXIS_Encoder_Shadow = (data[0] | data[1]<<8 | data[2]<<16 | data[3]<<24);
				AXIS->AXIS_Encoder_CPR = (data[4] | data[5]<<8 | data[6]<<16 | data[7]<<24);
				break;

			case GET_BUS_VOLTAGE_CURRENT:
				uint32_t *ptrBusV;
				ptrBusV = (uint32_t *)&(AXIS->AXIS_Bus_Voltage);
				*ptrBusV = (data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24));
				uint32_t *ptrBusI;
				ptrBusI = (uint32_t *)&(AXIS->AXIS_Bus_Current);
				*ptrBusI = (data[4] + (data[5]<<8) + (data[6]<<16) + (data[7]<<24));
				break;


			case GET_IQ: 
				uint32_t *ptrIqSet;
				ptrIqSet = (uint32_t *)&(AXIS->AXIS_Iq_Setpoint);
				*ptrIqSet = (data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24));
				uint32_t *ptrIqMsr;
				ptrIqMsr = (uint32_t *)&(AXIS->AXIS_Iq_Measured);
				*ptrIqMsr = (data[4] + (data[5]<<8) + (data[6]<<16) + (data[7]<<24));
				break;


		}
	}
}
