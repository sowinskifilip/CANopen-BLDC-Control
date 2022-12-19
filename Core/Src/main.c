/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//UART VARIABLES
const uint8_t sInitCommand[] = "INIT";
uint8_t sUserMessage[4];
const uint8_t sErrorMessage[] = "UART ERROR\r\n";
int length;
uint8_t data_msg[16];

const uint8_t sSerialStart[] = "STSE";
const uint8_t sGeneralStop[] = "HALT";
const uint8_t sReset[] = "RSET";
const uint8_t sContinue[] = "CONT";

//MODE SELECTION
const uint8_t sSingle_mode[] = "SNGL";
const uint8_t sSerial_mode[] = "SERL";

//LEG SELECTION
const uint8_t sRight[] = "RGHT";
const uint8_t sLeft[] = "LEFT";

//MINMAX VALUES
const uint32_t iMax_angle = 90;
const uint32_t iMin_angle = 0;

//STATE VALUES
volatile uint8_t iFlag = 0;
volatile uint8_t iMode = 100; // default:100, 0 - SINGLE MOTION MODE, 1 - SERIAL MOTION MODE
volatile uint8_t iSelected_leg = 100; // default:100, 0 - LEFT LEG, 1 - RIGHT LEG

//CAN VARIABLES
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

CAN_FilterTypeDef CANFilter;

//STATE MACHINE VARIABLES
uint8_t iMachineStatus = 100;
uint8_t iHomingStatus = 100;
uint8_t iSingleMachineStatus = 100;
uint8_t iSerialMachineStatus = 100;

//ENCODER VARIABLES
uint16_t iEncCountReal=0;
uint16_t iEncCount=0;
const uint16_t iEncCountsNumber = 1000;
float fEncAngle=0;
float fEncDegPerCount = 0.36; //counts per rotation = 40 -> 360 degrees / 40 counts = 9 deg/count
float fEncAngleTemp = 0;

//MOVE ABSOLUTE VARIABLES
uint32_t iPosition;

//SERIAL MODE VARIABLES
uint8_t iSerialCounter = 100;
uint32_t iSerialRange = 100;
uint8_t iSerialReps = 100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//ERROR SIGNALIZATION
void fnLEDsErrorState(){
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
}


//INIT FUNCTION
void fnInit(){
	switch(iMachineStatus){

	case 0: //RESET PDO
		TxHeader.StdId = 0x000;
		TxHeader.DLC = 2;
		TxData[0] = 0x82;
		TxData[1] = 0x0A;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 5;
			HAL_UART_Transmit(&huart3, "C000", 4, 100);
		}
		break;

	case 5: //SET PDO
		TxHeader.StdId = 0x000;
		TxHeader.DLC = 2;
		TxData[0] = 0x01;
		TxData[1] = 0x0A;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 10;
			HAL_UART_Transmit(&huart3, "C005", 4, 100);
		}
		break;

	case 10: //SHUTDOWN
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x06;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 20;
			HAL_UART_Transmit(&huart3, "C010", 4, 100);
		}
		break;

	case 20://SWITCH ON
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x07;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 25;
			HAL_UART_Transmit(&huart3, "C020", 4, 100);
		}
		break;

	case 25://ENABLE OPERATION
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x0F;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 30;
			HAL_UART_Transmit(&huart3, "C025", 4, 100);
		}
		break;

	case 30://POSITION MODE
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x60;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x01;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 40;
			HAL_UART_Transmit(&huart3, "C030", 4, 100);
		}
		break;

	case 40:// POSITION 0
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;

		TxData[0] = 0x22;
		TxData[1] = 0x7A;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 50;
			HAL_UART_Transmit(&huart3, "P000", 4, 100);
		}
		break;

	case 50://START SUPPLY
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x1F;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 55;
			HAL_UART_Transmit(&huart3, "C050", 4, 100);
		}
		break;

	case 55://SET POINT ACK CHECK
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x40;
		TxData[1] = 0x41;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else if (RxData[5] & 0b10000) {
			iMachineStatus = 60;
			HAL_UART_Transmit(&huart3, "C055", 4, 100);
		}
		break;

	case 60://STOP SUPPLY
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x0F;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 70;
			HAL_UART_Transmit(&huart3, "C060", 4, 100);
		}
		break;

	case 70://TARGET REACHED CHECK
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x40;
		TxData[1] = 0x41;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else if (RxData[5] & 0b00100) {
			iMachineStatus = 80;
			HAL_UART_Transmit(&huart3, "C070", 4, 100);
		}
		break;

	case 80://READ STATUS CHECK
		if (RxData[4] == 39) {
			iMachineStatus = 1;
			iHomingStatus = 1;

			// ENCODER TIMER START
			HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
			fnEncCalibration();

			//SEND INFO THAT ENGINE IS READY
			HAL_UART_Transmit(&huart3, "redy", 4, 100);

		}
		else {
			fnLEDsErrorState();
			Error_Handler();
		}

		break;
	}
}

void fnReset(){
	switch(iMachineStatus){

	case 10: //SHUTDOWN
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x06;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 20;
		}
		break;

	case 20://SWITCH ON
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x07;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 25;
		}
		break;

	case 25://ENABLE OPERATION
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x0F;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 70;
		}
		break;

	case 70://MOTOR STATUS CHECK
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x40;
		TxData[1] = 0x41;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iMachineStatus = 80;
		}
		break;

	case 80://READ STATUS CHECK
		if (RxData[4] == 39) {
			iMachineStatus = 1;
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
			HAL_TIM_Base_Stop_IT(&htim14);
		}
		else {
			fnLEDsErrorState();
			Error_Handler();
		}

		break;
	}
}


//CALCULATING ENCODER'S COUNTS TO ANGLE
float fnEncCounts2Angle(uint16_t iCounts)
{
	fEncAngleTemp = iCounts*fEncDegPerCount;

	return fEncAngleTemp;
}

//READING DATA FROM ENCODER
void fnEncReadCount()
{
	iEncCountReal = __HAL_TIM_GET_COUNTER(&htim3);
	if(iEncCountReal > iEncCountsNumber / 2)
	{
		iEncCount = iEncCountsNumber - iEncCountReal;
	}
	else
	{
		iEncCount = iEncCountReal;
	}

	fEncAngle = fnEncCounts2Angle(iEncCount)/2;
}

//ENCODER CALIBRATION - BASE
void fnEncCalibration()
{
	TIM3->CNT = 0;
	fnEncReadCount();
}

//FRAME SET POSITION SEND
void fnMoveAbsolute(uint32_t iNumber){

	//VAR CHECKING IF LEG IS SELECTED
	uint8_t iMoveEnabled = 100;
	if (iNumber > 90) {
		iNumber = 90;
	}
	else if (iNumber < 0) {
		iNumber = 0;
	}

	//ESTABLISHING MOTION DIRECTORY BASED ON SEL LEG
	if(iSelected_leg == 0){
		iNumber = iNumber * 1000;
		iMoveEnabled = 1;
	}

	else if(iSelected_leg == 1){
		iNumber = -iNumber * 1000;
		iMoveEnabled = 1;
	}

	else
		iMoveEnabled = 0;

	if(iMoveEnabled == 1){
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x7A;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = (uint8_t) iNumber;
		TxData[5] = (uint8_t)(iNumber >> 8);
		TxData[6] = (uint8_t)(iNumber >> 16);
		TxData[7] = (uint8_t)(iNumber >> 24);

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}

	}
}

void fnSingleMotionAction(){
	//SEND INFO THAT ENGINE IS WORKING
	HAL_UART_Transmit(&huart3, "work", 4, 100);
	switch (iSingleMachineStatus){
	case 10:
		//START SUPPLY
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x1F;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iSingleMachineStatus = 20;
		}
		break;

	case 20:
		//MOVE
		fnMoveAbsolute(iPosition);
		iSingleMachineStatus = 25;
		break;

	case 25:
		//SET POINT ACK CHECK
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x40;
		TxData[1] = 0x41;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else if (RxData[5] & 0b10000) {
			iSingleMachineStatus = 30;
		}
		break;

	case 30:
		//STOP SUPPLY
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x0F;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iSingleMachineStatus = 35;
		}
		break;

	case 35:
		//TARGET REACHED CHECK
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x40;
		TxData[1] = 0x41;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else if (RxData[5] & 0b00100) {
			iSingleMachineStatus = 100;
			HAL_TIM_Base_Stop_IT(&htim7);
			//SEND INFO THAT ENGINE IS READY
			HAL_UART_Transmit(&huart3, "redy", 4, 100);
		}
		break;
	}
}

void fnSerialMotionAction(){
	//SEND INFO THAT ENGINE IS WORKING
	HAL_UART_Transmit(&huart3, "work", 4, 100);

	switch (iSerialMachineStatus){
	case 10:
		//START SUPPLY
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x1F;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iSerialMachineStatus = 20;
		}
		break;

	case 20:
		//MOVE
		fnMoveAbsolute(iPosition);
		iSerialMachineStatus = 25;
		break;

	case 25:
		//SET POINT ACK CHECK
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x40;
		TxData[1] = 0x41;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else if (RxData[5] & 0b10000) {
			iSerialMachineStatus = 30;
		}
		break;

	case 30:
		//STOP SUPPLY
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x0F;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else{
			iSerialMachineStatus = 35;
		}
		break;

	case 35:
		//TARGET REACHED CHECK
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x40;
		TxData[1] = 0x41;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			fnLEDsErrorState();
			Error_Handler();
		}
		else if (RxData[5] & 0b00100) {
			iSerialCounter++;
			iSerialMachineStatus = 40;
		}
		break;

	case 40:
		//REPETITION COUNTER CHECK
		if (iSerialCounter / 2 == iSerialReps) {
			iSerialMachineStatus = 100;
			iSerialRange = 0;
			iSerialReps = 0;
			HAL_TIM_Base_Stop(&htim10);
			//SEND INFO THAT ENGINE IS READY
			HAL_UART_Transmit(&huart3, "redy", 4, 100);
		}
		else if (iSerialCounter % 2 == 0) {
			iPosition = iSerialRange;
			iSerialMachineStatus = 10;
		}
		else {
			iPosition = 0;
			iSerialMachineStatus = 10;
		}
		break;
	}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim -> Instance == TIM6){
		if (iHomingStatus != 1) {
			fnInit();
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		}
		else {
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
			fnEncReadCount();
<<<<<<< HEAD
=======

			//TRANSMIT THE ENCODER VALUE
			length = sprintf(data_msg, "e%.3f", fEncAngle);
			//			HAL_UART_Transmit(&huart3, data_msg, length, 0xffff);
			//			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

>>>>>>> main
		}
	}
	else if(htim -> Instance == TIM7){
		fnSingleMotionAction();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	else if(htim -> Instance == TIM10){
		fnSerialMotionAction();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	else if(htim -> Instance == TIM11){
		//SENDING ACTUAL ENCODER VALUE
		length = sprintf(data_msg, "e%.3f \r\n", fEncAngle);
		HAL_UART_Transmit(&huart3, data_msg, length, 0xffff);
	}
	else if(htim -> Instance == TIM14){
		fnReset();
	}


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == USER_Btn_Pin){
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x40;
		TxData[1] = 0x41;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			Error_Handler();

		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_CAN1_Init();
	MX_TIM6_Init();
	MX_TIM3_Init();
	MX_TIM7_Init();
	MX_TIM10_Init();
	MX_TIM14_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_CAN1_Init();
	MX_TIM6_Init();
	MX_TIM3_Init();
	MX_TIM7_Init();
	MX_TIM10_Init();
	MX_TIM14_Init();
	/* USER CODE BEGIN 2 */

	// UART START
	HAL_UART_Receive_IT(&huart3, sUserMessage, 4);

	// CAN START
	HAL_CAN_Start(&hcan1);

	// CAN CONFIG
	// TxHeader param config
	TxHeader.StdId = 0x000;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;
	TxHeader.TransmitGlobalTime = DISABLE;

	// CANFilter param config
	CANFilter.FilterActivation = CAN_FILTER_ENABLE;
	CANFilter.FilterBank = 18;
	CANFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CANFilter.FilterIdHigh = 0x58A<<5;
	CANFilter.FilterIdLow = 0x0000;
	CANFilter.FilterMaskIdHigh = 0x58A<<5;
	CANFilter.FilterMaskIdLow = 0x0000;
	CANFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	CANFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	CANFilter.SlaveStartFilterBank = 20;

	HAL_CAN_ConfigFilter(&hcan1, &CANFilter);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){

		//INIT COMMAND
		if(strncmp(sUserMessage, sInitCommand,4) == 0){
			iMachineStatus = 0;
			iHomingStatus = 0;
			HAL_TIM_Base_Start_IT(&htim6);
		}

		//MODE SELECTION [SNGL/SERL]
		else if(strncmp(sUserMessage, sSingle_mode, 4) == 0){
			iMode = 0;
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		}

		else if(strncmp(sUserMessage, sSerial_mode, 4) == 0){
			iMode = 1;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}

		//LEG SELECTION [RGHT/LEFT]
		else if(strncmp(sUserMessage, sLeft, 4) == 0){
			iSelected_leg = 0;
			HAL_TIM_Base_Start_IT(&htim11);
		}

		else if(strncmp(sUserMessage, sRight, 4) == 0){
			iSelected_leg = 1;
			HAL_TIM_Base_Start_IT(&htim11);
		}

		//SINGLE POSITION [P]
		else if(sUserMessage[3] == 'P'){

			if(iMode == 0){
				iPosition = (uint32_t)(atoi(sUserMessage));
				iSingleMachineStatus = 10;
				HAL_TIM_Base_Start_IT(&htim7);
				//fnSingleMotionAction();
				//fnMoveAbsolute(iPosition);

			}
		}

		//START SERIAL [STSE]
		else if(strncmp(sUserMessage, sSerialStart, 4) == 0){

			if(iMode == 1 && iSelected_leg != 100 && iSerialReps != 0 && iSerialRange != 0){
				iSerialCounter = 0;
				iSerialMachineStatus = 10;
				HAL_TIM_Base_Start_IT(&htim10);
			}

		}


		//SERIAL - REPETITIONS [N]
		else if(sUserMessage[3] == 'N'){
			if(iMode == 1){
				iSerialReps = (uint32_t)(atoi(sUserMessage));
			}
		}

		//SERIAL - RANGE [R]
		else if(sUserMessage[3] == 'R'){
			if(iMode == 1){
				iSerialRange = (uint32_t)(atoi(sUserMessage));
				iPosition = iSerialRange;
			}
		}

		//GENERAL STOP COMMAND [HALT]
		else if(strncmp(sUserMessage, sGeneralStop, 4) == 0){

			//QUICK STOP
			TxHeader.StdId = 0x60A;
			TxHeader.DLC = 8;
			TxData[0] = 0x22;
			TxData[1] = 0x40;
			TxData[2] = 0x60;
			TxData[3] = 0x00;
			TxData[4] = 0x02;
			TxData[5] = 0x00;
			TxData[6] = 0x00;
			TxData[7] = 0x00;

			if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
				fnLEDsErrorState();
				Error_Handler();
			}
			else{
				//DISBALE VOLTAGE
				TxHeader.StdId = 0x60A;
				TxHeader.DLC = 8;
				TxData[0] = 0x22;
				TxData[1] = 0x40;
				TxData[2] = 0x60;
				TxData[3] = 0x00;
				TxData[4] = 0x00;
				TxData[5] = 0x00;
				TxData[6] = 0x00;
				TxData[7] = 0x00;

				if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
					fnLEDsErrorState();
					Error_Handler();
				}
				else {
					HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
					HAL_UART_Transmit(&huart3, "STOP", 4, 100);
				}
			}
		}

		//RESET COMMAND [RSET]
		else if(strncmp(sUserMessage, sReset, 4) == 0){
			iMachineStatus = 10;
			iHomingStatus = 100;
			iMode = 100;

			iSerialMachineStatus = 100;
			iSerialRange = 0;
			iSerialReps = 0;

			iSingleMachineStatus = 100;
			iPosition = 0;

			HAL_TIM_Base_Stop_IT(&htim7);
			HAL_TIM_Base_Stop_IT(&htim10);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		}

		//CONTINUE COMMAND [CONT]
		else if(strncmp(sUserMessage, sContinue, 4) == 0){
			iMachineStatus=10;

			if (iMode == 0 && iSingleMachineStatus != 100) {
				iSingleMachineStatus=10;
			}
			else if (iMode == 1 && iSerialMachineStatus != 100) {
				if (iSerialMachineStatus == 40) {
					iSerialCounter--;
					iSerialMachineStatus = 10;
				}
				else{
					iSerialMachineStatus = 10;
				}
			}

			HAL_TIM_Base_Start_IT(&htim14);
		}


	}
	else{
		HAL_UART_Transmit(&huart3, sErrorMessage, strlen(sErrorMessage), 100);
	}
	HAL_UART_Receive_IT(&huart3, sUserMessage, 4);
	/*

	case 70://POSTION CHECK
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x40;
		TxData[1] = 0x64;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x00;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			Error_Handler();
		}
		else{
			iMachineStatus = 80;
			HAL_UART_Transmit(&huart3, "C070", 4, 100);
		}
		break;
	case 200://FAULT RESET
		TxHeader.StdId = 0x60A;
		TxHeader.DLC = 8;
		TxData[0] = 0x22;
		TxData[1] = 0x40;
		TxData[2] = 0x60;
		TxData[3] = 0x00;
		TxData[4] = 0x80;
		TxData[5] = 0x00;
		TxData[6] = 0x00;
		TxData[7] = 0x00;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			Error_Handler();
		}
		else{
			iMachineStatus = 10;
			HAL_UART_Transmit(&huart3, "C200", 4, 100);
		}
		break;
	}
	 */
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
