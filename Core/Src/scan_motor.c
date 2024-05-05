/*
 * scan_motor.c
 *
 *  Created on: Mar 17, 2024
 *      Author: Hwabok
 */


#include "scan_motor.h"
#include "main.h"
#include "stdlib.h"
#include "cmsis_os.h"

#define bookshelf_height 10 //모터 회전수 기준 높이
#define bookshelf_floor 5

int limit_switch_up = 0;
int limit_switch_down = 0;

extern int motor_status;
extern osThreadId motorTaskHandle;
extern osThreadId rfidExecuteTaskHandle;

void scan_motor_up(){
	if(limit_switch_up){
		TIM1->CCR1 = 0;
		motor_status = 1;
		limit_switch_up = 0;
		vTaskSuspend(rfidExecuteTaskHandle);
		vTaskSuspend(motorTaskHandle);
	}else{
		change_motor_direction(1);
		TIM1->CCR1 = 400;
	}
}

void scan_motor_down(){
	if(limit_switch_down){
		TIM1->CCR1 = 0;
		motor_status = 0;
		limit_switch_down = 0;
		vTaskSuspend(rfidExecuteTaskHandle);
		vTaskSuspend(motorTaskHandle);
	}else{
		change_motor_direction(0);
		TIM1->CCR1 = 400;
	}
}

/* 1:반시계 0:시계*/
void change_motor_direction(int direction){
	if(direction == 1){
		HAL_GPIO_WritePin(Direction0_GPIO_Port, Direction0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Direction1_GPIO_Port, Direction1_Pin, GPIO_PIN_RESET);
	}else if(direction == 0){
		HAL_GPIO_WritePin(Direction0_GPIO_Port, Direction0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Direction1_GPIO_Port, Direction1_Pin, GPIO_PIN_SET);
	}
}
