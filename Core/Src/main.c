/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "bno055_stm32.h"
#include "Encoder.h"
#include "PS5.h"
#include "duty.h"
#include "ODrive.h"
#include "Robomaster.h"

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joy.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
rcl_publisher_t publisher_float;
std_msgs__msg__Float32 pub_float_msg;

rcl_publisher_t publisher_int;
std_msgs__msg__Int32 pub_int_msg;

rcl_publisher_t publisher_en;
std_msgs__msg__Int32 pub_en_msg;

rcl_publisher_t publisher_can;
std_msgs__msg__Int32 pub_can_msg;

rcl_publisher_t publisher_ext;
std_msgs__msg__Float32 pub_ext_msg;

rcl_publisher_t publisher_can_ext;
std_msgs__msg__String pub_can_ext_msg;

rcl_publisher_t publisher;
std_msgs__msg__String msg;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
PS5 ps5;
float can_1 = 0;
Axis axis1;
Axis axis2;
Axis axis3;
Axis axis4;
static uint8_t can1_ready = 0U;
static uint8_t can2_ready = 0U;
static volatile uint32_t can1_rx_overflow_count = 0U;
static volatile uint32_t can2_rx_error_count = 0U;
static volatile uint32_t can1_tx_drop_count = 0U;
static volatile uint32_t can1_tx_error_count = 0U;
static volatile uint32_t ps5_last_rx_ms = 0U;
static volatile uint8_t emergency_stop_active = 0U;

#define PS5_INPUT_TIMEOUT_MS 200U

#define ODRIVE_MONITOR_NODE_COUNT 4U
static const uint8_t odrive_monitor_node_order[ODRIVE_MONITOR_NODE_COUNT] = {1U, 2U, 3U, 4U};
static volatile uint32_t odrive_hb_rx_count[ODRIVE_MONITOR_NODE_COUNT] = {0U, 0U, 0U, 0U};
static volatile uint32_t odrive_hb_last_tick_ms[ODRIVE_MONITOR_NODE_COUNT] = {0U, 0U, 0U, 0U};
static volatile int32_t odrive_hb_last_gap_ms[ODRIVE_MONITOR_NODE_COUNT] = {-1, -1, -1, -1};
static volatile uint32_t odrive_hb_max_gap_ms[ODRIVE_MONITOR_NODE_COUNT] = {0U, 0U, 0U, 0U};
static volatile uint32_t odrive_hb_seq_ok_count = 0U;
static volatile uint32_t odrive_hb_seq_ng_count = 0U;
static volatile uint8_t odrive_hb_expected_order_idx = 0U;

double g_heading = 0.0;
double g_roll = 0.0;
double g_pitch = 0.0;

double g_quaternion_w = 0.0;
double g_quaternion_x = 0.0;
double g_quaternion_y = 0.0;
double g_quaternion_z = 0.0;

double g_accel_x = 0.0;
double g_accel_y = 0.0;
double g_accel_z = 0.0;

double g_gyro_x = 0.0;
double g_gyro_y = 0.0;
double g_gyro_z = 0.0;

double g_mag_x = 0.0;
double g_mag_y = 0.0;
double g_mag_z = 0.0;

double g_linear_accel_x = 0.0;
double g_linear_accel_y = 0.0;
double g_linear_accel_z = 0.0;

double g_gravity_x = 0.0;
double g_gravity_y = 0.0;
double g_gravity_z = 0.0;

uint8_t g_calib_system = 0;
uint8_t g_calib_gyro = 0;
uint8_t g_calib_accel = 0;
uint8_t g_calib_mag = 0;

int8_t g_temperature = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */
void sendCAN1(uint32_t id, uint8_t data[]);
void sendCAN2(uint32_t id, uint8_t data[]);
void C610_C620(uint32_t id, int16_t order_data[], int can_port);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef RxHeaderCAN1;
uint8_t RxDataCAN1[8];

CAN_RxHeaderTypeDef RxHeaderCAN2;
uint8_t RxDataCAN2[8];

uint32_t id;
uint32_t dlc;
uint8_t dataCAN1[8];
uint8_t dataCAN2[8];

void updateSensorData(void)
{
  bno055_vector_t euler = bno055_getVectorEuler();
  g_heading = euler.x;
  g_roll = euler.y;
  g_pitch = euler.z;

  bno055_vector_t quat = bno055_getVectorQuaternion();
  g_quaternion_w = quat.w;
  g_quaternion_x = quat.x;
  g_quaternion_y = quat.y;
  g_quaternion_z = quat.z;

  bno055_vector_t accel = bno055_getVectorAccelerometer();
  g_accel_x = accel.x;
  g_accel_y = accel.y;
  g_accel_z = accel.z;

  bno055_vector_t gyro = bno055_getVectorGyroscope();
  g_gyro_x = gyro.x;
  g_gyro_y = gyro.y;
  g_gyro_z = gyro.z;

  bno055_vector_t mag = bno055_getVectorMagnetometer();
  g_mag_x = mag.x;
  g_mag_y = mag.y;
  g_mag_z = mag.z;

  bno055_vector_t linear = bno055_getVectorLinearAccel();
  g_linear_accel_x = linear.x;
  g_linear_accel_y = linear.y;
  g_linear_accel_z = linear.z;

  bno055_vector_t gravity = bno055_getVectorGravity();
  g_gravity_x = gravity.x;
  g_gravity_y = gravity.y;
  g_gravity_z = gravity.z;

  bno055_calibration_state_t calib = bno055_getCalibrationState();
  g_calib_system = calib.sys;
  g_calib_gyro = calib.gyro;
  g_calib_accel = calib.accel;
  g_calib_mag = calib.mag;

  g_temperature = bno055_getTemp();
}

typedef struct
{
	float kp;
	float ki;
	float kd;
	float integral;
	float prev_error;
	float integral_limit;
	float output_limit;
} m3508_pid_t;

#define ODRIVE_CMD_SET_AXIS_STATE            0x07U
#define ODRIVE_CMD_SET_INPUT_VEL             0x0DU
#define ODRIVE_AXIS_STATE_CLOSED_LOOP_CTRL   8U
#define ODRIVE_DPAD_BASE_VEL_REV_S           3.0f

// D-pad mapping table (node1=FR, node2=FL, node3=BR, node4=BL).
// Adjust signs/values here to tune wheel directions for your machine.
#define ODRIVE_DPAD_UP_NODE1    ( ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_UP_NODE2    ( ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_UP_NODE3    ( ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_UP_NODE4    ( ODRIVE_DPAD_BASE_VEL_REV_S)

#define ODRIVE_DPAD_DOWN_NODE1  (-ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_DOWN_NODE2  (-ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_DOWN_NODE3  (-ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_DOWN_NODE4  (-ODRIVE_DPAD_BASE_VEL_REV_S)

#define ODRIVE_DPAD_RIGHT_NODE1 ( ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_RIGHT_NODE2 (-ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_RIGHT_NODE3 (-ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_RIGHT_NODE4 ( ODRIVE_DPAD_BASE_VEL_REV_S)

#define ODRIVE_DPAD_LEFT_NODE1  (-ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_LEFT_NODE2  ( ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_LEFT_NODE3  ( ODRIVE_DPAD_BASE_VEL_REV_S)
#define ODRIVE_DPAD_LEFT_NODE4  (-ODRIVE_DPAD_BASE_VEL_REV_S)

int16_t m3508_current_cmd[4] = {0, 0, 0, 0};
m3508_pid_t m3508_speed_pid[4];
int16_t m3508_target_rpm[4] = {0, 0, 0, 0};
moto_measure_t moto1;
moto_measure_t m3508_moto[4];

static float clampf(float value, float limit)
{
	if (value > limit) return limit;
	if (value < -limit) return -limit;
	return value;
}

static uint32_t odrive_make_can_id(uint8_t axis_id, uint8_t cmd_id)
{
	return (((uint32_t)axis_id << 5U) | ((uint32_t)cmd_id & 0x1FU));
}

static void odrive_send_frame(Axis *axis, uint8_t cmd_id, const uint8_t payload[8])
{
	uint32_t can_id = odrive_make_can_id(axis->AXIS_ID, cmd_id);

	if (axis->CAN_INSTANCE == &hcan1) {
		sendCAN1(can_id, (uint8_t *)payload);
	} else if (axis->CAN_INSTANCE == &hcan2) {
		sendCAN2(can_id, (uint8_t *)payload);
	}
}

static void odrive_set_axis_state(Axis *axis, uint32_t axis_state)
{
	uint8_t payload[8] = {0};
	memcpy(&payload[0], &axis_state, sizeof(axis_state)); // little-endian uint32
	odrive_send_frame(axis, ODRIVE_CMD_SET_AXIS_STATE, payload);
}

static void odrive_set_input_vel(Axis *axis, float vel_rev_s, float torque_ff_nm)
{
	uint8_t payload[8] = {0};
	memcpy(&payload[0], &vel_rev_s, sizeof(vel_rev_s));         // little-endian float32
	memcpy(&payload[4], &torque_ff_nm, sizeof(torque_ff_nm));   // little-endian float32
	odrive_send_frame(axis, ODRIVE_CMD_SET_INPUT_VEL, payload);
}

static void emergency_stop_all_motors(void)
{
	// ODrive: command zero velocity to all axes.
	odrive_set_input_vel(&axis4, 0.0f, 0.0f);
	odrive_set_input_vel(&axis3, 0.0f, 0.0f);
	odrive_set_input_vel(&axis2, 0.0f, 0.0f);
	odrive_set_input_vel(&axis1, 0.0f, 0.0f);

	// RoboMaster: zero all targets/currents and send zero current frame.
	for (int i = 0; i < 4; i++) {
		m3508_target_rpm[i] = 0;
		m3508_current_cmd[i] = 0;
	}
	C610_C620(0x200, m3508_current_cmd, 2);
}

static void odrive_enter_closed_loop_all(void)
{
	odrive_set_axis_state(&axis4, ODRIVE_AXIS_STATE_CLOSED_LOOP_CTRL);
	osDelay(2);
	odrive_set_axis_state(&axis3, ODRIVE_AXIS_STATE_CLOSED_LOOP_CTRL);
	osDelay(2);
	odrive_set_axis_state(&axis2, ODRIVE_AXIS_STATE_CLOSED_LOOP_CTRL);
	osDelay(2);
	odrive_set_axis_state(&axis1, ODRIVE_AXIS_STATE_CLOSED_LOOP_CTRL);
}

static void odrive_dpad_control_from_ps5(void)
{
	// Deadman: stop if Joy is stale.
	if ((HAL_GetTick() - ps5_last_rx_ms) > PS5_INPUT_TIMEOUT_MS) {
		odrive_set_input_vel(&axis4, 0.0f, 0.0f);
		odrive_set_input_vel(&axis3, 0.0f, 0.0f);
		odrive_set_input_vel(&axis2, 0.0f, 0.0f);
		odrive_set_input_vel(&axis1, 0.0f, 0.0f);
		return;
	}

	float node1_vel = 0.0f;
	float node2_vel = 0.0f;
	float node3_vel = 0.0f;
	float node4_vel = 0.0f;
	float v = 0.5f;
	if(ps5.Joy_click_R)
		v+= 0.5f;
	// D-pad commands are additive, so diagonal is also possible.
	if (ps5.up_btn == 1) {
		node1_vel = -v;
		node2_vel = v;
		node3_vel = -v;
		node4_vel = v;
	}
	if (ps5.down_btn == 1) {
		node1_vel = v;
		node2_vel = -v;
		node3_vel = v;
		node4_vel = -v;
	}
	if (ps5.right_btn == 1) {
		node1_vel = v;
		node2_vel = v;
		node3_vel = -v;
		node4_vel = -v;
	}
	if (ps5.left_btn == 1) {
		node1_vel = -v;
		node2_vel = -v;
		node3_vel = v;
		node4_vel = v;
	}
	if (ps5.L1_btn == 1) {
		node1_vel = -v;
		node2_vel = -v;
		node3_vel = -v;
		node4_vel = -v;
	}
	if (ps5.R1_btn == 1) {
		node1_vel = v;
		node2_vel = v;
		node3_vel = v;
		node4_vel = v;
	}

	// Keep CAN send order as 4 -> 3 -> 2 -> 1.
	odrive_set_input_vel(&axis4, node4_vel, 0.0f);
	odrive_set_input_vel(&axis3, node3_vel, 0.0f);
	odrive_set_input_vel(&axis2, node2_vel, 0.0f);
	odrive_set_input_vel(&axis1, node1_vel, 0.0f);
}

static int8_t odrive_monitor_order_index(uint8_t node_id)
{
	for (uint8_t i = 0U; i < ODRIVE_MONITOR_NODE_COUNT; i++) {
		if (odrive_monitor_node_order[i] == node_id) {
			return (int8_t)i;
		}
	}
	return -1;
}

static void odrive_monitor_on_can1_rx(const CAN_RxHeaderTypeDef *rxHdr)
{
	if (rxHdr == NULL) {
		return;
	}

	if ((rxHdr->IDE != CAN_ID_STD) || (rxHdr->RTR != CAN_RTR_DATA)) {
		return;
	}

	const uint8_t cmd_id = (uint8_t)(rxHdr->StdId & 0x1FU);
	if (cmd_id != HEARTBEAT) {
		return;
	}

	const uint8_t node_id = (uint8_t)(rxHdr->StdId >> 5);
	const int8_t idx = odrive_monitor_order_index(node_id);
	if (idx < 0) {
		return;
	}

	const uint32_t now_ms = HAL_GetTick();
	const uint32_t prev_ms = odrive_hb_last_tick_ms[(uint8_t)idx];

	if (prev_ms != 0U) {
		const uint32_t gap_ms = now_ms - prev_ms;
		odrive_hb_last_gap_ms[(uint8_t)idx] = (int32_t)gap_ms;
		if (gap_ms > odrive_hb_max_gap_ms[(uint8_t)idx]) {
			odrive_hb_max_gap_ms[(uint8_t)idx] = gap_ms;
		}
	}

	odrive_hb_last_tick_ms[(uint8_t)idx] = now_ms;
	odrive_hb_rx_count[(uint8_t)idx]++;

	if ((uint8_t)idx == odrive_hb_expected_order_idx) {
		odrive_hb_seq_ok_count++;
		odrive_hb_expected_order_idx = (uint8_t)((odrive_hb_expected_order_idx + 1U) % ODRIVE_MONITOR_NODE_COUNT);
	} else {
		odrive_hb_seq_ng_count++;
		odrive_hb_expected_order_idx = (uint8_t)(((uint8_t)idx + 1U) % ODRIVE_MONITOR_NODE_COUNT);
	}
}

void m3508_pid_init(m3508_pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
	pid->integral_limit = integral_limit;
	pid->output_limit = output_limit;
}

int16_t m3508_pid_calc(m3508_pid_t *pid, int16_t target_rpm, int16_t feedback_rpm)
{
	float error = (float)target_rpm - (float)feedback_rpm;
	pid->integral += error;
	pid->integral = clampf(pid->integral, pid->integral_limit);

	float derivative = error - pid->prev_error;
	pid->prev_error = error;

	float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
	output = clampf(output, pid->output_limit);

	return (int16_t)output;
}

void m3508_pid_init_all(void)
{
		// Start with conservative gains, then tune on hardware
		m3508_pid_init(&m3508_speed_pid[0], 11.9f, 0.02f, 0.0f, 8000.0f, 10000.0f);
		m3508_pid_init(&m3508_speed_pid[1], 8.9f, 0.12f, 0.0f, 8000.0f, 10000.0f);
		m3508_pid_init(&m3508_speed_pid[2], 22.5f, 0.02f, 0.0f, 8000.0f, 10000.0f);
		m3508_pid_init(&m3508_speed_pid[3], 14.0f, 0.07f, 0.0f, 8000.0f, 10000.0f);

}

void m3508_speed_pid_control_all(void)
{
	for (int i = 0; i < 4; i++) {
		int16_t target_rpm = m3508_target_rpm[i];
		if ((i == 0) || (i == 2)) {
			target_rpm = (int16_t)(-target_rpm);
		}

		if (m3508_moto[i].msg_cnt > 50) {
			m3508_current_cmd[i] = m3508_pid_calc(&m3508_speed_pid[i], target_rpm, m3508_moto[i].speed_rpm);
		} else {
			m3508_current_cmd[i] = 0;
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    if(hcan->Instance == CAN1){
        CAN_RxHeaderTypeDef rxHdr;
        uint8_t rxData[8];
        uint8_t handled = 0U;
        const uint8_t max_frames_per_irq = 8U;

        // ISR占有を防ぐため、1回のIRQで処理するフレーム数を制限する
        while ((HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) &&
               (handled < max_frames_per_irq)) {

            if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHdr, rxData) != HAL_OK) {
                break;
            }
            handled++;

            if ((rxHdr.IDE == CAN_ID_STD) && (((uint8_t)(rxHdr.StdId >> 5)) == 4U)) {

            }

            odrive_monitor_on_can1_rx(&rxHdr);

            // Axisが複数あるなら順に投げる
            (void)ODrive_ProcessRx(&axis4, &rxHdr, rxData);
            (void)ODrive_ProcessRx(&axis3, &rxHdr, rxData);
            (void)ODrive_ProcessRx(&axis2, &rxHdr, rxData);
            (void)ODrive_ProcessRx(&axis1, &rxHdr, rxData);
        }

        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
            can1_rx_overflow_count++;
        }
    } else if(hcan->Instance == CAN2){
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeaderCAN2, RxDataCAN2) == HAL_OK){

            //CAN2の受信データ格納 or 処理
            if (RxHeaderCAN2.IDE == CAN_ID_STD &&
                RxHeaderCAN2.StdId >= 0x201 &&
                RxHeaderCAN2.StdId <= 0x204) {
            	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                uint8_t idx = (uint8_t)(RxHeaderCAN2.StdId - 0x201);
                m3508_moto[idx].msg_cnt++ <= 50 ?
                    get_moto_offset(&m3508_moto[idx], RxDataCAN2) :
                    encoder_data_handler(&m3508_moto[idx], RxDataCAN2);
            }

            id = (RxHeaderCAN2.IDE == CAN_ID_STD)? RxHeaderCAN2.StdId : RxHeaderCAN2.ExtId;     // ID
            dlc = RxHeaderCAN2.DLC;                                                               // DLC
            for(int i=0;i<RxHeaderCAN2.DLC;i++){                                                  // Data
                dataCAN2[i] = RxDataCAN2[i];
           }
        }
        else {
            can2_rx_error_count++;
        }
    }
}

void subscription_joy_callback(const void * msgin)
{
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //LED turned on


  sensor_msgs__msg__Joy * msg = (sensor_msgs__msg__Joy *)msgin;

  gets_ps5(msg, &ps5);
  ps5_last_rx_ms = HAL_GetTick();

  // Remote emergency stop: PS + L1 latches stop state.
//  if ((ps5.PS_btn == 1) && (ps5.L1_btn == 1)) {
//	  emergency_stop_active = 1U;
//  }
}
void sendCAN1(uint32_t id, uint8_t data[]){
	if (can1_ready == 0U) {
		return;
	}

	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	uint32_t TxMailbox;

	const uint32_t tx_wait_start_ms = HAL_GetTick();
	const uint32_t tx_wait_timeout_ms = 2U;
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
		if ((HAL_GetTick() - tx_wait_start_ms) >= tx_wait_timeout_ms) {
			can1_tx_drop_count++;
			return;
		}
	}

	TxHeader.StdId = id;                    // CAN ID
	TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
	TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
	TxHeader.DLC = 8;                    // データ長(バイト)
	TxHeader.TransmitGlobalTime = DISABLE;  // ???

	for(int i=0;i<TxHeader.DLC;i++){
		TxData[i] = data[i];
	}
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	} else {
		can1_tx_error_count++;
	}
}

void sendCAN2(uint32_t id, uint8_t data[]){
	if (can2_ready == 0U) {
		return;
	}

	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	uint32_t TxMailbox;

	//while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3) Error_Handler();
	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)){
		TxHeader.StdId = id;                    // CAN ID
		TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
		TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
		TxHeader.DLC = 8;                    // データ長(バイト)
		TxHeader.TransmitGlobalTime = DISABLE;  // ???

		for(int i=0;i<TxHeader.DLC;i++){
			TxData[i] = data[i];
		}
		if(HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) == HAL_OK){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		}
	}
}

//id 0x200 or 0x1FF
void C610_C620(uint32_t id, int16_t order_data[], int can_port)
{
	// C610/C620: 4 motors x 16-bit current command in one CAN frame (0x200/0x1FF)
	uint8_t data[8];

	data[0] = (uint8_t)(order_data[0] >> 8);
	data[1] = (uint8_t)(order_data[0]);
	data[2] = (uint8_t)(order_data[1] >> 8);
	data[3] = (uint8_t)(order_data[1]);
	data[4] = (uint8_t)(order_data[2] >> 8);
	data[5] = (uint8_t)(order_data[2]);
	data[6] = (uint8_t)(order_data[3] >> 8);
	data[7] = (uint8_t)(order_data[3]);

	if(can_port == 1)
	{
		sendCAN1(id,data);
	}
	if(can_port == 2){
		sendCAN2(id,data);
	}
}
void robomaster_test(void){
	int16_t target_rpm = 0;

	if ((HAL_GetTick() - ps5_last_rx_ms) > PS5_INPUT_TIMEOUT_MS) {
		for (int i = 0; i < 4; i++) {
			m3508_target_rpm[i] = 0;
		}
		return;
	}

	if(ps5.triangle_btn == 1){
		target_rpm = 1000;
	} else if (ps5.cross_btn == 1) {
		target_rpm = -1000;
	}

	// M3508 x4: same speed target [rpm]
	for (int i = 0; i < 4; i++) {
		m3508_target_rpm[i] = target_rpm;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  // CANスタート（未接続でも起動を継続する）
  if (HAL_CAN_Start(&hcan1) == HAL_OK) {
    can1_ready = 1U;
  } else {
    can1_ready = 0U;
    printf("WARN: CAN1 start failed (err=0x%08lX)\r\n", HAL_CAN_GetError(&hcan1));
  }
  if (HAL_CAN_Start(&hcan2) == HAL_OK) {
    can2_ready = 1U;
  } else {
    can2_ready = 0U;
    printf("WARN: CAN2 start failed (err=0x%08lX)\r\n", HAL_CAN_GetError(&hcan2));
  }

  // 割り込み有効（開始できたCANのみ）
  if (can1_ready == 1U) {
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      can1_ready = 0U;
      printf("WARN: CAN1 notification failed (err=0x%08lX)\r\n", HAL_CAN_GetError(&hcan1));
    }
  }
  if (can2_ready == 1U) {
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      can2_ready = 0U;
      printf("WARN: CAN2 notification failed (err=0x%08lX)\r\n", HAL_CAN_GetError(&hcan2));
    }
  }
  axis1.CAN_INSTANCE = &hcan1;
  axis1.AXIS_ID = 1;

  axis2.CAN_INSTANCE = &hcan1;
  axis2.AXIS_ID = 2;

  axis3.CAN_INSTANCE = &hcan1;
  axis3.AXIS_ID = 3;

  axis4.CAN_INSTANCE = &hcan1;
  axis4.AXIS_ID = 4;
  m3508_pid_init_all();

  bno055_assignI2C(&hi2c2);
  bno055_setup();
  bno055_setOperationModeNDOF();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.FilterIdHigh         = 0;                        // フィルターID(上位16ビット)
  sFilterConfig.FilterIdLow          = 0;                        // フィルターID(下位16ビット)
  sFilterConfig.FilterMaskIdHigh     = 0;                        // フィルターマスク(上位16ビット)
  sFilterConfig.FilterMaskIdLow      = 0;                        // フィルターマスク(下位16ビット)
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;    // フィルタースケール
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // フィルターに割り当てるFIFO
  sFilterConfig.FilterBank           = 0;                        // フィルターバンクNo
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;    // フィルターモード
  sFilterConfig.SlaveStartFilterBank = 14;                       // スレーブCANの開始フィルターバンクNo
  sFilterConfig.FilterActivation     = ENABLE;                   // フィルター無効／有効



  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.FilterIdHigh         = 0;                        // フィルターID(上位16ビット)
  sFilterConfig.FilterIdLow          = 0;                        // フィルターID(下位16ビット)
  sFilterConfig.FilterMaskIdHigh     = 0;                        // フィルターマスク(上位16ビット)
  sFilterConfig.FilterMaskIdLow      = 0;                        // フィルターマスク(下位16ビット)
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;    // フィルタースケール
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // フィルターに割り当てるFIFO
  sFilterConfig.FilterBank           = 14;                        // フィルターバンクNo
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;    // フィルターモード
  sFilterConfig.SlaveStartFilterBank = 14;                       // スレーブCANの開始フィルターバンクNo
  sFilterConfig.FilterActivation     = ENABLE;                   // フィルター無効／有効

  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);


void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	// micro-ROS configuration
	rmw_uros_set_custom_transport(
		true,
		(void *) &huart2,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

		rcl_subscription_t subscriber_joy;
		sensor_msgs__msg__Joy sub_joy_msg;
		rcl_publisher_t m3508_telemetry_pub;
		rcl_publisher_t imu_pub;
		std_msgs__msg__Int32MultiArray m3508_telemetry_msg;
		sensor_msgs__msg__Imu imu_msg;
		int32_t m3508_telemetry_data[20];
		rclc_support_t support;
		rcl_allocator_t allocator;
		rcl_node_t node;

	allocator = rcl_get_default_allocator();

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	size_t domain_id = (size_t)111;
	rcl_init_options_set_domain_id(&init_options, domain_id);

	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
		RCCHECK(rclc_node_init_default(&node, "f446re_node", "", &support));
		RCCHECK(rclc_publisher_init_default(
			&m3508_telemetry_pub,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
			"/m3508/telemetry"));
		RCCHECK(rclc_publisher_init_default(
			&imu_pub,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
			"/imu/data"));

		RCCHECK(rclc_subscription_init_default(
			&subscriber_joy,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
		"/joy"));

	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_joy, &sub_joy_msg, &subscription_joy_callback, ON_NEW_DATA));

	memset(&sub_joy_msg, 0, sizeof(sub_joy_msg));
	sub_joy_msg.buttons.capacity = 100;
	sub_joy_msg.buttons.data = (int32_t *)malloc(100 * sizeof(int32_t));
	sub_joy_msg.buttons.size = 0;

	sub_joy_msg.axes.capacity = 100;
	sub_joy_msg.axes.data = (float *)malloc(100 * sizeof(float));
	sub_joy_msg.axes.size = 0;

		sub_joy_msg.header.frame_id.capacity = 100;
		sub_joy_msg.header.frame_id.data = (char *)malloc(100 * sizeof(char));
		sub_joy_msg.header.frame_id.size = 0;

		memset(&m3508_telemetry_msg, 0, sizeof(m3508_telemetry_msg));
		m3508_telemetry_msg.data.capacity = 20;
		m3508_telemetry_msg.data.size = 20;
		m3508_telemetry_msg.data.data = m3508_telemetry_data;

		memset(&imu_msg, 0, sizeof(imu_msg));
		if (!sensor_msgs__msg__Imu__init(&imu_msg)) {
			printf("Failed to init IMU message\r\n");
			return;
		}
		if (!rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link")) {
			printf("Failed to assign IMU frame_id\r\n");
			return;
		}
		imu_msg.orientation_covariance[0] = -1.0;
		imu_msg.angular_velocity_covariance[0] = -1.0;
		imu_msg.linear_acceleration_covariance[0] = -1.0;

		uint32_t last_pub_ms = HAL_GetTick();
		uint32_t last_imu_pub_ms = HAL_GetTick();
		while (1) {
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

			uint32_t now_ms = HAL_GetTick();
			if ((now_ms - last_pub_ms) >= 20U) {
				last_pub_ms = now_ms;
				for (int i = 0; i < 4; i++) {
					int base = i * 5;
					m3508_telemetry_data[base + 0] = (int32_t)m3508_target_rpm[i];
					m3508_telemetry_data[base + 1] = (int32_t)m3508_moto[i].speed_rpm;
					m3508_telemetry_data[base + 2] = (int32_t)m3508_current_cmd[i];
					m3508_telemetry_data[base + 3] = (int32_t)m3508_moto[i].given_current;
					m3508_telemetry_data[base + 4] = (int32_t)m3508_moto[i].msg_cnt;
				}
				RCSOFTCHECK(rcl_publish(&m3508_telemetry_pub, &m3508_telemetry_msg, NULL));
			}

			if ((now_ms - last_imu_pub_ms) >= 100U) {
				last_imu_pub_ms = now_ms;
				updateSensorData();

				imu_msg.orientation.w = g_quaternion_w;
				imu_msg.orientation.x = g_quaternion_x;
				imu_msg.orientation.y = g_quaternion_y;
				imu_msg.orientation.z = g_quaternion_z;

				imu_msg.angular_velocity.x = g_gyro_x;
				imu_msg.angular_velocity.y = g_gyro_y;
				imu_msg.angular_velocity.z = g_gyro_z;

				imu_msg.linear_acceleration.x = g_linear_accel_x;
				imu_msg.linear_acceleration.y = g_linear_accel_y;
				imu_msg.linear_acceleration.z = g_linear_accel_z;

				RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
			}
		}

	sensor_msgs__msg__Imu__fini(&imu_msg);
	RCCHECK(rcl_publisher_fini(&imu_pub, &node));
	RCCHECK(rcl_publisher_fini(&m3508_telemetry_pub, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_joy, &node));
	RCCHECK(rcl_node_fini(&node));
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  uint8_t odrive_closed_loop_requested = 0U;
  uint32_t odrive_tx_last_ms = HAL_GetTick();
  const uint32_t odrive_tx_period_ms = 10U;
  for(;;)
  {
			if (odrive_closed_loop_requested == 0U) {
				odrive_enter_closed_loop_all();
				odrive_closed_loop_requested = 1U;
			}
			uint32_t now_ms = HAL_GetTick();
			if (emergency_stop_active == 1U) {
				if ((now_ms - odrive_tx_last_ms) >= odrive_tx_period_ms) {
					odrive_tx_last_ms = now_ms;
					emergency_stop_all_motors();
				}
				osDelay(1);
				continue;
			}
			if ((now_ms - odrive_tx_last_ms) >= odrive_tx_period_ms) {
				odrive_tx_last_ms = now_ms;
				odrive_dpad_control_from_ps5();
			}

		robomaster_test();
		m3508_speed_pid_control_all();
		C610_C620(0x200, m3508_current_cmd, 2);
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
