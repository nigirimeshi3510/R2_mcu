#ifndef ODRIVE_H_
#define ODRIVE_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// ODrive CAN Command IDs (CANSimple)
typedef enum {
    GET_VERSION               = 0x00,
    HEARTBEAT                 = 0x01,
    GET_ERROR                 = 0x03,
    SET_AXIS_STATE            = 0x07,
    GET_ENCODER_ESTIMATES     = 0x09,
    SET_CONTROLLER_MODE       = 0x0B,
    SET_INPUT_VEL             = 0x0D,
    GET_IQ                    = 0x14,
    GET_TEMPERATURE           = 0x15,
    GET_BUS_VOLTAGE_CURRENT   = 0x17,
    CLEAR_ERRORS              = 0x18,
    GET_TORQUES               = 0x1C,
} ODriveCommandID;

// ODrive Axis States
typedef enum {
    ODRIVE_AXIS_STATE_IDLE                      = 1,
    ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL       = 8
} ODriveAxisState;

// ODrive Control Modes
typedef enum {
    VOLTAGE_CONTROL  = 0,
    TORQUE_CONTROL   = 1,
    VELOCITY_CONTROL = 2,
    POSITION_CONTROL = 3
} ODriveControlMode;

// ODrive Input Modes
typedef enum {
    INACTIVE      = 0,
    PASSTHROUGH   = 1,
    VEL_RAMP      = 2,
    POS_FILTER    = 3,
    MIX_CHANNELS  = 4,
    TRAP_TRAJ     = 5,
    TORQUE_RAMP   = 6,
    MIRROR        = 7
} ODriveInputMode;

// ODrive feedback (ODrive → Host cyclic messages)
typedef struct {
    // Heartbeat (0x01)
    uint32_t axis_error;
    uint8_t  axis_state;
    uint8_t  procedure_result;
    uint8_t  trajectory_done;

    // Get_Error (0x03)
    uint32_t active_errors;
    uint32_t disarm_reason;

    // Get_Encoder_Estimates (0x09)
    float pos_estimate;
    float vel_estimate;

    // Get_Iq (0x14)
    float iq_setpoint;
    float iq_measured;

    // Get_Temperature (0x15)
    float fet_temperature;
    float motor_temperature;

    // Get_Bus_Voltage_Current (0x17)
    float bus_voltage;
    float bus_current;

    // Get_Torques (0x1C)
    float torque_target;
    float torque_estimate;

} ODriveFeedback_t;

// ODrive axis
typedef struct {
    CAN_HandleTypeDef* CAN_INSTANCE;
    uint8_t            AXIS_ID;    // = node_id
    ODriveFeedback_t   feedback;   
} Axis;

// TX
void Set_Axis_Requested_State(Axis axis, ODriveAxisState state);
void Set_Input_Vel(Axis axis, float velocity, float torque_feedforward);
void Set_Controller_Modes(Axis axis, ODriveControlMode control_mode, ODriveInputMode input_mode);

// RX decode: 受信したCANフレームを Axis.feedback に反映する
bool ODrive_ProcessRx(Axis* axis, const CAN_RxHeaderTypeDef* hdr, const uint8_t data[8]);

#endif /* ODRIVE_H_ */

