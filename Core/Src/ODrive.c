#include "ODrive.h"
#include <string.h>

static void pack_float(uint8_t* data, float value) {
    memcpy(data, &value, sizeof(float));
}

static void pack_u32_le(uint8_t* data, uint32_t v)
{
    data[0] = (uint8_t)(v >> 0);
    data[1] = (uint8_t)(v >> 8);
    data[2] = (uint8_t)(v >> 16);
    data[3] = (uint8_t)(v >> 24);
}

// build can message id (node_id + cmd_id)
static uint32_t build_can_id(uint8_t axis_id, uint8_t cmd_id) {
    return ((uint32_t)axis_id << 5) | (uint32_t)cmd_id;
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern void sendCAN1(uint32_t id, uint8_t data[], int size);
extern void sendCAN2(uint32_t id, uint8_t data[], int size);

void Set_Axis_Requested_State(Axis axis, ODriveAxisState state) {
    // Axis_Requested_State is uint32, 4 bytes :contentReference[oaicite:11]{index=11}
    uint8_t payload[8] = {0};
    pack_u32_le(&payload[0], (uint32_t)state);

    uint32_t can_id = build_can_id(axis.AXIS_ID, SET_AXIS_STATE);
    if(axis.CAN_INSTANCE == &hcan1) sendCAN1(can_id, payload, 4);
    if(axis.CAN_INSTANCE == &hcan2) sendCAN2(can_id, payload, 4);
}

void Set_Input_Vel(Axis axis, float velocity, float torque_feedforward) {
    uint8_t payload[8] = {0};
    pack_float(&payload[0], velocity);
    pack_float(&payload[4], torque_feedforward);

    uint32_t can_id = build_can_id(axis.AXIS_ID, SET_INPUT_VEL);
    if(axis.CAN_INSTANCE == &hcan1) sendCAN1(can_id, payload, 8);
    if(axis.CAN_INSTANCE == &hcan2) sendCAN2(can_id, payload, 8);
}

void Set_Controller_Modes(Axis axis, ODriveControlMode control_mode, ODriveInputMode input_mode) {
    // Control_Mode (u32) + Input_Mode (u32) = 8 bytes :contentReference[oaicite:12]{index=12}
    uint8_t payload[8] = {0};
    pack_u32_le(&payload[0], (uint32_t)control_mode);
    pack_u32_le(&payload[4], (uint32_t)input_mode);

    uint32_t can_id = build_can_id(axis.AXIS_ID, SET_CONTROLLER_MODE);
    if(axis.CAN_INSTANCE == &hcan1) sendCAN1(can_id, payload, 8);
    if(axis.CAN_INSTANCE == &hcan2) sendCAN2(can_id, payload, 8);
}

// ---------------- RX decode ----------------

static bool dlc_at_least(const CAN_RxHeaderTypeDef* hdr, uint8_t n) {
    return (hdr->DLC >= n);
}

bool ODrive_ProcessRx(Axis* axis, const CAN_RxHeaderTypeDef* hdr, const uint8_t data[8])
{
    if (!axis || !hdr || !data) return false;

    if (hdr->IDE != CAN_ID_STD) return false;
    if (hdr->RTR != CAN_RTR_DATA) return false;

    const uint8_t node_id = (uint8_t)(hdr->StdId >> 5);
    if (node_id != axis->AXIS_ID) return false;

    const uint8_t cmd_id = (uint8_t)(hdr->StdId & 0x1F);

    switch (cmd_id) {

    case HEARTBEAT:
        // Axis_Error(u32), Axis_State(u8), Procedure_Result(u8), Trajectory_Done_Flag(u8) :contentReference[oaicite:13]{index=13}
        if (!dlc_at_least(hdr, 7)) return false;
        memcpy(&axis->feedback.axis_error, &data[0], 4);
        axis->feedback.axis_state       = data[4];
        axis->feedback.procedure_result = data[5];
        axis->feedback.trajectory_done  = data[6];
        return true;

    case GET_ERROR:
        // Active_Errors(u32), Disarm_Reason(u32) :contentReference[oaicite:14]{index=14}
        if (!dlc_at_least(hdr, 8)) return false;
        memcpy(&axis->feedback.active_errors, &data[0], 4);
        memcpy(&axis->feedback.disarm_reason, &data[4], 4);
        return true;

    case GET_ENCODER_ESTIMATES:
        // Pos_Estimate(float), Vel_Estimate(float) :contentReference[oaicite:15]{index=15}
        if (!dlc_at_least(hdr, 8)) return false;
        memcpy(&axis->feedback.pos_estimate, &data[0], 4);
        memcpy(&axis->feedback.vel_estimate, &data[4], 4);
        return true;

    case GET_IQ:
        // Iq_Setpoint(float), Iq_Measured(float) :contentReference[oaicite:16]{index=16}
        if (!dlc_at_least(hdr, 8)) return false;
        memcpy(&axis->feedback.iq_setpoint, &data[0], 4);
        memcpy(&axis->feedback.iq_measured, &data[4], 4);
        return true;

    case GET_TEMPERATURE:
        // FET_Temperature(float), Motor_Temperature(float) :contentReference[oaicite:17]{index=17}
        if (!dlc_at_least(hdr, 8)) return false;
        memcpy(&axis->feedback.fet_temperature, &data[0], 4);
        memcpy(&axis->feedback.motor_temperature, &data[4], 4);
        return true;

    case GET_BUS_VOLTAGE_CURRENT:
        // Bus_Voltage(float), Bus_Current(float) :contentReference[oaicite:18]{index=18}
        if (!dlc_at_least(hdr, 8)) return false;
        memcpy(&axis->feedback.bus_voltage, &data[0], 4);
        memcpy(&axis->feedback.bus_current, &data[4], 4);
        return true;

    case GET_TORQUES:
        // Torque_Target(float), Torque_Estimate(float) :contentReference[oaicite:19]{index=19}
        if (!dlc_at_least(hdr, 8)) return false;
        memcpy(&axis->feedback.torque_target, &data[0], 4);
        memcpy(&axis->feedback.torque_estimate, &data[4], 4);
        return true;

    default:
        return false;
    }
}

