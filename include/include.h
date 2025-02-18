#ifndef INCLUDE_H
#define INCLUDE_H

extern "C" {
    #include "soem/ethercat.h"
}
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>

///////// Object Dictionary for EPOS4 //////////

// RxPDO Mapping (Output from master to slave)
#define OD_RXPDO_ASSIGN 0x1C12
#define OD_RXPDO_MAP_1 0x1600

#define OD_CONTROL_WORD 0X6040
#define OD_CONTROL_WORD_MAP 0x60400010
#define OD_TARGET_POSITION 0X607A
#define OD_TARGET_POSITION_MAP 0X607A0020
#define OD_PROFILE_VELOCITY 0X6081
#define OD_PROFILE_VELOCITY_MAP 0X60810020
#define OD_DIGITAL_OUTPUTS 0X60FE
#define OD_DIGITAL_OUTPUTS_MAP 0X60FE0120
#define OD_MODE_OF_OPERATION 0X6060
#define OD_MODE_OF_OPERATION_MAP 0X60600008

// TxPDO Mapping (Input from slave to master)
#define OD_TXPDO_ASSIGN 0x1C13
#define OD_TXPDO_MAP_1 0X1A00

#define OD_STATUS_WORD 0X6041
#define OD_STATUS_WORD_MAP 0X60410010
#define OD_POSITION_ACTUAL 0X6064
#define OD_POSITION_ACTUAL_MAP 0X60640020
#define OD_VELOCITY_ACTUAL 0X606C
#define OD_VELOCITY_ACTUAL_MAP 0X606C0020
#define OD_DIGITAL_INPUTS 0X60FD
#define OD_DIGITAL_INPUTS_MAP 0X60FD0020
#define OD_ERROR_CODE 0X603F
#define OD_ERROR_CODE_MAP 0X603F0010
#define OD_MODE_OF_OPERATION_DISPLAY 0X6061
#define OD_MODE_OF_OPERATION_DISPLAY_MAP 0X60610008


// Not mapped
#define OD_POSITION_DEMAND 0X6062
#define OD_VELOCITY_DEMAND 0X606B
#define OD_TARGET_TORQUE 0X6071
#define OD_TORQUE_ACTUAL 0X6077
#define OD_MAX_PROFILE_VELOCITY 0X607F
#define OD_PROFILE_ACCELERATION 0X6083
#define OD_PROFILE_DECELERATION 0X6084
#define OD_HOME_POSITION 0X30B0
#define OD_HOME_OFFSET_MOVE_DISTANCE 0X30B1
#define OD_HOMING_METHOD 0X6098
#define OD_HOMING_SPEED 0X6099
#define OD_HOMING_ACCELERATION 0X609A
#define OD_SOFTWARE_LIMIT 0X607D
#define OD_MANUFACTURER 0X1008
#define OD_IDENTIFY_OBJECT 0X1018

// POLARITY: index = OD_AXIS, subindex = 0x04, bit 0 (CCW=0x00000000, CW=0x00000001)
#define OD_AXIS 0X3000

// Control Word (Homing or PPM mode)
#define CONTROL_WORD_HM_START 0X001F
#define CONTROL_WORD_PP_CHANGE 0X003F
#define CONTROL_WORD_PP_CHANGE_REL 0X007F
#define CONTROL_WORD_PP_NEW 0X002F
#define CONTROL_WORD_CSP 0X000F

// Mode of Operation
#define MODE_HM 0X06
#define MODE_PP 0X01
#define MODE_CSP 0X08

// // Status words
// #define STATUS_NOT_READY_SWITCH 0X0
// #define STATUS_SWITCH_ON_DISABLED (1 << 6)
// #define STATUS_READY_SWITCH_ON ((1 << 5) | 1)
// #define STATUS_SWITCH_ON ((1 << 5) | 3)
// #define STATUS_OPERATION_ENABLED ((1 << 5) | (1 << 4) | 7)
// #define STATUS_QUICK_STOP_A 0X7
// #define STATUS_FAULT_REACT 0XF
// #define STATUS_FAULT 0X8

// Status words
#define READY_SWITCH_ON_BIT 0
#define SWITCHED_ON_BIT 1
#define OPERATION_ENABLED_BIT 2
#define QUICK_STOP_A_BIT 0X7
#define FAULT_REACT_BIT 0XF
#define FAULT_BIT 0X8

// Status word home bits
#define STATUS_HOME_ERROR (1 << 13)
#define HOME_ATTAINED (1 << 12)
#define HOME_REACHED (1 << 10)

#define SENSOR_SUPERVISE_BIT (1 << 8)

// Set/Clear Bit
#define BIT_VALUE(bit) (1 << bit)
#define IS_BIT_SET(val, bit) (val & BIT_VALUE(bit))
#define IS_BIT_CLEAR(val, bit) !(val & BIT_VALUE(bit))
#define SET_BIT(val, bit) (val | BIT_VALUE(bit))
#define CLEAR_BIT(val, bit) (val & ^BIT_VALUE(bit))

#define CYCLE_TIME_MS 5
#define CYCLE_TIME_US 5000
#define NUM_MOTORS 2

//////////// SDO read/write wrapper functions ///////////////

int read_sdo_u8(uint16 slave, uint16 index, uint8 subindex, uint8 *data);
int read_sdo_u16(uint16 slave, uint16 index, uint8 subindex, uint16 *data);
int read_sdo_u32(uint16 slave, uint16 index, uint8 subindex, uint32 *data);
int read_sdo_s8(uint16 slave, uint16 index, uint8 subindex, int8 *data);
int read_sdo_s16(uint16 slave, uint16 index, uint8 subindex, int16 *data);
int read_sdo_s32(uint16 slave, uint16 index, uint8 subindex, int32 *data);

int write_sdo_u8(uint16 slave, uint16 index, uint8 subindex, uint8 data);
int write_sdo_u16(uint16 slave, uint16 index, uint8 subindex, uint16 data);
int write_sdo_u32(uint16 slave, uint16 index, uint8 subindex, uint32 data);
int write_sdo_s8(uint16 slave, uint16 index, uint8 subindex, int8 data);
int write_sdo_s16(uint16 slave, uint16 index, uint8 subindex, int16 data);
int write_sdo_s32(uint16 slave, uint16 index, uint8 subindex, int32 data);

bool stateOperationEnabled(const uint16& status);
bool stateSwitchedOn(const uint16& status);
bool stateReadyswitchOn(const uint16& status);
bool stateSwitchOnDisabled(const uint16& status);
bool stateNotReadySwitchOn(const uint16& status);
bool stateFault(const uint16& status);
bool stateFaultReact(const uint16& status);
bool stateQuickStop(const uint16& status);

#include <cstdio>

typedef struct PACKED
{
        uint16 control_word;
        int32 target_pos;
        int32 profile_vel;
        int32 digital_output;
        int8 mode_of_operation;
} motor_rxpdo_t;

typedef struct PACKED
{
        uint16 status_word;
        int32 actual_pos;
        int32 actual_vel;
        int32 digital_inputs;
        uint16 error_code;
        int8 mode_of_operation_display;
} motor_txpdo_t;

typedef struct
{
        std::atomic<int> targetPos;
        std::atomic<bool> reached;
        std::atomic<int> inc;
} controlParam_t;

// 13 bits -> 8192 increments/revolution
// inc to deg: inc/8192 -> 1 rev -> 360 deg

// deg to inc (knee motor)
#define DEG_TO_INC_KNEE(DEG) (1100*(8192/360)*DEG);
#define INC_TO_DEG_KNEE(INC) ((INC/1100)/(8192/360));

void exchange();

enum Status {
        ABORT,
        CONT,
        FORWARD
};

enum Mode {
        PP,
        CSP
};

#endif