#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <iostream>
#include "../include/include.h"

class EthercatMotor {
private:
    int motor_id; // 1, 2, ... (0 is master)
    motor_rxpdo_t *motor_rxpdo; // pointer to outputs
    motor_txpdo_t *motor_txpdo; // pointer to inputs

public:
    EthercatMotor(int id) : motor_id(id) {}
    ~EthercatMotor(){}

    controlParam_t controlParam;

    void configure() {
        std::cout << "Initializing motor " << motor_id << std::endl;
        // RxPDO assign
        write_sdo_u8(motor_id, OD_RXPDO_ASSIGN, 0x00, 1);
        write_sdo_u16(motor_id, OD_RXPDO_ASSIGN, 0x01, OD_RXPDO_MAP_1);

        // TxPDO assign
        write_sdo_u8(motor_id, OD_TXPDO_ASSIGN, 0x00, 1);
        write_sdo_u16(motor_id, OD_TXPDO_ASSIGN, 0x01, OD_TXPDO_MAP_1);

        // // RxPDO mapping (outputs)
        write_sdo_u8(motor_id, OD_RXPDO_MAP_1, 0x00, 0); // set no. of mapped object to 0 before mapping
        write_sdo_u32(motor_id, OD_RXPDO_MAP_1, 0x01, OD_CONTROL_WORD_MAP);
        write_sdo_u32(motor_id, OD_RXPDO_MAP_1, 0x02, OD_TARGET_POSITION_MAP);
        write_sdo_u32(motor_id, OD_RXPDO_MAP_1, 0x03, OD_PROFILE_VELOCITY_MAP);
        write_sdo_u32(motor_id, OD_RXPDO_MAP_1, 0x04, OD_DIGITAL_OUTPUTS_MAP);
        write_sdo_u32(motor_id, OD_RXPDO_MAP_1, 0x05, OD_MODE_OF_OPERATION_MAP);
        write_sdo_u8(motor_id, OD_RXPDO_MAP_1, 0x00, 5);

        // // TxPDO mapping (inputs)
        write_sdo_u8(motor_id, OD_TXPDO_MAP_1, 0x00, 0); // set no. of mapped object to 0 before mapping
        write_sdo_u32(motor_id, OD_TXPDO_MAP_1, 0x01, OD_STATUS_WORD_MAP);
        write_sdo_u32(motor_id, OD_TXPDO_MAP_1, 0x02, OD_POSITION_ACTUAL_MAP);
        write_sdo_u32(motor_id, OD_TXPDO_MAP_1, 0x03, OD_VELOCITY_ACTUAL_MAP);
        write_sdo_u32(motor_id, OD_TXPDO_MAP_1, 0x04, OD_DIGITAL_INPUTS_MAP);
        write_sdo_u32(motor_id, OD_TXPDO_MAP_1, 0x05, OD_ERROR_CODE_MAP);
        write_sdo_u32(motor_id, OD_TXPDO_MAP_1, 0x06, OD_MODE_OF_OPERATION_DISPLAY_MAP);
        write_sdo_u8(motor_id, OD_TXPDO_MAP_1, 0x00, 6);

        // Set software limit
        write_sdo_s32(motor_id, OD_SOFTWARE_LIMIT, 0x01, -2147483648);
        write_sdo_s32(motor_id, OD_SOFTWARE_LIMIT, 0x02, 2147483647);

        // Set polarity
        int polarity = 0;
        if (polarity == 0){
            write_sdo_u32(motor_id, OD_AXIS, 0x04, 0x00000000);
        }
        if (polarity == 1){
            write_sdo_u32(motor_id, OD_AXIS, 0x04, 0x00000001);
        }

        // Set Main Sensor and Process Value Reference for motor 1
        // if (motor_id == 1){                        
            // write_sdo_u32(motor_id, OD_AXIS, 0x02, 0x4025111); // main sensor = SSI (on gear), process val ref = on gear
        // }
        // write_sdo_u32(motor_id, OD_AXIS, 0x02, 0x4011111); // main sensor = inc (on motor), process val ref = on motor

        // Set interpolation time for CSP
        write_sdo_u8(motor_id, 0X60C2, 0x01, 5);
    }

    void print_config() {
        // Print RxPDO/TxPDO assign
        uint8 rxpdo_assign = 0;
        uint8 txpdo_assign = 0;
        read_sdo_u8(motor_id, OD_RXPDO_ASSIGN, 0x00, &rxpdo_assign);
        read_sdo_u8(motor_id, OD_TXPDO_ASSIGN, 0x00, &txpdo_assign);
        std::cout << "RxPDO assign: " << (int)rxpdo_assign << std::endl;
        std::cout << "TxPDO assign: " << (int)txpdo_assign << std::endl;

        // Print RxPDO/TxPDO objects
        uint8 rxpdo_num_obj= 0;
        uint8 txpdo_num_obj = 0;
        read_sdo_u8(motor_id, OD_RXPDO_MAP_1, 0x00, &rxpdo_num_obj);
        read_sdo_u8(motor_id, OD_TXPDO_MAP_1, 0x00, &txpdo_num_obj);
        std::cout << "RxPDO number of objects: " << (int)rxpdo_num_obj << std::endl;   
        for (int i=1; i<rxpdo_num_obj+1; i++){
            uint32 data = 0;
            read_sdo_u32(motor_id, OD_RXPDO_MAP_1, i, &data);
            printf("Object %d: 0x%x\n", i, data);
        }
        std::cout << "TxPDO number of objects: " << (int)txpdo_num_obj << std::endl;
        for (int i=1; i<txpdo_num_obj+1; i++){
            uint32 data = 0;
            read_sdo_u32(motor_id, OD_TXPDO_MAP_1, i, &data);
            printf("Object %d: 0x%x\n", i, data);
        }

        // Print software limit
        int32 min = 0;
        int32 max = 0;
        read_sdo_s32(motor_id, 0x607D, 0x01, &min);
        read_sdo_s32(motor_id, 0x607D, 0x02, &max);
        printf("Software min: %d\n", min);
        printf("Software max: %d\n", max);

        // Print manufacturer name, hardware version
        char name[5];
        int size = sizeof(name);
        ec_SDOread(motor_id, OD_MANUFACTURER, 0X00, FALSE, &size, &name, EC_TIMEOUTRXM);
        printf("Manufcaturer device name: %s\n", name);
        
        uint32 product_code = 0;
        read_sdo_u32(motor_id, OD_IDENTIFY_OBJECT, 0x02, &product_code);
        uint16 hardware = (product_code >> 16);
        printf("Hardware version: 0x%x\n", hardware);

        // Print max profile velocity, profile acceleration/deceleration
        uint32 max_profile_vel = 0;
        read_sdo_u32(motor_id, OD_MAX_PROFILE_VELOCITY, 0x00, &max_profile_vel);
        printf("Max profile velocity: %d\n", max_profile_vel);
        uint32 profile_acc = 0;
        read_sdo_u32(motor_id, OD_PROFILE_ACCELERATION, 0x00, &profile_acc);
        printf("Profile acceleration: %d\n", profile_acc);
        uint32 profile_dec = 0;
        read_sdo_u32(motor_id, OD_PROFILE_DECELERATION, 0x00, &profile_dec);
        printf("Profile deceleration: %d\n", profile_dec);

        // Print axis information
        uint32 control_structure = 0;
        read_sdo_u32(motor_id, OD_AXIS, 0x01, &control_structure);
        printf("Slave %d control structure: 0x%x\n", motor_id, control_structure);

        uint32 data_bits;
        read_sdo_u32(motor_id, 0x3012, 0x02, &data_bits);
        printf("SSI data bits: 0x%x\n", data_bits);

        int8 int_time_idx;
        read_sdo_s8(motor_id, 0X60C2, 0x02, &int_time_idx);
        printf("Interpolation time index: %d\n", int_time_idx);

        uint32 ssi_comm;
        read_sdo_u32(motor_id, 0X3012, 0x09, &ssi_comm);
        printf("SSI commutation offset value: %u\n", ssi_comm);

        uint32 ssi_val;
        read_sdo_u32(motor_id, 0X3012, 0x09, &ssi_val);
        printf("SSI position raw value: %u\n", ssi_val);

        // Incremental index position, try using this for homing
        int32 inc_index_pos;
        read_sdo_s32(motor_id, 0x3010, 0x04, &inc_index_pos);
        printf("Incremental Encoder Index Position: %d\n", inc_index_pos);

        uint32 axis_config;
        read_sdo_u32(motor_id, OD_AXIS, 0x04, &axis_config);
        printf("Axis configuration: 0x%x\n", axis_config);

        uint32 nominal_current;
        read_sdo_u32(motor_id, 0x3001, 0x01, &nominal_current);
        printf("Nominal Current: %u\n", nominal_current);

        uint32 torque_constant;
        read_sdo_u32(motor_id, 0x3001, 0x05, &torque_constant);
        printf("Torque Constant: %u\n", torque_constant);

        uint8 polepairs;
        read_sdo_u8(motor_id, 0x3001, 0x03, &polepairs);
        printf("Pole pairs: %d\n", polepairs);
    }

    void set_pdo(motor_rxpdo_t *outputsP, motor_txpdo_t *inputsP) {
        motor_rxpdo = outputsP;
        motor_txpdo = inputsP;
    }

    void print_current_position() {
        int actual_pos;
        int demand;
        read_sdo_s32(motor_id, OD_POSITION_ACTUAL, 0x00, &actual_pos);
        read_sdo_s32(motor_id, OD_POSITION_DEMAND, 0x00, &demand);
        uint32 ssi_val;
        read_sdo_u32(motor_id, 0X3012, 0x09, &ssi_val);
        printf("Target Position: %d, Actual Position: %d, Demand Position: %d, SSI position raw value: %u\n", motor_rxpdo->target_pos, actual_pos, demand, ssi_val);
    }

    void print_torque() {
        int16 torque;
        read_sdo_s16(motor_id, OD_TORQUE_ACTUAL, 0x00, &torque);
        printf("Actual Torque: %d\n", torque);
    }

    int currentPosition(){
        int actual_pos;
        read_sdo_s32(motor_id, OD_POSITION_ACTUAL, 0x00, &actual_pos);
        return actual_pos;
    }

    void printRxPDO() {
        printf("Control word: 0x%x\n", motor_rxpdo->control_word);
        printf("Mode of operation: 0x%x\n", motor_rxpdo->mode_of_operation);
        printf("Target position: 0x%x\n", motor_rxpdo->target_pos);
        printf("Profile velocity: 0x%x\n", motor_rxpdo->profile_vel);
    }

    void printTxPDO() {
        printf("Status word: 0x%x\n", motor_txpdo->status_word);
        printf("Mode of operation display: 0x%x\n", motor_txpdo->mode_of_operation_display);
        printf("Actual position: 0x%x\n", motor_txpdo->actual_pos);
        printf("Actual velocity: 0x%x\n", motor_txpdo->actual_vel);
        printf("Error code: 0x%x\n", motor_txpdo->error_code);
    }

    Status check_status() {
        // Check STATUS WORD
        uint16 status;
        uint16 lower_4;
        status = motor_txpdo->status_word;
        lower_4 = status & 0x000F;
        // std::cout << "Motor " << motor_id << " Status word: 0x" << std::hex << status << std::dec << std::endl;

        if ((lower_4 | (status & (7 << 4))) != STATUS_OPERATION_ENABLED){
            if (status == STATUS_NOT_READY_SWITCH){
                return Status::CONT;
            }
            else if ((lower_4 | (status & (1 << 6))) == STATUS_SWITCH_ON_DISABLED){ // 0x240
                printf("Switch on disabled.\n");
                motor_rxpdo->control_word = 0x6;
                return Status::CONT;
            }
            else if ((lower_4 | (status & (3 << 5))) == STATUS_READY_SWITCH_ON){ // 0x221
                printf("Ready switch on.\n");
                motor_rxpdo->control_word = 7;
                return Status::CONT;
            }
            else if ((lower_4 | (status & (7 << 4))) == STATUS_SWITCH_ON){
                printf("Switch on.\n");
                motor_rxpdo->control_word = 0xf;
                return Status::CONT;
            }
            else if ((lower_4 | (status & (3 << 5))) == STATUS_QUICK_STOP_A){
                printf("Quick stop active.\n");
                return Status::CONT;
            }
            else if ((lower_4 | (status & (1 << 6))) == STATUS_FAULT_REACT){
                printf("Fault react active.\n");
                return Status::CONT;
            }
            else if ((lower_4 | (status & (1 << 6))) == STATUS_FAULT){
                printf("Fault.\n");
                motor_rxpdo->control_word = (1 << 7);
                return Status::CONT;
            }
            else {
                printf("Unknown status: 0x%x\n", status);
                return Status::CONT;
            }
        }
        else {
            // std::cout << "Operation Enabled." << std::endl;
            return Status::FORWARD;
        }
        if ((motor_txpdo->mode_of_operation_display == MODE_PP) && (status && (1<<12))){
            motor_rxpdo->control_word = CONTROL_WORD_PP_NEW;
            motor_rxpdo->mode_of_operation = MODE_PP;
            return Status::CONT;
        }
    }

    void moveCspOnce(int position) { // send one CSP command
        motor_rxpdo->control_word = CONTROL_WORD_CSP;
        motor_rxpdo->mode_of_operation = MODE_CSP;
        motor_rxpdo->target_pos = position;
    }

    // Horizontal position for knee motor is almost 5900 (SSI)
    int getHomeOffset() {
        uint32 ssi_val;
        read_sdo_u32(1, 0X3012, 0x09, &ssi_val);
        int diff_ssi = 5900 - ssi_val;
        int diff_inc = diff_ssi * -1100;
        return diff_inc;
    }

    void moveRelPP(int position, int velocity) { // move to target position using PP
        bool set = false;
        int initial_pos = motor_txpdo->actual_pos;
        // int targetReached = 0;
        while (1) {
            exchange();
            // print_current_position();
            Status stat = check_status();
            if (stat == Status::CONT){
                continue;
            }
            if (!set){
                motor_rxpdo->mode_of_operation = MODE_PP;
                motor_rxpdo->target_pos = position + initial_pos;
                motor_rxpdo->profile_vel = velocity;
                motor_rxpdo->control_word = CONTROL_WORD_PP_CHANGE;
                set = true;
            }
            // targetReached = (motor_txpdo->status_word & 0x400) >> 10;
            if ((motor_rxpdo->target_pos) == motor_txpdo->actual_pos){
                break;
            }
        }
    }

    // void moveRelCSP(int relPos) { 
    //     int pos = currentPosition();
    //     int target = pos + relPos; // 5000, 10000
    //     // int diff = position - currentPosition();
    //     int diff = relPos;
    //     int sign = diff/abs(diff);
    //     int inc = sign*100;
    //     int demand;
    //     read_sdo_s32(motor_id, OD_POSITION_DEMAND, 0x00, &demand);
    //     printf("Current position: %d, Target position: %d\n", pos, target);
    //     motor_rxpdo->control_word = 0x0;
    //     motor_rxpdo->mode_of_operation = MODE_CSP;
    //     motor_rxpdo->target_pos = currentPosition();
    //     exchange();
    //     while (1) {
    //         exchange();
    //         Status stat = check_status();
    //         if (stat == Status::CONT){
    //             continue;
    //         }
    //         if ((sign > 0 && pos > target)){
    //             printf("here\n");
    //             printf("sign: %d, pos: %d, target: %d\n", sign, pos, target);
    //             motor_rxpdo->target_pos = currentPosition();
    //             break;
    //         }
    //         motor_rxpdo->control_word = CONTROL_WORD_CSP;
    //         motor_rxpdo->mode_of_operation = MODE_CSP;
    //         motor_rxpdo->target_pos = pos;
    //         pos += inc;
    //     }
    // }

    void takeInput(std::atomic<int>& target_pos, std::atomic<int>& inc, std::atomic<bool>&reached){
        while(1){
            int new_target;
            std::cin >>new_target;
            inc = new_target > 0 ? 10 : -10;
            target_pos.store(target_pos.load() + new_target);
            reached = false;
            while(!reached){ 
            }
        }
    }

    void moveRelCSPTh(std::atomic<int>& target_pos, std::atomic<int>& inc, std::atomic<bool>&reached){
        int current_pos = currentPosition();
        while(1){
            exchange();
            Status stat = check_status();
            if (stat == Status::CONT){
                continue;
            }
            if (!reached){
                current_pos += inc;
                if ((inc > 0 && current_pos >= target_pos || (inc < 0 && current_pos <= target_pos))){
                    reached = true;
                }
            }
            motor_rxpdo->control_word = CONTROL_WORD_CSP;
            motor_rxpdo->mode_of_operation = MODE_CSP;
            motor_rxpdo->target_pos = current_pos;
        }
    }
};

#endif