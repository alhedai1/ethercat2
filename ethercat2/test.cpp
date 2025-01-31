#include <iostream>
#include <iomanip>
#include "include.hpp"
#include <thread>
#include <chrono>

char IOmap[4096];
uint32 data_bits;

// Motor home positions
int32 motor_home_positions[2] = {1000, 1000};

// Struct pointers for inputs/outputs
motor_rxpdo_t **motor_rxpdos = NULL;
motor_txpdo_t **motor_txpdos = NULL;

// Array for initial position of each motor
int32 motor_initial_positions[2];
// Array for start offset of each motor
int32 motor_start_offset[2];

int main() { 

    const char *ifname = "enp4s0";
    if (!ec_init(ifname)){
        std::cout << "ec_init failed" << std::endl;
        return -1;
    }
    if (ec_config_init(FALSE) == 0){ // puts slaves in PreOP state
        std::cout << "ec_config_init failed" << std::endl;
        return -1;
    }
    std::cout << ec_slavecount << " slaves found and configured." << std::endl;
    std::cout << "[PreOP] State of slaves after ec_config_init: " << ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) << std::endl;

    // Structs for RxPDO and TxPDO for each motor
    // Check if you need to use malloc here, or can just assign w/o malloc
    motor_rxpdos = (motor_rxpdo_t **)malloc(ec_slavecount * sizeof(motor_rxpdo_t *));
    motor_txpdos = (motor_txpdo_t **)malloc(ec_slavecount * sizeof(motor_txpdo_t *));

    // Mapping and transition to SafeOP state
    for (int i=1; i<ec_slavecount+1; i++){
        // ec_slave[i].PO2SOconfig = slave_config;
    }
    ec_config_map(&IOmap);
    ec_configdc();
    int state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    std::cout << "[SafeOP] State after ec_config_map: " << state << std::endl;

    // Transition to Operational State
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    int wk = ec_receive_processdata(EC_TIMEOUTRET);
    std::cout << "working counter: " << wk << std::endl;
    ec_writestate(0);
    int chk = 200;
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    
    printf("[OP] Master state: %d\n", ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE));

    // Check if any slaves are not in Operational state
    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        printf("Not all slaves in Operational state!\n");
        ec_readstate();
        for (int i=1; i<ec_slavecount+1; i++)
        {
            if (ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) != EC_STATE_OPERATIONAL)
            {
                printf("Slave: %d | State: %d\n", i, ec_slave[i].state);
            }
        }
        return -1;
    }
    printf("All slaves in Operational state.\n");

    int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

    // Print slaves actual positions
    for (int i=0; i< ec_slavecount; i++)
    {
        read_sdo_s32(i+1, OD_POSITION_ACTUAL, 0x00, &motor_initial_positions[i]);
        printf("Motor: %d, Initial Position: %d\n", i+1, motor_initial_positions[i]);
    }

    // Assign IO buffer pointers to RxPDO/TxPDO struct pointers (to easily access inputs/outputs)
    for (int i=0; i< ec_slavecount; i++)
    {
        motor_rxpdos[i] = (motor_rxpdo_t *)ec_slave[i+1].outputs;
        motor_txpdos[i] = (motor_txpdo_t *)ec_slave[i+1].inputs;
    }

    // Control Loop
    printf("Expected WKC: %d\n", expectedWKC);
    int wkc;

    // Control Loop
    while (0)
    {
        osal_usleep(5000);
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        std::cout << "working counter: " << wkc << std::endl;

        for (int i=0; i< ec_slavecount; i++)
        {
            int32 buffer = 0;
            read_sdo_s32(i+1, 0x6062, 0x00, &buffer);
            printf("Motor: %d, Current Position: %d, Target Position: %d, Demand Position: %d\n", i+1, motor_txpdos[i]->actual_pos, motor_rxpdos[i]->target_pos, buffer);
            // printf("Motor: %d, Current Velocity: %d\n", i+1, motor_txpdos[i]->actual_vel);
            // printf("Motor: %d, Error code: %d\n", i+1, motor_txpdos[i]->error_code);
            // printf("Motor: %d, Mode of OP display: %d\n", i+1, motor_txpdos[i]->mode_of_operation_display);
            // printf("Motor: %d, Status word: 0x%x\n", i+1, motor_txpdos[i]->status_word);
            // printf("Motor: %d, Control word: 0x%x\n", i+1, motor_rxpdos[i]->control_word);
            
            // Check STATUS WORD
            uint16 status = motor_txpdos[i]->status_word;
            uint16 lower_4 = status & 0x000F;
            printf("Motor: %d, Status word: 0x%x\n", i+1, status);

            if ((lower_4 | (status & (7 << 4))) != STATUS_OPERATION_ENABLED){
                if (status == STATUS_NOT_READY_SWITCH){
                    continue;
                }
                else if ((lower_4 | (status & (1 << 6))) == STATUS_SWITCH_ON_DISABLED){ // 0x240
                    printf("Switch on disabled.\n");
                    motor_rxpdos[i]->control_word = 0x6;
                    continue;
                }
                else if ((lower_4 | (status & (3 << 5))) == STATUS_READY_SWITCH_ON){ // 0x221
                    printf("Ready switch on.\n");
                    motor_rxpdos[i]->control_word = 7;
                    continue;
                }
                else if ((lower_4 | (status & (7 << 4))) == STATUS_SWITCH_ON){
                    printf("Switch on.\n");
                    motor_rxpdos[i]->control_word = 0xf;
                    continue;
                }
                else if ((lower_4 | (status & (3 << 5))) == STATUS_QUICK_STOP_A){
                    printf("Quick stop active.\n");
                    continue;
                }
                else if ((lower_4 | (status & (1 << 6))) == STATUS_FAULT_REACT){
                    printf("Fault react active.\n");
                    continue;
                }
                else if ((lower_4 | (status & (1 << 6))) == STATUS_FAULT){
                    printf("Fault.\n");
                    motor_rxpdos[i]->control_word = (1 << 7);
                    continue;
                }
                else {
                    printf("Unknown status: 0x%x\n", status);
                    return -1;
                }
            }
            else {
                printf("Operation Enabled.\n");

                // if (!(status & HOME_REACHED)){
                //     printf("Homing not complete.\n");
                //     continue;
                // }
                // else if (status & STATUS_HOME_ERROR){
                //     printf("Homing error.\n");
                //     return 1;
                // }
                // else if (!(status & HOME_ATTAINED)){
                //     printf("Homing starting.\n");
                //     motor_rxpdos[1]->control_word = CONTROL_WORD_HM_START;
                //     motor_rxpdos[1]->mode_of_operation = MODE_HM;
                // }
                // else if (status & HOME_ATTAINED){
                //     printf("Homing complete.\n");
                //     motor_rxpdos[0]->target_pos = 0;
                //     motor_rxpdos[0]->profile_vel = 5000;
                //     motor_rxpdos[0]->control_word = CONTROL_WORD_PP_CHANGE;
                //     motor_rxpdos[0]->mode_of_operation = MODE_PP;
                // }
                // else {
                //     continue;
                // }

                motor_rxpdos[0]->control_word = CONTROL_WORD_PP_CHANGE;
                motor_rxpdos[0]->mode_of_operation = MODE_PP;
                motor_rxpdos[0]->profile_vel = 10;
                motor_rxpdos[0]->target_pos = 100;

                // motor_rxpdos[1]->control_word = CONTROL_WORD_PP_CHANGE;
                // motor_rxpdos[1]->mode_of_operation = MODE_PP;
                // motor_rxpdos[1]->profile_vel = 50;
                // motor_rxpdos[1]->target_pos = motor2_pos;
            }
        }
    }
    return 0;
}

