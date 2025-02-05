#include <iostream>
#include <stdexcept>
#include "../include/include.h"
#include "Master.hpp"
#include "Motor.hpp"


int main() {

    EthercatMaster *master = new EthercatMaster();
    if (master->init() == 0) {
        master->close();
        return -1;
    }

    EthercatMotor *knee = new EthercatMotor(1);
    EthercatMotor *thigh = new EthercatMotor(2);

    knee->initialize();
    thigh->initialize();
    // knee->print_config();
    // thigh->print_config();

    if (master->transitionToOperational() == 0) {
        master->close();
        return -1;
    }

    knee->set_pdo((motor_rxpdo_t *)ec_slave[1].outputs, (motor_txpdo_t *)ec_slave[1].inputs);
    thigh->set_pdo((motor_rxpdo_t *)ec_slave[2].outputs, (motor_txpdo_t *)ec_slave[2].inputs);

    std::atomic<int> target_pos(0);
    std::atomic<bool> reached(true);
    std::thread knee_thread([knee, &target_pos, &reached]() {knee->moveRelCSPTh(target_pos, reached);});
    std::thread input_thread([knee, &target_pos, &reached]() {knee->takeInput(target_pos, reached);});

    knee_thread.join();
    input_thread.join();

    // int kneeHome = knee->getHomeOffset();
    // knee->moveRelPP(kneeHome, 100);
    
    // master->controlLoop(knee, thigh);
    // master->gait(knee, thigh, 30000, 20000, 0.1);

    // Close EtherCAT master delete pointers
    master->close();
    delete knee;
    delete thigh;
    delete master;

    return 0;
}
