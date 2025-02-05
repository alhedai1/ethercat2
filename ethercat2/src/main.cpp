#include <iostream>
#include <stdexcept>
#include "../include/include.h"
#include "Master.hpp"
#include "Motor.hpp"


int main() {

    // Initialize Master
    EthercatMaster *master = new EthercatMaster();
    if (master->init() == 0) {
        master->close();
        return -1;
    }

    // Initialize and configure slaves
    EthercatMotor *knee = new EthercatMotor(1);
    EthercatMotor *thigh = new EthercatMotor(2);
    knee->initialize();
    thigh->initialize();

    // Transition to OP
    if (master->transitionToOperational() == 0) {
        master->close();
        return -1;
    }

    // Set PDO pointers
    knee->set_pdo((motor_rxpdo_t *)ec_slave[1].outputs, (motor_txpdo_t *)ec_slave[1].inputs);
    thigh->set_pdo((motor_rxpdo_t *)ec_slave[2].outputs, (motor_txpdo_t *)ec_slave[2].inputs);

    // Move knee to home position
    int kneeHome = knee->getHomeOffset();
    knee->moveRelPP(kneeHome, 100);

    // Input thread & Control thread
    std::atomic<int> kneeTarget(knee->currentPosition()), thighTarget(thigh->currentPosition());
    std::atomic<bool> kneeReached(true), thighReached(true);
    std::atomic<int> kneeInc, thighInc;
    std::thread inputThread([master, &kneeTarget, &thighTarget, &kneeInc, &thighInc, &kneeReached, &thighReached]() 
                            {master->takeInputs(kneeTarget, thighTarget, kneeInc, thighInc, kneeReached, thighReached);});
    std::thread controlThread([master, knee, thigh, &kneeTarget, &thighTarget, &kneeInc, &thighInc, &kneeReached, &thighReached]() 
                            {master->controlLoop(knee, thigh, kneeTarget, thighTarget, kneeInc, thighInc, kneeReached, thighReached);});
    controlThread.join();
    inputThread.join();

    // master->gait(knee, thigh, 30000, 20000, 0.1);

    // Close EtherCAT master delete pointers
    master->close();
    delete knee;
    delete thigh;
    delete master;

    return 0;
}
