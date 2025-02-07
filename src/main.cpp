#include <iostream>
#include <stdexcept>
#include "../include/include.h"
#include "Master.hpp"
#include "Motor.hpp"


int main(int argc, char *argv[]) {

    if (argc > 1){
        char *ifname = argv[1];

        // Initialize Master
        EthercatMaster *master = new EthercatMaster();
        if (master->init(ifname) == 0) {
            master->close();
            return -1;
        }

        // Initialize and configure slaves
        EthercatMotor *knee = new EthercatMotor(1);
        EthercatMotor *thigh = new EthercatMotor(2);
        knee->configure();
        thigh->configure();

        // Transition to SafeOP then to OP
        if (master->transitionToOperational() == 0) {
            master->close();
            return -1;
        }

        // Set PDO pointers
        knee->set_pdo((motor_rxpdo_t *)ec_slave[1].outputs, (motor_txpdo_t *)ec_slave[1].inputs);
        thigh->set_pdo((motor_rxpdo_t *)ec_slave[2].outputs, (motor_txpdo_t *)ec_slave[2].inputs);

        // Enable motors (thigh motor has initial jerk)


        // Move knee to home position
        int kneeHome = knee->getHomeOffset();
        knee->moveRelPP(kneeHome, 100);

        std::atomic<bool> done = false;
        std::thread inputThread([master, knee, thigh, &done]() {master->takeInputs(knee->controlParam, thigh->controlParam, done);});
        std::thread controlThread([master, knee, thigh, &done]() {master->controlLoop(knee, thigh, knee->controlParam, thigh->controlParam, done);});
        
        inputThread.join();
        controlThread.join();
        
        // master->gait(knee, thigh, 30000, 20000, 0.1);

        // Close EtherCAT master delete pointers
        master->close();
        delete knee;
        delete thigh;
        delete master;
    }
    else {
        std::cout << "Usage: main [ifname]\nFor Raspberry Pi: [ifname] = eth0\nFor PC: [ifname] = enp4s0" << std::endl;
    }
    return 0;
}
