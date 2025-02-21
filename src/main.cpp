#include <iostream>
#include <stdexcept>
#include "../include/include.h"
#include "Master.hpp"
#include "Motor.hpp"


int main(int argc, char *argv[]) {

    if (argc > 1){
        // Network interface name is given as argument when executing.
        char *ifname = argv[1];

        // Initialize Master
        EthercatMaster *master = new EthercatMaster();
        master->init(ifname);

        // Initialize and configure slaves
        EthercatMotor *knee = new EthercatMotor(1);
        EthercatMotor *thigh = new EthercatMotor(2);
        knee->configure();
        thigh->configure();
        master->knee = knee;
        master->thigh = thigh;

        // Transition to SafeOP then to OP
        master->transitionToOperational();

        // Set PDO struct pointers
        master->mapPDOStructs();

        if (master->stateOP){

            // Move knee to home position
            int kneeHome = knee->getHomeOffset();
            knee->moveRelPP(kneeHome, 50);
            // thigh->moveRelPP(-10000, 50);

            master->takeInputs();
            master->controlLoop();
            master->stopInputs();
            master->stopControl();
            
            // master->sine(2, 30000, 20000, 0.1);
        }

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
