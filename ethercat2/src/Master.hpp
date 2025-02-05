#ifndef MASTER_HPP
#define MASTER_HPP

#include "../include/include.h"
#include "Motor.hpp"

class EthercatMaster {
    private:
    int expectedWKC;
    char IOmap[4096];
    int chk = 200;


    public:
    EthercatMaster() {}
    ~EthercatMaster() {}

    int num_motors;

    // Initialize
    int init() {
        try {
            if (ec_init("enp4s0")) {
                std::cout << "EtherCAT initialized" << std::endl;
                if (ec_config_init(false) > 0) {
                    std::cout << ec_slavecount << " slaves found and initialized" << std::endl;
                    num_motors = ec_slavecount;
                    return 1;
                }
                std::cout << "Could not find slaves" << std::endl;
                return 0;
            }
            else {
                std::cout << "Could not initialize EtherCAT" << std::endl;
                return 0;
            }
        } catch (const std::exception &ex) {
            std::cerr << "Error: " << ex.what() << std::endl;
        }
        return 0;
    }

    int transitionToOperational() { // return 1 if successful, 0 if not
        // PreOP to SafeOP
        ec_config_map(&IOmap);
        ec_configdc();
        if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP) {
            std::cout << "Could not reach SafeOP state" << std::endl;
            return 0;
        }
        std::cout << "Slaves state to SafeOP" << std::endl;

        expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        std::cout << "Expected WKC: " << expectedWKC << std::endl;
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_slave[0].state = EC_STATE_OPERATIONAL;
        ec_writestate(0);
        do {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
        
        // Check if any slaves are not in Operational state
        if (ec_slave[0].state != EC_STATE_OPERATIONAL)
        {
            std::cout << "Not all slaves in Operational state!" << std::endl;
            ec_readstate();
            for (int i=1; i<=ec_slavecount; i++)
            {
                if (ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) != EC_STATE_OPERATIONAL)
                {
                    printf("Slave: %d | State: %d\n", i, ec_slave[i].state);
                }
            }
            return 0;
        }
        std::cout << "All slaves in Operational state." << std::endl;
        return 1;
    }

    void moveAll(EthercatMotor &knee, int kneePosition, int kneeVelocity, EthercatMotor &thigh, int thighPosition, int thighVelocity) {
        knee.moveRelPP(kneePosition, kneeVelocity);
        thigh.moveRelPP(thighPosition, thighVelocity);
    }

    void gait(EthercatMotor *knee, EthercatMotor *thigh, double AMPLITUDE1, double AMPLITUDE2, double FREQUENCY) {
        const double PERIOD = 1/FREQUENCY;
        const int kneeOFFSET = knee->currentPosition();
        const int thighOFFSET = thigh->currentPosition();
        double omega = 2 * M_PI * FREQUENCY; // Angular frequency
        double scale = 0.0;
        double ramp_time = 2.0;
        auto start_time = std::chrono::steady_clock::now();
        Status statk;
        Status statth;

        while (1) {
            exchange();
            // thigh->print_torque();
            statk = knee->check_status();
            statth = thigh->check_status();
            if (statk == Status::CONT || statth == Status::CONT) {
                continue;
            }
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - start_time;
            double t = elapsed.count();
            if (t >= (PERIOD + ((M_PI/4)/omega))){
                knee->moveToPosition(0);
                thigh->moveToPosition(0);
                exchange();
                break;
            }

            if (t < ramp_time){
                scale = t / ramp_time;
            }
            else {
                scale = 1.0;
            }

            double target_position1 = scale * AMPLITUDE1 * sin((omega*t) - (M_PI/4)) + kneeOFFSET;
            double target_position2 = scale * AMPLITUDE2 * sin(omega*t) + thighOFFSET;

            // if ((omega*t) < (M_PI/4)){
            //     target_position1 = 0;
            // }
            if ((omega*t) >= (M_PI/4)){
                knee->moveToPosition(target_position1);
            }
            if (t < PERIOD){
                thigh->moveToPosition(target_position2);
            }
        }
    }

    void controlLoop(EthercatMotor *knee) {
        bool takingInput = true;
        while (takingInput) {
            int kneeTargetPosition;
            std::cout << "Knee Target Position: ";
            std::cin >> kneeTargetPosition;

            

            char choice;
            std::cout << "Continue? (y/n)";
            std::cin >> choice;
            if (choice != 'y' && choice != 'Y'){
                break;
            }
        }

    }

    void close() {
        ec_close();
        std::cout << "EtherCAT closed" << std::endl;
    }
};

#endif