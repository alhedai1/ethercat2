#ifndef MASTER_HPP
#define MASTER_HPP

#include "../include/include.h"
#include "Motor.hpp"

class EthercatMaster {
    private:
    int expectedWKC;
    char IOmap[4096];
    int chk = 200;
    std::thread inputsThread;
    std::thread controlThread;

    public: 
    EthercatMaster() {}
    ~EthercatMaster() {}

    bool stateOP = false;
    bool done = false;
    EthercatMotor* knee;
    EthercatMotor* thigh;

    // Initialize EtherCAT
    void init(char *ifname) {
        if (ec_init(ifname)) {
            std::cout << "EtherCAT initialized" << std::endl;
            if (ec_config_init(false) > 0) {
                std::cout << ec_slavecount << " slaves found" << std::endl;
                if (ec_slavecount != NUM_MOTORS){
                    std::cout << "Some slaves were not found" << std::endl;
                    close();
                    exit(0);
                }
            }
            else{
                std::cout << "Could not find any slaves" << std::endl;
                close();
                exit(0);
            }
        }
        else {
            std::cout << "Could not initialize EtherCAT" << std::endl;
            exit(0);
        }        
    }

    void transitionToOperational() { // return 1 if successful, 0 if not
        // PreOP to SafeOP
        ec_config_map(&IOmap);
        ec_configdc();
        if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP) {
            std::cout << "Could not reach SafeOP state" << std::endl;
            ec_close();
            exit(0);
        }
        std::cout << "Slaves state to SafeOP" << std::endl;

        expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        std::cout << "Expected WKC: " << expectedWKC << std::endl;

        // ec_dcsync0(knee->getMotorId(), true, CYCLE_TIME_MS, 0);
        // ec_dcsync0(thigh->getMotorId(), true, CYCLE_TIME_MS, 0);

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
            ec_close();
            exit(0);
        }
        std::cout << "All slaves in Operational state." << std::endl;
        stateOP = true;
    }

    void mapPDOStructs(){
        knee->setPdo();
        thigh->setPdo();
    }

    // Follow sinusoidal trajectory w/ phase difference between motors to roughly simulate knee and thigh joints during gait
    void sine(int steps, double kneeAmp, double thighAmp, double frequency) {
        const double period = 1/frequency;
        const int kneeOffset = knee->currentPosition();
        const int thighOffset = thigh->currentPosition();
        double omega = 2 * M_PI * frequency;
        double scale;
        double scaleTime = 2.0;
        Status statk;
        Status statth;

        double duration = steps * period;
        auto startTime = std::chrono::steady_clock::now();

        while (1) {
            exchange();
            statk = knee->checkStatus();
            statth = thigh->checkStatus();
            if (statk == Status::CONT || statth == Status::CONT) {
                continue;
            }
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsedTime = now - startTime;
            double t = elapsedTime.count();
            if (t >= (duration + ((M_PI/4)/omega))){
                // knee->moveCspOnce(0);
                // thigh->moveCspOnce(0);
                // exchange();
                break;
            }

            if (t < scaleTime){
                scale = t / scaleTime;
            }
            else {
                scale = 1.0;
            }
            double targetPosition1 = scale * kneeAmp * sin((omega*t) - (M_PI/4)) + kneeOffset;
            double targetPosition2 = scale * thighAmp * sin(omega*t) + thighOffset;

            if ((omega*t) >= (M_PI/4)){
                knee->moveCspOnce(targetPosition1);
            }
            if (t < duration){
                thigh->moveCspOnce(targetPosition2);
            }
        }
    }

    void takeInputs(){

        done = false;

        inputsThread = std::thread ([&](){
            osal_usleep(500000);
            while(1){
                char choice;
                int kneeNewTarget;
                int thighNewTarget;

                std::cout << "Continue? [y/n] ";
                std::cin >> choice;
                if (choice == 'n'){
                    done = true;
                    break;
                }
                else if (choice != 'y'){
                    std::cout << "Invalid input! [y/n]" << std::endl;
                    continue;
                }
                std::cout << "Knee Target Position: ";
                std::cin >> kneeNewTarget;
                std::cout << "Thigh Target Position: ";
                std::cin >> thighNewTarget;

                if (kneeNewTarget != 0){
                    // kneeNewTarget = DEG_TO_INC_KNEE(kneeNewTarget);
                    knee->controlParam.inc = kneeNewTarget > 0 ? 10 : -10;
                    knee->controlParam.targetPos.store(knee->controlParam.targetPos.load() + kneeNewTarget);
                    knee->controlParam.reached = false;
                }
                if (thighNewTarget != 0){
                    thigh->controlParam.inc = thighNewTarget > 0 ? 10 : -10;
                    thigh->controlParam.targetPos.store(thigh->controlParam.targetPos.load() + thighNewTarget);
                    thigh->controlParam.reached = false;
                }
                while(!knee->controlParam.reached || !thigh->controlParam.reached){
                }
            }
        });
    }

    void stopInputs(){
        inputsThread.join();
    }

    void controlLoop() {

        controlThread = std::thread ([&](){
            int kneeCurrPos = knee->currentPosition();
            int thighCurrPos = thigh->currentPosition();
            knee->controlParam.targetPos = knee->currentPosition();
            thigh->controlParam.targetPos = thigh->currentPosition();
            while(1){
                if (done == true){
                    break;
                }
                exchange();
                Status statk = knee->checkStatus();
                Status statth = thigh->checkStatus();
                if (statk == Status::CONT || statth == Status::CONT){
                    continue;
                }
                if (!knee->controlParam.reached){
                    kneeCurrPos += knee->controlParam.inc;
                    if (((knee->controlParam.inc > 0 && kneeCurrPos >= knee->controlParam.targetPos) 
                        || (knee->controlParam.inc < 0 && kneeCurrPos <= knee->controlParam.targetPos))){
                        knee->controlParam.reached = true;
                    }
                }
                if (!thigh->controlParam.reached){
                    thighCurrPos += thigh->controlParam.inc;
                    if (((thigh->controlParam.inc > 0 && thighCurrPos >= thigh->controlParam.targetPos) 
                        || (thigh->controlParam.inc < 0 && thighCurrPos <= thigh->controlParam.targetPos))){
                        thigh->controlParam.reached = true;
                    }
                }
                knee->moveCspOnce(kneeCurrPos);
                thigh->moveCspOnce(thighCurrPos);
            }
        });
    }

    void stopControl(){
        controlThread.join();
    }

    void close() {
        ec_close();
        std::cout << "EtherCAT closed" << std::endl;
    }
};

#endif