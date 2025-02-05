#include <iostream>
#include <stdexcept>
#include "include.h"
#include "Motor.hpp"
#include <chrono>
#include <cmath>

char IOmap[4096];
int expectedWKC;
int chk = 200;

// int pos1 = 0;
// int inc = 50000;
// int pvel = 200;
Status stat1;
Status stat2;
bool done1 = false;
bool done2 = false;

const double AMPLITUDE1 = 50000;     // Amplitude in encoder counts
const double AMPLITUDE2 = 20000;
const double FREQUENCY = 0.1;     // Frequency in Hz
const double PERIOD = 1/FREQUENCY;
const int CYCLE_TIME_MS = 10;     // EtherCAT cycle time in milliseconds

int main() {
    try {
        // Initialize the EtherCAT master
        if (ec_init("enp4s0")) {
            std::cout << "EtherCAT initialized" << std::endl;

            if (ec_config_init(false) > 0) { // Slaves in PreOP now
                std::cout << ec_slavecount << " slaves found and configured" << std::endl;

                EthercatMotor motor1(1);
                EthercatMotor motor2(2);
                
                motor1.initialize();
                motor2.initialize();
                motor1.print_config();
                motor2.print_config();

                // PreOP to SafeOP
                ec_config_map(&IOmap);
                ec_configdc();
                ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
                std::cout << "Slaves state to SafeOP." << std::endl;

                // SafeOP to OP
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
                
                printf("[OP] Master state: %d\n", ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE));

                // Check if any slaves are not in Operational state
                if (ec_slave[0].state != EC_STATE_OPERATIONAL)
                {
                    printf("Not all slaves in Operational state!\n");
                    ec_readstate();
                    for (int i=1; i<=ec_slavecount; i++)
                    {
                        if (ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) != EC_STATE_OPERATIONAL)
                        {
                            printf("Slave: %d | State: %d\n", i, ec_slave[i].state);
                        }
                    }
                    return -1;
                }
                std::cout << "All slaves in Operational state." << std::endl;

                motor1.set_pdo((motor_rxpdo_t *)ec_slave[1].outputs, (motor_txpdo_t *)ec_slave[1].inputs);
                motor2.set_pdo((motor_rxpdo_t *)ec_slave[2].outputs, (motor_txpdo_t *)ec_slave[2].inputs);

                // Print Initial Position
                motor1.print_current_position();
                motor2.print_current_position();

                bool home_reached1 = false;
                // motor1.changeMainSensor(0x4025111);
                // int home_position1 = motor1.getHomePosition();

                bool gait = true;
                const int OFFSET1 = motor1.currentPosition();
                const int OFFSET2 = motor2.currentPosition();
                double omega = 2 * M_PI * FREQUENCY; // Angular frequency
                double scale = 0.0;
                double ramp_time = 2.0;
                auto start_time = std::chrono::steady_clock::now();

                while (0){
                    // motor1.print_current_position();
                    motor1.print_current_position();
                }

                // motor1.movePP(-50000, 100);

                while (0) {
                    // motor2.print_current_position();
                    exchange();
                    stat1 = motor1.check_status();
                    stat2 = motor2.check_status();
                    if (stat1 == Status::CONT || stat2 == Status::CONT){
                        continue;
                    }
                    if (gait) {
                        auto now = std::chrono::steady_clock::now();
                        std::chrono::duration<double> elapsed = now - start_time;
                        double t = elapsed.count();
                        // if (t >= (PERIOD + ((M_PI/4)/omega))){
                        //     break;
                        // }
                        if (t < ramp_time){
                            scale = t / ramp_time;
                        }
                        else {
                            scale = 1.0;
                        }
                        double target_position1 = scale * AMPLITUDE1 * sin((omega*t) - (M_PI/4)) + OFFSET1;
                        double target_position2 = scale * AMPLITUDE2 * sin(omega * t) + OFFSET2;
                        if ((omega*t) < (M_PI/4)){
                            target_position1 = 0;
                        }
                        done1 = motor1.moveToPosition(target_position1);
                        // if (t < PERIOD){
                        done2 = motor2.moveToPosition(target_position2);
                        // }
                    }
                    // else if (!home_reached1) {
                    //     if (motor1.homing(home_position1)) {
                    //         home_reached1 = true;
                    //         std::cout << "here" << std::endl;
                    //         motor1.changeMainSensor(0x4011111);
                    //         break;
                    //     }
                    // }
                }

                // Close the EtherCAT master
                ec_close();
            } else {
                throw std::runtime_error("No slaves found");
            }
        } else {
            return -1;
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    return 0;
}
