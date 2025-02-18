#include <soem/ethercat.h>
#include <iostream>
#include "../include/include.h"
using namespace std;

///////////////////// Wrapper functions for Mailbox communication //////////////////////

///////////// write unsigned /////////////
int write_sdo_u8(uint16_t slave, uint16 index, uint8 subindex, uint8 data)
{
    int wkc = ec_SDOwrite(slave, index, subindex, false, sizeof(data), &data, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO WRITE ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}
int write_sdo_u16(uint16 slave, uint16 index, uint8 subindex, uint16 data)
{
    int wkc = ec_SDOwrite(slave, index, subindex, false, sizeof(data), &data, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO WRITE ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}
int write_sdo_u32(uint16 slave, uint16 index, uint8 subindex, uint32 data)
{
    int wkc = ec_SDOwrite(slave, index, subindex, false, sizeof(data), &data, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO WRITE ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}

//////////// write signed /////////////
int write_sdo_s8(uint16 slave, uint16 index, uint8 subindex, int8 data)
{
    int wkc = ec_SDOwrite(slave, index, subindex, false, sizeof(data), &data, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO WRITE ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}
int write_sdo_s16(uint16 slave, uint16 index, uint8 subindex, int16 data)
{
    int wkc = ec_SDOwrite(slave, index, subindex, false, sizeof(data), &data, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO WRITE ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}
int write_sdo_s32(uint16 slave, uint16 index, uint8 subindex, int32 data)
{
    int wkc = ec_SDOwrite(slave, index, subindex, false, sizeof(data), &data, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO WRITE ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}

//////////// read unsigned /////////////
int read_sdo_u8(uint16 slave, uint16 index, uint8 subindex, uint8 *pdata)
{
    int size = sizeof(*pdata);
    int wkc = ec_SDOread(slave, index, subindex, false, &size, pdata, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO READ ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}

int read_sdo_u16(uint16 slave, uint16 index, uint8 subindex, uint16 *pdata)
{
    int size = sizeof(*pdata);
    int wkc = ec_SDOread(slave, index, subindex, false, &size, pdata, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO READ ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}

int read_sdo_u32(uint16 slave, uint16 index, uint8 subindex, uint32 *pdata)
{
    int size = sizeof(*pdata);
    int wkc = ec_SDOread(slave, index, subindex, false, &size, pdata, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO READ ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}

/////////////  read signed ///////////////
int read_sdo_s8(uint16 slave, uint16 index, uint8 subindex, int8 *pdata)
{
    int size = sizeof(*pdata);
    int wkc = ec_SDOread(slave, index, subindex, false, &size, pdata, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO READ ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}

int read_sdo_s16(uint16 slave, uint16 index, uint8 subindex, int16 *pdata)
{
    int size = sizeof(*pdata);
    int wkc = ec_SDOread(slave, index, subindex, false, &size, pdata, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO READ ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}

int read_sdo_s32(uint16 slave, uint16 index, uint8 subindex, int32 *pdata)
{
    int size = sizeof(*pdata);
    int wkc = ec_SDOread(slave, index, subindex, false, &size, pdata, EC_TIMEOUTRXM);
    if (wkc == 0){
        cout << "[SDO READ ERROR] index: " << index << ", subindex: " << (int)subindex << endl;
    }
    return wkc;
}



// Send & receive process data, then sleep for cycle duration.
void exchange() {
    int wkc;
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    // std::cout << "Sent and received process data. Working Counter = " << wkc << std::endl;
    osal_usleep(CYCLE_TIME_MS * 1000); // 5 ms
}


// Read status word & check the state of the motor. Then set the control word accordingly to reach "operation enabled".
bool stateOperationEnabled(const uint16& status){
    uint16 lower4 = status & 0xf;
    return (lower4 == 7 && IS_BIT_SET(status, 5) && IS_BIT_CLEAR(status, 6));
}
bool stateSwitchedOn(const uint16& status){
    uint16 lower4 = status & 0xf;
    return (lower4 == 3 && IS_BIT_SET(status, 5) && IS_BIT_CLEAR(status, 6));
}
bool stateReadyswitchOn(const uint16& status){
    uint16 lower4 = status & 0xf;
    return (lower4 == 1 && IS_BIT_SET(status, 5) && IS_BIT_CLEAR(status, 6));
}
bool stateSwitchOnDisabled(const uint16& status){
    uint16 lower4 = status & 0xf;
    return (lower4 == 0 && IS_BIT_CLEAR(status, 5) && IS_BIT_SET(status, 6));
}
bool stateNotReadySwitchOn(const uint16& status){
    uint16 lower4 = status & 0xf;
    return (lower4 == 0 && IS_BIT_CLEAR(status, 5) && IS_BIT_CLEAR(status, 6));
}
bool stateFault(const uint16& status){
    uint16 lower4 = status & 0xf;
    return (lower4 == 8 && IS_BIT_CLEAR(status, 5) && IS_BIT_CLEAR(status, 6));
}
bool stateFaultReact(const uint16& status){
    uint16 lower4 = status & 0xf;
    return (lower4 == 0xf && IS_BIT_CLEAR(status, 5) && IS_BIT_CLEAR(status, 6));
}
bool stateQuickStop(const uint16& status){
    uint16 lower4 = status & 0xf;
    return (lower4 == 7 && IS_BIT_CLEAR(status, 5) && IS_BIT_CLEAR(status, 6));
}