#include <soem/ethercat.h>
#include <iostream>
#include "../include/include.h"
using namespace std;

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


void exchange() {
    int wkc;
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    // std::cout << "Sent and received process data. Working Counter = " << wkc << std::endl;
    osal_usleep(5000); // 5 ms
}

// int deg_to_inc(int deg) {
//     deg * 
// }