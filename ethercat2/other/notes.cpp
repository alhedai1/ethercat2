

// this works
const double AMPLITUDE1 = 100000;     // Amplitude in encoder counts
const double AMPLITUDE2 = 10000;
const double FREQUENCY = 0.1;     // Frequency in Hz
// int OFFSET;       // Center position in encoder counts
const int CYCLE_TIME_MS = 10;     // EtherCAT cycle time in milliseconds


// home position of motor 1: 5945


// 5900 -> 58300
// 5890 -> 72000

// 5891 -> 0

// 0000 0110 0011 0111
// 0000 1110 0011 0111

// limit of first motor: (highest position it can go)
// SSI position raw value: 6015 (highest SSI value)
// when jig moves down, incremental encoder's value increases, and SSI value decreases

// inc: 0 -> 50000
// ssi: 5944 -> 5902

// inc: 0 -> 50000
// ssi: 5902 -> 5856

// inc: 0 -> -50000
// ssi: 5856 -> 5903

// inc: 0 -> -50000
// ssi: 5903 -> 5943

// inc: 0 -> 25000
// ssi: 5943 -> 5924

// 50000 inc = 40~45 abs (almost)
// 1100 inc = 1 abs (almost)


// auto now = std::chrono::steady_clock::now();
//             std::chrono::duration<double> elapsed = now - start_time;
//             double t = elapsed.count();
//             // if (t >= (PERIOD + ((M_PI/4)/omega))){
//             //     break;
//             // }
//             if (t < ramp_time){
//                 scale = t / ramp_time;
//             }
//             else {
//                 scale = 1.0;
//             }
//             double target_position1 = scale * AMPLITUDE1 * sin((omega*t) - (M_PI/4)) + OFFSET1;
//             double target_position2 = scale * AMPLITUDE2 * sin(omega * t) + OFFSET2;
//             if ((omega*t) < (M_PI/4)){
//                 target_position1 = 0;
//             }
//             done1 = motor1.moveToPosition(target_position1);
//             // if (t < PERIOD){
//             done2 = motor2.moveToPosition(target_position2);

// 8192/360 = increments for 1 degree
// 1 degree = 8192/360 abs = 1100*(8192/360) inc


during second moveRelPP call:
	target reached bit is on
	setpoint acknowledged bit is on
