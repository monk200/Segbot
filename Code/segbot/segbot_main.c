//#############################################################################
// FILE:   segbot_main.c
//
// TITLE:  Segbot
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398

// PREDEFINITIONS ----------------------------------------------------------------------------------------------------------------------
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM6A(float);
void setEPWM6B(float);
void setDACA(float);
void setDACB(float);
__interrupt void ADCA_ISR(void);
__interrupt void SPIB_isr(void);
void init_eQEPs(void);
void setupEPWM5(void);
void setupADC(void);
void setupDAC(void);
void setupSpib(void);
void serialRXA(serial_t *s, char data);

// VARIABLES ---------------------------------------------------------------------------------------------------------------------------
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint32_t SPIB_isr_count = 0;
// joystick readings
float adcina2Volts = 0;  // ADCINA2 is connected to pin 29 = VRX = LR
float adcina3Volts = 0;  // ADCINA3 is connected to pin 26 = VRY = UD
// MPU-9250 readings
float accelXreading = 0, accelYreading = 0, accelZreading = 0;
float gyroXreading = 0, gyroYreading = 0, gyroZreading = 0;
// wheel info
float leftWheel = 0, leftWheelPrev = 0;
float rightWheel = 0, rightWheelPrev = 0;
// control algorithm
float uLeft = 0, uRight = 0;
float K1 = -34.3159*(10.0/6.0), K2 = -3.6375*(10.0/6.0), K3 = -3.3290*(10.0/6.0);       // -30, -2.8, -1.0
float leftVel = 0, leftVelPrev = 0;
float rightVel = 0, rightVelPrev = 0;
float ubal = 0;
float wheelDiff = 0, wheelDiffPrev = 0;
float wheelDiffVel = 0, wheelDiffVelPrev = 0;
// turning
float turnRef = 0, turnRefPrev = 0;
float turnRate = 0, turnRatePrev = 0;
float turnError = 0, turnErrorPrev = 0;
float turnErrorInt = 0, turnErrorIntPrev = 0;
float turn = 0;     // positive is left, negative is right
float FwdBackOffset = 0;
float Kp = 3.0, Ki = 20.0, Kd = 0.08;       // 3.0, 20.0, 0.08
// variables for calibrating and finding the balancing point
float accelx_offset = 0, accely_offset = 0, accelz_offset = 0;
float gyrox_offset = 0, gyroy_offset = 0, gyroz_offset = 0;
float accelzBalancePoint = -.83;        // -.76
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

void main(void) {
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();
    InitGpio();

    // Blue LED on LuanchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    // LED3
    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;

    // LED4
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED5
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED6
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED7
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED8
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED9
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED10
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;

    // LED11
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

    // LED12
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

    // LED13
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;

    // LED14
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;

    // LED15
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;

    // LED16
    GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;

    // LED17
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED18
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED19
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED20
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED21
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED22
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED23
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //PushButton 1
    GPIO_SetupPinMux(122, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(122, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(123, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(123, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(124, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(124, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // My setup functions
    setupSpib();
    init_eQEPs();

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.ADCA1_INT = &ADCA_ISR;     // tells processor to call ADCA_ISR when ADCA2 interrupt occurs

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;       // ex2, q3, part b
    EDIS;    // This is needed to disable write to EALLOW protected registers

    // More of my setup functions
    setupEPWM5();       // Setup EPWM5 to be a timer to trigger ADCD conversion sequence
    setupADC();     // Setup SOC0 and SOC1 to trigger
    setupDAC();     // Enable DACA and DACB outputs

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);     // 1,000us = 1ms = 0.001s
    ConfigCpuTimer(&CpuTimer1, 200, 4000);     // 4,000us = 4ms = 0.004s
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
//    init_serial(&SerialD,115200,serialRXD);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6;      // SPIB_RX

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enables PIE interrupt 1.1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      // ADCA1, Table3-2 in ADC technical reference
    // Enable SPIB_RX interrupt: 6.3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    // setup EPWM6A and B with 20kHz carrier frequency
    EALLOW;
    EPwm6Regs.TBCTL.bit.CTRMODE = 0x0;     // Up-count mode
    EPwm6Regs.TBCTL.bit.FREE_SOFT = 0x2;      // Free run; 1x means use any value that isn't already in use
    EPwm6Regs.TBCTL.bit.PHSEN = 0x0;       // disable phase loading
    EPwm6Regs.TBCTL.bit.CLKDIV = 0x0;      // clock divide to 1
    EPwm6Regs.TBCTR = 0;     // start timers at zero
    EPwm6Regs.TBPRD = 2500;     // (1/20,000)/(1/50,000,000) = 2500 b/c main clock ctr is 50MHz and prd here is 20KHz
    EPwm6Regs.CMPA.bit.CMPA = 0;    // start duty cycle at 0%
    EPwm6Regs.CMPB.bit.CMPB = 0;
    EPwm6Regs.AQCTLA.bit.CAU = 0x1;    // when TBCTR = CMPB on up count, clear (force EPWMxA output low)
    EPwm6Regs.AQCTLB.bit.CBU = 0x1;
    EPwm6Regs.AQCTLA.bit.ZRO = 0x2;    // when TBCTR = 0, set high
    EPwm6Regs.AQCTLB.bit.ZRO = 0x2;
    EPwm6Regs.TBPHS.bit.TBPHS = 0;     // set phase to zero
    EDIS;
    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 1);     // set DRV1PWM from GPIO10 to EPWM6A
    GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 1);     // set DRV2PWM from GPIO11 to EPWM6B

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;
    EDIS;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while(1) {
        if (UARTPrint == 1 ) {
                //serial_printf(&SerialA,"wheels: (%.3f, %.3f) gyro: (%.3f, %.3f, %.3f) accel:(%.3f, %.3f, %.3f)\r\n", leftWheel, rightWheel, gyroXreading, gyroYreading, gyroZreading, accelXreading, accelYreading, accelZreading);
            serial_printf(&SerialA,"tilt_value: %.3f, gyro_value: %.3f, wheel vel: (%.3f, %.3f)\r\n", tilt_value, gyro_value, leftWheel, rightWheel);
            UARTPrint = 0;
        }
    }
}

// HELPER FUNCTIONS -----------------------------------------------------------------------------------------------------------------------

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 20 North South magnet poles in the encoder disk so 20 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 80 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 18.7.
    return (raw*(-TWOPI/(80*18.7)));
}

float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 20 North South magnet poles in the encoder disk so 20 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 80 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 18.7.
    return (raw*(TWOPI/(80*18.7)));
}

void setEPWM6A(float controleffort) {       // LEFT WHEEL
    controleffort = -controleffort;     // so that a positive u spins the wheels forwards
    // set control effort range
    if (controleffort > 10) controleffort = 10;
    if (controleffort < -10) controleffort = -10;

    // there was an issue when testing my board where the motor's pinout was different on the right wheel so the code here for forward and backwards are swapped
    if (controleffort >= 0) GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;    // if positive control effort, turn wheel forwards
    else GpioDataRegs.GPASET.bit.GPIO29 = 1;       // if negative control effort, turn wheel backwards

    EPwm6Regs.CMPA.bit.CMPA = (fabs(controleffort) /10.0) * EPwm6Regs.TBPRD;        // scales control effort to period and set for duty cycle
}

void setEPWM6B(float controleffort) {       // RIGHT WHEEL
    controleffort = -controleffort;     // so that a positive u spins the wheel forwards
    // set control effort range
    if (controleffort > 10) controleffort = 10;
    if (controleffort < -10) controleffort = -10;

    if (controleffort >= 0) GpioDataRegs.GPBSET.bit.GPIO32 = 1;    // if positive control effort, turn wheel forwards
    else GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;       // if negative control effort, turn wheel backwards

    EPwm6Regs.CMPB.bit.CMPB = (fabs(controleffort) /10.0) * EPwm6Regs.TBPRD;        // scales control effort to period and set for duty cycle
}

void setDACA(float dacouta0) {
    if (dacouta0 > 3.0) dacouta0 = 3.0;
    if (dacouta0 < 0.0) dacouta0 = 0.0;
    DacaRegs.DACVALS.bit.DACVALS = (int)(dacouta0/3.0 * 4095); // perform scaling of 0-3 to 0-4095
}

void setDACB(float dacouta1) {
    if (dacouta1 > 3.0) dacouta1 = 3.0;
    if (dacouta1 < 0.0) dacouta1 = 0.0;
    DacbRegs.DACVALS.bit.DACVALS = (int)(dacouta1/3.0 * 4095); // perform scaling of 0-3 to 0-4095
}

// INTERRUPT SERVICE ROUTINES AND RX TX BUFFERS -------------------------------------------------------------------------------------------

// ADCA hardware interrupt, called every 1ms
__interrupt void ADCA_ISR(void) {
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;

    // JOYSTICK READINGS
    float adca2out = AdcaResultRegs.ADCRESULT0;       // ADCINA2 is connected to pin 29 = VRX = LR
    float adca3out = AdcaResultRegs.ADCRESULT1;       // ADCINA3 is connected to pin 26 = VRY = UD
    adcina2Volts = adca2out * (3.0/4095.0);
    adcina3Volts = adca3out * (3.0/4095.0);

    // IMU READINGS
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;       // GPIO66 low to act as slave select
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;        // issue the SPIB_RX_INT when all values are in RX FIFO
    // get accel readings
    SpibRegs.SPITXBUF =  ((0x8000) | (0x3A00));      // read = 0x8000, register = 0x3A00 = INT_STATUS start
    SpibRegs.SPITXBUF = 0;      // full 16 bits of ACCEL_XOUT
    SpibRegs.SPITXBUF = 0;      // full 16 bits of ACCEL_YOUT
    SpibRegs.SPITXBUF = 0;      // full 16 bits of ACCEL_ZOUT
    SpibRegs.SPITXBUF = 0;      // temp
    // get gyro readings
    SpibRegs.SPITXBUF = 0;      // full 16 bits of GYRO_XOUT
    SpibRegs.SPITXBUF = 0;      // full 16 bits of GYRO_YOUT
    SpibRegs.SPITXBUF = 0;      // full 16 bits of GYRO_ZOUT

    // CLEAR INTERRUPT FLAG
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
}

// SPIB_RX interrupt
__interrupt void SPIB_isr(void) {
    SPIB_isr_count++;

    // RECIEVE DATA
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;     // deselect
    // read values
    int16_t temp = SpibRegs.SPIRXBUF;      // reads start for gyro
    int16_t accelXraw = SpibRegs.SPIRXBUF;      // reads full accel x
    int16_t accelYraw = SpibRegs.SPIRXBUF;      // reads full accel y
    int16_t accelZraw = SpibRegs.SPIRXBUF;      // reads full accel z
    temp = SpibRegs.SPIRXBUF;      // reads temp
    int16_t gyroXraw = SpibRegs.SPIRXBUF;       // reads full gyro x
    int16_t gyroYraw = SpibRegs.SPIRXBUF;       // reads full gyro y
    int16_t gyroZraw = SpibRegs.SPIRXBUF;       // reads full gyro z
    // manipulate values
    accelXreading = accelXraw*4.0/32767.0;      // scale to g (-4g to 4g)
    accelYreading = accelYraw*4.0/32767.0;
    accelZreading = accelZraw*4.0/32767.0;
    gyroXreading = gyroXraw*250.0/32767.0;      // scale to degrees/second (-250 to 250 degrees per sec)
    gyroYreading = gyroYraw*250.0/32767.0;
    gyroZreading = gyroZraw*250.0/32767.0;

    // SENSE MOTOR ANGLES
    leftWheel = -readEncLeft();     // disance in radians
    rightWheel = -readEncRight();        // polarities are to account for weird wiring in my encoder

    // CALIBRATE SENSORS
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1) {
        accelx_offset += accelXreading;
        accely_offset += accelYreading;
        accelz_offset += accelZreading;
        gyrox_offset += gyroXreading;
        gyroy_offset += gyroYreading;
        gyroz_offset += gyroZreading;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset /= 2000.0;
            accely_offset /= 2000.0;
            accelz_offset /= 2000.0;
            gyrox_offset /= 2000.0;
            gyroy_offset /= 2000.0;
            gyroz_offset /= 2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2) {
        accelXreading -= (accelx_offset);
        accelYreading -= (accely_offset);
        accelZreading -= (accelz_offset-accelzBalancePoint);
        gyroXreading -= gyrox_offset;
        gyroYreading -= gyroy_offset;
        gyroZreading -= gyroz_offset;
        // KALMAN FILTER
        float tiltrate = (gyroXreading*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;       // new prediction = old prediction + dt*gyro
        pred_P = kalman_P + Q;      // new uncertainty = old uncertainty + environmental uncertainty
        // Update Step
        z = -accelZreading;     // laggy accel
        y = z - pred_tilt;      // prediction error
        S = pred_P + R;     // covariance = new uncertainty + sensor noise uncertainty
        kalman_K = pred_P/S;        // kalman gain, decrases if measurements match the predicted state
        kalman_tilt = pred_tilt + kalman_K*y;       // new prediction
        kalman_P = (1 - kalman_K)*pred_P;       // new uncertainty
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = -readEncLeft();
        RightWheelArray[SpibNumCalls] = -readEncRight();
        // average last 4 readings, 1 for each ms
        if (SpibNumCalls >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            leftWheel = (LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            rightWheel = (RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }
    timecount++;
    if((timecount%200) == 0) {
        if(doneCal == 0) GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1; // Tell While loop to print
    }

    // EXITING INTERRUPT
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {
    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    // 125s/(s+125) -> (100z-100)/(z-0.6) for 0.004s to estimate wheel velocities
    leftVel = 0.6*leftVelPrev + 100*(leftWheel - leftWheelPrev);
    rightVel = 0.6*rightVelPrev + 100*(rightWheel - rightWheelPrev);

    // BALANCING CONTROL
    ubal = -K1*tilt_value - K2*gyro_value - K3*((leftVel + rightVel)/2.0);
    //ubal = 0;

    // TURN CONTROL
    wheelDiff = leftWheel - rightWheel;     // difference between wheel positions in radians
    // 250s/(s+250) -> (166.667z-166.667)/(z-0.333) for 0.004s
    wheelDiffVel = 0.333*wheelDiffVelPrev + 166.667*(wheelDiff - wheelDiffPrev);
    turnError = turnRef - wheelDiff;
    turnErrorInt = turnErrorIntPrev + 0.004*((turnError + turnErrorPrev)/2);
    turnRef = turnRefPrev + 0.004*((turnRate + turnRatePrev)/2);        // uncomment this line to use keyboard controls
    turn = Kp*turnError + Ki*turnErrorInt - Kd*wheelDiffVel;
    // integral windup
    if (fabs(turn) > 3) turnErrorInt = turnErrorIntPrev;
    // saturate
    if (turn >= 4) turn = 4;
    if (turn <= -4) turn = -4;

    // determine control effort
    uLeft = (ubal/2) + turn + FwdBackOffset;
    uRight = (ubal/2) - turn + FwdBackOffset;
    // saturate
    if (uLeft > 10) uLeft = 10;
    if (uLeft < -10) uLeft = -10;
    if (uRight > 10) uRight = 10;
    if (uRight < -10) uRight = -10;
    setEPWM6A(uLeft);
    setEPWM6B(uRight);

    // update past states
    leftVelPrev = leftVel;
    leftWheelPrev = leftWheel;
    rightVelPrev = rightVel;
    rightWheelPrev = rightWheel;
    wheelDiffPrev = wheelDiff;
    wheelDiffVelPrev = wheelDiffVel;
    turnErrorIntPrev = turnErrorInt;
    turnErrorPrev = turnError;
    turnRefPrev = turnRef;
    turnRatePrev = turnRate;

    numSWIcalls++;
    DINT;
}

// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
    if (data == 'a') {
        turnRate = turnRate - 0.2;
    } else if (data == 'd') {
        turnRate = turnRate + 0.2;
    } else if (data == 'w') {
        FwdBackOffset = FwdBackOffset - 0.2;
    } else if (data == 's') {
        FwdBackOffset = FwdBackOffset + 0.2;
    } else {
        turnRate = 0;
        FwdBackOffset = 0;
    }
}

// TIMER INTERRUPTS -----------------------------------------------------------------------------------------------------------------------

// cpu_timer0_isr - CPU Timer0 ISR
// called every 1ms
__interrupt void cpu_timer0_isr(void) {
    CpuTimer0.InterruptCount++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;     // Acknowledge this interrupt to receive more interrupts from group 1
}

// cpu_timer1_isr - CPU Timer1 ISR
// called every 4ms
__interrupt void cpu_timer1_isr(void) {
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void) {
    CpuTimer2.InterruptCount++;
}

// MAIN INITIALIZATIONS -------------------------------------------------------------------------------------------------------------------

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EQep1Regs.QPOSCNT = 0;
    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EQep2Regs.QPOSCNT = 0;
}

// Setup EPWM5 to be a timer to trigger ADCA channels 2 and 3 every 1 ms
void setupEPWM5(void) {
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (pulse is the same as trigger)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz. 50,000,000 (1/s) * .001 s = 50,000 prd
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; //unfreeze, and enter up count mode
    EDIS;
}

// Setup SOC0 and SOC1 to trigger ADCA2 and ADCA3
void setupADC(void) {
    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x2; //SOC0 will convert Channel ADCINA2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x3; //SOC1 will convert Channel ADCINA3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCB
    //AdcbRegs.ADCSOC0CTL.bit.CHSEL = ???; //SOC0 will convert Channel you choose Does not have to be B0
    //AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to be B1
    //AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have to be B2
    //AdcbRegs.ADCSOC2CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    //AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have to be B3
    //AdcbRegs.ADCSOC3CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    //AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = ???; //set to last SOC that is converted and it will set INT1 flag ADCB1
    //AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    //AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCD
    //AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0; // set SOC0 to convert pin D0
    //AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC0
    //AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1; //set SOC1 to convert pin D1
    //AdcdRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC1
    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    //AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
    //AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    //AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

// Initialize DACA and DACB
void setupDAC(void) {
    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    EDIS;
}

void setupSpib(void) {
    // Step 1 SPI SETUP ---------------------------------------------------------------------------------------------------
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0); // Set as GPIO2 and used as DAN777 SS
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO2 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO2 = 1; //Initially Set GPIO2/SS High so DAN777 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set prequalifier for SPI PINS
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // The prequalifier eliminates short noise spikes
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // by making sure the serial pin stays low for 3 clock periods.
    EDIS;

    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN777 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 0x1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 0x32; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLKs period
    // 0x32 is 50 in hex -> divide 50MHz by 50 = 1MHz
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 0x1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 0x1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 0x1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 0x1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00; //Set delay between transmits to 0 spi clocks. Needed by DAN777 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 0x1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 0x1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I dont think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; //Interrupt Level to 2 words or more received into FIFO causes interrupt

    // Step 2 Initialize MPU registers ---------------------------------------------------------------------------
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers
    SpibRegs.SPITXBUF = (0x1300 | 0x0000);       // To address 0x13 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);      // address 0x14 write 0x00, address 0x15 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);      // address 0x16 write 0x00, address 0x17 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0013);      // address 0x18 write 0x00, address 0x19 write 0x13
    SpibRegs.SPITXBUF = (0x0200 | 0x0000);      // address 0x1A write 0x02, address 0x1B write 0x00
    SpibRegs.SPITXBUF = (0x0800 | 0x0006);      // address 0x1C write 0x08, address 0x1D write 0x06
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);      // address 0x1E write 0x00, address 0x1F write 0x00

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    int i;
    float temp;
    for (i=0; i<7; i++) temp = SpibRegs.SPIRXBUF;      // read the number of garbage receive values off the RX FIFO to clear out the RX FIFO

    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // Step 3 Initialize MORE MPU registers! ------------------------------------------------------------------------------------------
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    SpibRegs.SPITXBUF = (0x2300 | 0x0000);      // to address 0x23 write 0x00
    SpibRegs.SPITXBUF = (0x4000 | 0x008C);      // address 0x24 write 0x40, address 0x25 write 0x8C
    SpibRegs.SPITXBUF = (0x0200 | 0x0088);      // address 0x26 write 0x02, address 0x27 write 0x88
    SpibRegs.SPITXBUF = (0x0C00 | 0x000A);      // address 0x29 write 0x0C, address 0x29 write 0x0A

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    for (i=0; i<4; i++) temp = SpibRegs.SPIRXBUF;       // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // Step 4 ONE MORE MPU REGISTER -----------------------------------------------------------------------------------------
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081);       // Write to address 0x2A the value 0x81
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // Step 5 Trust Dan's code -----------------------------------------------------------------------------------------
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x0000); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0000); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0000); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x0000); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0021); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
