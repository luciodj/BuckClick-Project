/*
 * Project: Buck-Click
 * File:    main.c
 * Author:  Lucio Di Jasio
 * Compiled using: XC8 v. 1.32
 *
 * Updated to support Buck-Click board v1.01
 *
 */

//*****************************************************************************
// Include and Header files
//*****************************************************************************
#include <xc.h>
#include "main.h"

//*****************************************************************************
// PIC Configuration
//*****************************************************************************
__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & CP_OFF & CPD_OFF &
               BOREN_ON & CLKOUTEN_OFF & IESO_OFF & FCMEN_ON);

__CONFIG(WRT_OFF & PLLEN_OFF & STVREN_ON & BORV_HI & LPBOR_OFF & LVP_OFF);

#define I2C_DEV_ADD  0x10    // top nibble only, unique I2C address
#define I2C_ADD_MASK 0xF0    // only address bits
#define I2C_CMD_MASK 0X0E    // only command bits
#define I2C_RW_MASK  0x01    // only direction bit

#define S_START     0       // default state
#define S_READ      1       // reading data from device
#define S_WRITE     2       // writing data to device

#define P_INT   LATB0       // MikroBus INT signal
#define P_LED   LATA7       // Green LED
#define P_PWM   LATC2       // PWM pin controlling slope compensation

/*****************************************************************************
 * INTERRUPT Service Routine
 *****************************************************************************/
void interrupt isr(void)
{
    unsigned char data;
    unsigned char *p;

    static char state = 0;
    static char s_cmd;
    static char s_count;
    static char s_error = 0;
    static char s_bank;
    static char s_reg;
    static char temp;

    // I2C state machine
    SSP1IF = 0;             // clear interrupt flag
    data = SSP1BUF;
    
    switch( state)
    {
        case S_READ:    // reading data from device
            if ( SSP1CON2bits.ACKSTAT == 0)
            {
                // send out pre-buffered data
                SSP1BUF = temp;             // load data
                SSPCON1bits.CKP = 1;        // allow clock to continue
            }
            else // Nack received
                state = S_START;
            break;

        case S_WRITE:   // writing data to device
            s_count--;
            if ( s_count == 0)
            {
                // execute command
                switch (s_cmd)
                {
                    case 0:     // set reg pointer (bank, reg)
                        s_bank = temp;
                        s_reg = data;
                        break;
                    case 1:     // write *p
                        p = (unsigned char *)(s_reg + ((unsigned short)s_bank<<7));
                        *p++ = temp;
                        //*p   = data;
                        break;
                    default:
                        s_error = 1;        // undefined command
                        break;
                } // switch
                SSPCON1bits.CKP = 1;        // release SCL
                state = S_START;
            }
            else // s_count == 1
            {
                temp = data;                // buffer the first byte
                SSPCON1bits.CKP = 1;        // release SCL
            }
            break;

        case S_START:   // default state, waiting for an address/command match
        default:
            SSPCON1bits.SSPOV = 0;              // clear overflow conditions

            // expecting an address
            if ( SSPSTATbits.D_nA)     break;  // error, remain in start

            s_cmd = (data & I2C_CMD_MASK)>>1;  // save the command for later 
            
            // separate read from write
            if ( (data & 1) == 0)
            {
                state = S_WRITE;
                // set data counter (all commands have 2 bytes data)
                s_count = 2;
                SSPCON1bits.CKP = 1;        // release SCL
            } // -> write
            else 
            {
                state = S_READ;
                // execute command to prepare data
                switch ( s_cmd)
                {
                    case 0: // read reg pointer (bank, reg)
                        SSP1BUF = s_bank;
                        temp = s_reg;      //
                        break;
                    case 1: // read *p
                        p = (unsigned char *)(s_reg + ((unsigned short)s_bank<<7));
                        SSP1BUF = *p++;
                        temp = *p;
                        break;
                    case 7:                 // read the error flag (and clear it)
                        SSP1BUF = s_error;
                        s_error = 0;
                        break;
                    default:
                        SSP1BUF = 0xFD;     // return something anyway
                        s_error = 1;        // undefined command
                        break;
                }  // switch
                SSPCON1bits.CKP = 1;        // allow clock to continue

            } // -> read
            break;

    } // switch
} // isr



/************************************************************************
 * Initialize I/Os
 ************************************************************************/
void initIO(void)
{

  //Port initialization
  LATA      = 0b00000000;   // (all active low).
  LATB      = 0b00000000;   //
  LATC      = 0b00000000;   //

  TRISA     = 0b00100100;   // A2 ana, A7-LED, A5 in, A0 A1 A3 A4 A6 are NC (output low to reduce power/noise)
  TRISB     = 0b00011111;   // B0-INT in, B1 B2 B3 B4 analog, B5 B6 B7 are NC (output low)
  TRISC     = 0b10111100;   // C[0,1] PSMC[A,B] outputs, C2 input ; C3-SCL, C4-SDA, C5-SDI, C7-RX input, C6-TX out

  ANSELA    = 0b00000100;   // A2 analog inputs
  ANSELB    = 0b00011110;   // B0-INT dig in, B1 B2 B3 B4 analog inputs

  WPUA      = 0b00100000;   // Pullup A5-CS
  WPUB      = 0b00000000;   // none
  WPUC      = 0b10100000;   // Pullup C5-SDI, C7-RX

} // initIO


/************************************************************************
*      Function:       void initModules(void)                           
*                                                                                                                                              *
*      Note:                                                            
*           All peripherals are initialized, interrupts enabled         
***********************************************************************/
void initModules(void)
{

  // Initialize Internal Oscillator
  OSCCON = 0x6A;                 // 4MHz IntOsc

  // Interrupts
  INTCON = 0;			 // Start with all interrupts disabled

  // INT pin initialization
  INTF = 0;
  INTE = 0;			//NO INT/RB0 pin interrupts

  // OpAmp Initialization (v 1.01 uses OPA2)
  OPA2CON = 0b11000010;         // OPA2 Enabled, High Gain, OPA2IN+ <= DAC
  OPA1CON = 0b00000000;         // OPA1 OFF, not needed in this example

  // Comparator Initialization (v 1.01 uses C3)
  CM3CON0 = 0b10001110;         // Enabled, Pol Normal, Hyst, Async,
  //  CM3CON1 = 0b00001011;
  CM3CON1bits.C3PCH =   1;      // Positive IN1+(RB4, IFB),
  CM3CON1bits.C3NCH =   3;      // Negative IN3-(RB1, OPA2OUT)

  // FVR initialization
  FVRCON  = 0b10001000;  	// FVR enabled, DAC VR=2.048V

  // DAC Initialization
  DACCON0 = 0b10101000;         // DAC enabled, DACout1 on, Source: FVR and Vss
  DACCON1 = 154;                // 256/Vdac * 1.5V

  // PWM Initialization // not used! Slope Compensation performed with PSMC
//  PR2     = 255;                  // set perio
//  T2CON   = 0b00000100;         // turn on T2
//  CCPR1L  = 128;                  // set duty
//  CCP1CON = 0b00001100;         // PWM mode

  // ADC Initialization, sensing Vout * R12/(R12+R15) ~Vout/3
  ADCON0 = 0b00100001;          // AN8=RB2 (OPA2IN-), ADON
  ADCON1 = 0b01110000;          // Vdd and Vss as references, FRC clock, left align
  ADCON2 = 0b00001111;          // negative input selected by ADNREF(above)

  // I2C Initialization
  SSPMSK = I2C_ADD_MASK;        // set the mask bits
  SSPADD = I2C_DEV_ADD;         // set the device address
  SSPCON1 = 0b00100110;         // I2C enabled, 7-bit SLAVE
  SSPCON2bits.SEN = 1;          // enable strech on receive
  //SSPCON3bits.SDAHT = 1;        // extend SDA hold to 300ns
  SSP1IF = 0;                   // clear interrupt flag
  SSP1IE = 1;                   // enable I2C interrupts
  
  // Interrupt Enabling
  PEIE    = 1;			// Peripheral Interrupts Enabled
  GIE     = 1;          	// Global interrupt Enabled

} // initModules


/************************************************************************
*                                                                       *
*      Function:       void initPSMC(void)                              *
*                                                                       *
***********************************************************************/
void initPSMC(void)
{
// ********* PSMC MODE CONFIGURATION **************

// General
  	PSMC1MDL = 0b00000000;	// No Modulation of PSMC Output
  	PSMC1SYNC= 0b00000000;	// No Sync to other PSMCs

  	PSMC1CLK = 0b00000001;	// Based on Fosc 64Mhz, so 20nS resolution

// Output Pin Enables & Polarity // Slope compensation is controlled by RC2, PSMC1C or CCP1
	PSMC1OEN = 0b00000011;	// Enable Outputs A(H) and B(L) at startup, no slope compensation
  	PSMC1POL = 0b00000000;	// PSMC1A/B is active high.

// Blanking  Blanking of CxOUT (C3 is used in v1.01)
  	PSMC1BLNK = 0b00000001;	// Immediate Blanking of Rising Edge enabled
  	PSMC1REBS = 0b00001000;	// Rising Edge Blanking of C3OUT
  	PSMC1FEBS = 0b00000000;	// Falling Edge Blanking disabled

// Output Edge Trigger Configuration (Period, Rising and Falling Edges)

// RISING EDGE
  	PSMC1PHS = 0b00000001;	// Rising edge controlled by PSMC1PH Timer Match

// FALLING EDGE - End of Sync. Time v1.01 uses C3
	PSMC1DCS = 0b00001001;	// Falling edge controlled by Timer Match OR C3OUT
//	PSMC1DCS = 0b00000001;	// Falling edge controlled by Timer Match

// PERIOD CONTROL
 	PSMC1PRS = 0b00000001;	// Period event controlled by PSMC1PR Timer Match (Period)

//Auto Shutdown Enable, States, and Source. Not used for this lab
  	PSMC1ASDC = 0b00000000;	// Auto-Shutdown control register
  	PSMC1ASDL = 0b00000000;	// In Auto-Shutdown, PSMC1A will go high, PSMC1B will go low
  	PSMC1ASDS = 0b00000000;	// Auto-Shutdown source register (none set)

// Output Timer Triggers (these are based on PSMC1TMR based on above configuration)
	PSMC1PH = 1;            // Rising Event timer value compared to the primary TMR  (Cycle Start Delay)

// Set the Duty Cycle
	PSMC1DC = 128;          // Falling Event timer value compared to the primary TMR (Duty Cycle)
                                // maximum 80%
//	PSMC1DC = 50;          // Falling Event timer value compared to the primary TMR (Duty Cycle)

// Set the period timer to force the restart of the period
// If running at 64MHz every tic is 15.625ns We want a period of 400KHz

	PSMC1PR = 159;          // Period Event timer value compared to the primary TMR  (Period)

// Dead-band Rising
	PSMC1DBR = 3;           // 48ns

//  Dead-band Falling
	PSMC1DBF = 3;           // 48ns

// Fractional Frequency Adjust
	PSMC1FFA  = 0b00000000;	// No FFA set

// Add blanking of pin inputs increments of PSMC_CLK
	PSMC1BLKR = 32;         // 160ns rising edge blanking
	PSMC1BLKF = 0;          // No blanking set

// Output Steering
	PSMC1STR0 = 0b00000011;	// Primary output on PSMC1A/B

// PWM Sync and Modulation
	PSMC1STR1 = 0b00000000;	// No PWM synchronization or modulation

// Interrupt Selection
	PSMC1INT = 0b00000000;  // No PSMC Interrupts

    	PSMC1CON = 0b10110001;  // Load, Module Enabled, Deadband En, Single PWM Compl. Mode

} // initPSMC


/*
 * main
 */
void main(void)
{
    int time;

    initIO();            // Initialize Device
    initModules();       // Setup Comparators, OPAMP, DAC and ADC
    initPSMC();          // Setup PSMC module

    // Once the PSMC and feedback is setup up the boost circuit will
    // run on its own

    while(1)
    {
        if ( time++ >= 10000)           // buck click heart beat
        {
            time = 0;
            P_LED = 1- P_LED;           // toggle the LED
        }

        // perform a conversion 
        GO_nDONE =1;                    // Start reading of ADC
        while(GO_nDONE);
    }
} // main
