/*
 * Project: Buck-Click
 * File:    main.c
 * Author:  Lucio Di Jasio
 * Compiled using: XC8 v. 1.30
 * Created on Dec 10, 2013
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
               BOREN_OFF & CLKOUTEN_ON & IESO_OFF & FCMEN_OFF);

__CONFIG(WRT_OFF & PLLEN_OFF & STVREN_ON & BORV_LO & LPBOR_OFF & LVP_OFF);

#define I2C_DEV_ADD  0x10    // top nibble only, unique I2C address
#define I2C_ADD_MASK 0xF0    // only address bits
#define I2C_CMD_MASK 0X0E    // only command bits
#define I2C_RW_MASK  0x01    // only direction bit

#define S_START     0       // default state
#define S_READ      1       // reading data from device
#define S_WRITE     2       // writing data to device

#define P_INT   LATB0       // MikroBus INT signal


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
 * Initialize micro I/O
 ************************************************************************/
void initIO(void)
{

  //Port initialization
  LATA      = 0b00000000;   // (all active low).
  LATB      = 0b00000000;
  LATC      = 0b00000000;   //

  TRISA     = 0b00100111;   // A3 A4 A6 A7 NC ( output low)
  TRISB     = 0b00000010;   // B0 out B1 in digital; B2 B4 B5 B6 B7 NC (output low)
  TRISC     = 0b11111100;   // PSMC A, B, PWM outputs; SDA, SCL, SDI, TX, RX input


  ANSELA    = 0b00100111;   // A0, A1, A2, A5 analog inputs
  ANSELB    = 0b00000010;   // B1 Analog input for POT, rest is all digital


  WPUA      = 0b00000000;   // No pullups
  WPUB      = 0b00000000;   // No pullups
  WPUC      = 0b11100000;   // TX, RX, SDI pull up

} // initIO


/************************************************************************
*                                                                       *
*      Function:       void initModules(void)                           *
*                                                                       *
*      Description:                                                     *
*                                                                       *
*      Parameters:      none                                            *
*      Return value:    none                                            *
*                                                                       *
*      Note:                                                            *
*           All peripherals are initialized, interrupts enabled         *
*                                                                       *
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


  // Opamp1 Initialization
  OPA1CON = 0b11000010;         // Opamp Enabled, High Gain, OPA1IN+ <= DAC

  //Opamp2 initialization
  OPA2CON = 0b00000000;         // OPA2 OFF, not needed in this lab

  //Comparator Initialization
  CM1CON0 = 0b10001110;         // Enabled, Pol Normal, Hyst, Async,
  CM1CON1 = 0b00000001;         // Positive IN0+(RA2, IFA), negative IN0-(RA0, OPAOUT)

  //FVR initialization
  FVRCON  = 0b10001000;  	// FVR enabled, DAC VR=2.048V

  //DAC Initialization
  DACCON0 = 0b10001000;         // DAC enabled, DACout1/2 off, Source: FVR and Vss
  DACCON1 = 255;                // 256/Vdac * 1.5V

  //ADC Initialization
//  ADCON0 = 0b00101001;          // 10-bit, AN10=RB1(CS), ADON
  ADCON0 = 0b00010001;          // AN4=RA5(OPAMPIN), ADON
  ADCON1 = 0b01110000;          // Vdd and Vss as references, FRC clock, left align
  ADCON2 = 0b00001111;          // negative input set to ref-

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

// Output Pin Enables & Polarity
	PSMC1OEN = 0b00000011;	// Enable Outputs A(H) and B(L) at startup
  	PSMC1POL = 0b00000000;	// PSMC1A/B is active high.

// Blanking  No blanking for this lab.
  	PSMC1BLNK = 0b00000001;	// Immediate Blanking of Rising Edge enabled
  	PSMC1REBS = 0b00000010;	// Rising Edge Blanking of C1OUT
  	PSMC1FEBS = 0b00000000;	// Falling Edge Blanking disabled


// Output Edge Trigger Configuration (Period, Rising and Falling Edges)

// RISING EDGE
  	PSMC1PHS = 0b00000001;	// Rising edge controlled by PSMC1PH Timer Match

// FALLING EDGE - End of Sync. Time
//	PSMC1DCS = 0b00000001;	// Falling edge controlled by Timer Match
        PSMC1DCS = 0b00000011;  // adding OR C1OUT

// PERIOD CONTROL
 	PSMC1PRS = 0b00000001;	// Period event controlled by PSMC1PR Timer Match (Period)

//Auto Shutdown Enable, States, and Source. Not used for this lab
  	PSMC1ASDC = 0b00000000;	// Auto-Shutdown control register
  	PSMC1ASDL = 0b00000000;	// In Auto-Shutdown, PSMC1A will go high, PSMC1B will go low
  	PSMC1ASDS = 0b00000000;	// Auto-Shutdown source register (none set)


// Output Timer Triggers (these are based on PSMC1TMR based on above configuration)
	PSMC1PH = 1;            // Rising Event timer value compared to the primary TMR  (Cycle Start Delay)

// Set the Duty Cycle
	PSMC1DC = 143;          // Falling Event timer value compared to the primary TMR (Duty Cycle)
                                // maximum 90%

// Set the period timer to force the restart of the period
// If running at 64MHz every tic is 15.625ns We want a period of 400KHz

	PSMC1PR = 159;          // Period Event timer value compared to the primary TMR  (Period)

// Dead-band Rising
	PSMC1DBR = 6;           // 100ns

//  Dead-band Falling
	PSMC1DBF = 6;           // 100ns

// Fractional Frequency Adjust
	PSMC1FFA  = 0b00000000;	// No FFA set

// Add blanking of pin inputs increments of PSMC_CLK
	PSMC1BLKR = 10;         // 160ns falling edge blanking
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
            P_INT = 1-P_INT;            // toggle the INT pin
        }

        // perform a conversion 
        GO_nDONE =1;                    // Start reading of ADC
        while(GO_nDONE);

    //DACCON1 = ((ADRESH>>2) + 7);       // Value of ADC placed into DAC
                                        // for Comparator input
    }
} // main
