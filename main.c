/*
 * File:   main.c
 * Author: Patrick Damery.
 * Property of: Cabalry Limited.
 *
 * Bluetooth Panic Button Code.
 * 
 * Created on 24 October 2015, 16:14
 */
#include <xc.h>
#include <stdlib.h>

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// Define Oscillator Frecuency to use in delay_ms calculations.
#define _XTAL_FREQ   4000000  // 4MHz

// Function that initializes ADC parameters.
void initADC(void) {
    
    // Enable Fixed Voltage Reference.
    FVRCONbits.FVREN = 1;
    
    // Set Fixed Voltage Reference to 2.048V.
    FVRCONbits.ADFVR = 0b10;
    
    // Define Conversion Clock to use FOSC/4.
    ADCON1bits.ADCS = 0b100;
    
    // Configure ADC Positive Reference Voltage to use VDD.
    ADCON1bits.ADPREF = 0b00;
    
    // Configure ADC Negative Reference Voltage to use VSS.
    ADCON1bits.ADNREF = 0;
    
    // Output format as right justified.
    ADCON1bits.ADFM = 1;
    
    // Select Internal Voltage Reference as Channel.
    ADCON0bits.CHS = 0b11111;
    
    // Enable ADC Interrupt.
    PIE1bits.ADIE = 1;
    
    // Start ADC Module.
    ADCON0bits.ADON = 1;
    
    // Start conversion.
    ADCON0bits.GO_nDONE = 1;
}

// Function that initializes UART.
void initUART(void) {
    
    // Define pins.
    TRISB2 = 1;                                   // TX Pin
    TRISB1 = 1;                                   // RX Pin
        
    // Define UART setup.
    BRG16 = 0;                                    // Run with 8-bit timer
    BRGH = 1;                                     // Set High Baud Rate
    SPBRG = 25;                                    // Writing SPBRG Register
    SYNC = 0;                                     // Setting Asynchronous Mode, ie UART
    SPEN = 1;                                     // Enables Serial Port
    //CREN = 1;                                     // Enables Continuous Reception
    //TXIE  = 1;                                    // Disable tx interrupts
    //RCIE  = 1;                                    // Enable rx interrupts
    TX9   = 0;                                    // 8-bit transmission
    RX9   = 0;                                    // 8-bit reception
    TXEN = 1;                                     // Enables Transmission
}

// Function that initializes button interrupt parameters.
void initInterrupts(void) {
    
    // Set RB0 as input.
    TRISB0 = 1;
    
    // Enable Global Interrupts.
    INTCONbits.GIE = 1;
    
    // Enable External Interrupts.
    INTCONbits.INTE = 1;
    
    // Enable Peripheral Interrupts.
    INTCONbits.PEIE = 1;
}

// Function that initializes Watch Dog Timer parameters.
void initWatchDog(void) {
    
    // Set timer to 256 seconds.
    WDTCONbits.WDTPS = 0b10010;
    //WDTCONbits.WDTPS = 0b01110;
}

// Function that sends characters through serial port.
void sendData(char data) {
   while(!TRMT);  // wait for previous transmission to finish
   TXREG = data;
}

// Function that sends strings through serial port.
void sendString(char* text) {
    int i;
    for(i = 0; text[i] != '\0'; i++) {
        sendData(text[i]);
    }
}

void interrupt catchInterrupts(void) {
    
    // Check what flag set interrupt and reset it.
    if(INTF) {
        
        // Reset interrupt flag.
        INTF = 0;
    } else if(ADIF) {
        
        // Reset ADC interrupt flag.
        ADIF = 0;
    }
    
    // Check if panic button is pressed.
    int alarm = 0;
    if(RB0) {
        alarm = 1;
    }
    
    // Wait until ADC conversion is complete.
    while(ADCON0bits.GO_nDONE);
    
    // Read ADC Result Register High and 
    // Register Low for full 10 bit scope.
    int adcResult = ((ADRESH)<<8)|(ADRESL);
        
    // Calculate current Voltage.
    // Vref*1023=2095104
    int currentVoltage = 2095104/adcResult;

    // Max Voltage 4.1v, minimum operating voltage 3.1v.
    int batteryPercent = (currentVoltage-3100)/10;
         
    // Make sure we don't overflow.
    if(batteryPercent > 100) {
        batteryPercent = 100;
    } else if(batteryPercent < 0) {
        batteryPercent = 0;
    }
            
    char percentToChar[5];
    itoa(percentToChar, batteryPercent, 10);
           
    // Send identification string for Cabalry button and state.
    if(alarm) {
        sendString("cabA");
    } else {
        sendString("cabS");
    }
    
    // Send battery percentage.
    sendString(percentToChar);
    sendString("\n");
}

// Main function.
void main(void) {
        
    // Setup Internal Oscillator to run at 4MHz.
    OSCCONbits.IRCF = 0b1101;
    
    // Initialize UART.
    initUART();
    
    // Initialize ADC.
    initADC();
    
    // Initialize Interrupts.
    initInterrupts();
    
    // Set RB0 as panic button input.
    TRISB0 = 1;
    
    // Initialize Watchdog timer.
    initWatchDog();
    
    // Put device to sleep.
    SLEEP();
}