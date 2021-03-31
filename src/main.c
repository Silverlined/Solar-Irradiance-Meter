/*******************************************
 Solar Irradiance Meter Assignment 6
 Dimitriy Georgiev, Stoyan Bonev, Richard Kaiva
 Embedded Systems Semester 5
 Date: 11 January 2021
********************************************/

#define __18F25K80
#include "xc.h"
#include "fuses.h"
#include <stdio.h>
#include <stdbool.h>

#define _XTAL_FREQ 8000000 // X-tal = 8 MHz

#define button1 PORTBbits.RB0
#define button2 PORTBbits.RB1
#define LED LATCbits.LATC7
volatile bool transmitFlag = 0;

int _Analog_Digital_convertor(int channel)
{
    ANCON0bits.ANSEL1 = 1; // Configure AN1 as analogue pin
    if (channel == 1)
    {
        ADCON0 = 0b00000111; // Channel AN1, Start ADC conversion, ADC on
    }
    else if (channel == 9)
    {
        ADCON0 = 0b00100111; // Channel AN9, Start ADC conversion, ADC on
    }
    ADCON1 = 0b00110000; // Trigger ECCP1, AVdd, AVss, Channel 00 (AVss)
    ADCON2 = 0b10111000; // Right justified, Tad = 0, conversion clock Fosc/2
    while (ADCON0bits.GO == 1)
        ;                        // Wait until conversion is ready
    return ADRESH << 8 | ADRESL; // Read ADC value
}

void putch(char glyph)
{
    while (TXSTA1bits.TRMT == 0)
        ;           //Wait while the Trasmit Shift Register is full.
    TXREG1 = glyph; // When the TSR register is empty, load the new data.
}

void interrupt myIsr(void)
{
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF) // Counter overflow
    {
        transmitFlag = 1;
        INTCONbits.TMR0IF = 0;            // clear this interrupt condition
        LATCbits.LATC4 = ~LATCbits.LATC4; // Flip state of LED2
        TMR0H = 0x67;                     // Counter offset HIGH byte
        TMR0L = 0x69;                     // Counter offset LOW byte
        return;
    }

    if (INTCONbits.INT0IE && INTCONbits.INT0F) // Button 1 pressed
    {
        LED = 0;
        INTCONbits.TMR0IE = 0; // timer0 = disabled
        INTCONbits.INT0F = 0;  // button 1 flag = false
        return;
    }

    if (INTCON3bits.INT1E && INTCON3bits.INT1F) // Button 2 pressed
    {
        LED = 1;
        INTCONbits.TMR0IE = 1; //timer0 = enabled
        INTCON3bits.INT1F = 0; // button2 flag = false
        return;
    }
}

void initRS232(void)
{
    // USART1
    TRISCbits.TRISC6 = 0;  // Set TX1 output
    PIE3bits.RC2IE = 0;    // Disable RX interrupt USART2
    PIE3bits.TX2IE = 0;    // Disable TX interrupt USART2
    PIE1bits.RC1IE = 0;    // Disable RX interrupt USART1
    PIE3bits.TXB1IE = 0;   // Disable TX interrupt USART1
    TXSTA1 = 0b10100000;   // Clock = BRG, 8-bits, TX enabled, async, sync break, low speed, TSR full
    RCSTA1 = 0b10000000;   // Port enabled, 8-bit reception, disable receive, async, disable address, No FRA
    BAUDCON1 = 0b11000000; // BRG rollover, RX1 idle, RX1 not inverted, idle state low, 8-bit baud, RX1 not monitored, Baud rate disabled
    SPBRG1 = 51;           // (decimal) 51, 2400 b/s baud rate
}

void init(void)
{

    TRISC = 0x00; // Port C output
    LATC = 0x00;  // All LEDs off

    TRISBbits.TRISB0 = 1; // Button 1 pin as input
    TRISBbits.TRISB1 = 1; // Button 2 pin as input

    ANCON1bits.ANSEL10 = 0; // INT0 pin as digital pin
    ANCON1bits.ANSEL8 = 0;  // INT1 pin as digital pin

    T0CON = 0b10000111; // Enable the timer as 16 bit, internal clock, prescaler 1:256

    INTCON2bits.TMR0IP = 1;  // set high priority
    INTCONbits.TMR0IF = 0;   // int timer flag = false
    INTCONbits.TMR0IE = 1;   // int timer 0 = enabled
    INTCON2bits.INTEDG0 = 0; // set falling edge detection

    INTCONbits.INT0E = 1;    // int button 1 = enabled
    INTCON3bits.INT1E = 1;   // int button 2 = enabled
    INTCON2bits.INTEDG1 = 1; // set rising edge detection

    ei(); // enable all interrupts
}

void main(void)
{
    const unsigned int SHUNT_RESISTANCE = 121;
    const float PV_AREA = 0.0396;        // PV Panel Area
    const float V_REF = 4.1;             // ADC Reference Voltage
    const unsigned int ADC_COUNT = 4096; // 2^12; 12-bit ADC

    initRS232();
    init();

    while (1)
    {
        if (transmitFlag == 1)
        {
            unsigned int pv_value = _Analog_Digital_convertor(1);
            unsigned int ntc_value = _Analog_Digital_convertor(9);
            float pv_voltage = pv_value * V_REF / ADC_COUNT;
            float pv_power = pv_voltage * pv_voltage / SHUNT_RESISTANCE; // Power = V^2/R
            float temp = -0.028 * ntc_value + 95.1;                      // Linear approximation NTC output
            float pv_power_corrected = pv_power - ((temp - 25.0) * 0.4 * pv_power) / 100;
            float solar_irradiance = pv_power_corrected / PV_AREA;
            printf("%.3f,", solar_irradiance);
            printf("%.2f\n\r", temp);
            transmitFlag = 0;
        }
    }
}
