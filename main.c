// PIC16F73 Configuration Bit Settings
// 'C' source line config statements

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define _XTAL_FREQ 8000000

#include <xc.h>
#include <stdio.h>  // For sprintf
#include <string.h> // For strlen, strcpy (maybe)
#include <math.h>   // For sin, sqrt (M_PI might require -lm in linker and C99 mode, or define PI manually)

// Define PI if not available in math.h for XC8 C90 standard
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Placeholder for I2C LCD functions
#include "i2c_lcd.h"

//----------------------------------------------------------------------
// Global Definitions and Constants
//----------------------------------------------------------------------

// Pin Assignments
#define H_BRIDGE_A_PIN  RB0 // Square wave A for H-Bridge
#define H_BRIDGE_A_TRIS TRISBbits.TRISB0
#define H_BRIDGE_B_PIN  RB1 // Square wave B for H-Bridge
#define H_BRIDGE_B_TRIS TRISBbits.TRISB1

#define SPWM_CCP1_PIN   RC5 // Hardware SPWM output 1 (CCP1)
#define SPWM_CCP1_TRIS  TRISCbits.TRISC5
#define SPWM_SW_PIN     RB2 // Software SPWM output 2
#define SPWM_SW_TRIS    TRISBbits.TRISB2

// I2C pins are handled in i2c_lcd.c (RC3/SCL, RC4/SDA)
// ADC Pins (already defined by hardware ANx)
// RA0/AN0: Battery Voltage
// RA1/AN1: AC Output Voltage
// RA2/AN2: Current Sense

// Fault Thresholds
#define LOW_BAT_VOLT            10.5f // Volts
#define OVER_BAT_VOLT           15.0f // Volts
#define OVERLOAD_WATTS          350.0f// Watts
#define SHORT_CIRCUIT_AMPS      20.0f // Amps

// System Parameters
#define TARGET_AC_VOLTS         22.0f // Target RMS AC output voltage
#define PWM_CARRIER_FREQ        20000 // Hz (SPWM carrier)
#define SINE_OUTPUT_FREQ        50    // Hz (Desired output sine wave)

// ADC & Calculation Parameters
#define ADC_VREF                5.0f  // ADC Reference Voltage
#define ADC_RESOLUTION          1023.0f // For 10-bit ADC (0-1023)
// Scaling Ratios
// Actual_Value = ( (ADC_read_value / ADC_RESOLUTION) * ADC_VREF ) / Ratio_Inverse
// Example: Battery Voltage: Ratio 1:4. Actual_Bat_V = ( (ADC_val/1023.0)*5.0 ) / (1.0/4.0)
//                                                 = ( (ADC_val/1023.0)*5.0 ) * 4.0
#define BAT_VOLT_RATIO_INV      (1.0f/4.0f)
#define AC_OUT_VOLT_RATIO_INV   (1.0f/100.0f)

// Current Sensor: CT sensor 1V=100A, then divided by 100.
// V_sensor_out = I_actual / 100.0
// V_at_adc_pin = V_sensor_out / 100.0 = (I_actual / 100.0) / 100.0 = I_actual / 10000.0
// So, I_actual = V_at_adc_pin * 10000.0
// I_actual = ( (ADC_val/1023.0)*VREF ) * 10000.0
// Ratio_Inverse for current = 1.0f / 10000.0f
#define CURRENT_SENSE_RATIO_INV (1.0f/10000.0f)

// Sine Table Configuration
// PWM_CARRIER_FREQ / (SINE_OUTPUT_FREQ * 2) gives steps for one half-cycle of the sine wave,
// if the ISR that increments the table index runs at SINE_OUTPUT_FREQ * 2.
// 20000 Hz / (50 Hz * 2) = 20000 / 100 = 200 steps for a half sine wave.
// The table will store values for a positive half cycle (0 to peak to 0).
#define SINE_TABLE_STEPS        200

//----------------------------------------------------------------------
// Global Variables
//----------------------------------------------------------------------

// ADC Readings and Calculated Values
volatile float battery_voltage_global;
volatile float ac_output_voltage_rms_global;
volatile float current_rms_global;
volatile float wattage_global;
volatile float ac_output_voltage_peak_raw; // Raw peak from ADC samples for feedback
volatile float current_peak_raw;           // Raw peak from ADC samples

// SPWM Control
volatile unsigned int sine_table[SINE_TABLE_STEPS];
volatile unsigned int sine_table_index = 0;
volatile float spwm_amplitude_scale = 1.0f; // For voltage feedback control
volatile unsigned int spwm_duty_ccp1 = 0;
volatile unsigned int spwm_duty_sw = 0; // For software PWM on RB2

// System Status
enum SystemState { STATE_RUNNING, STATE_FAULT };
volatile enum SystemState system_state = STATE_RUNNING;
volatile char fault_message[17]; // For 16x2 LCD

// H-Bridge state (for 50Hz generation)
volatile unsigned char h_bridge_phase = 0; // 0 for phase A, 1 for phase B

//----------------------------------------------------------------------
// Forward Declarations of Functions
//----------------------------------------------------------------------
void init_oscillator(void);
void init_ports(void);
void init_adc(void);
// init_i2c_lcd() is lcd_init() from i2c_lcd.h
void init_pwm_ccp1(void);
void init_timer0_h_bridge_sine(void);
void init_timer1_soft_pwm(void); // Or Timer2 if Timer1 is used for something else

void generate_sine_table(void);
void update_spwm_duties(void); // Removed index, will use global sine_table_index
void set_ccp1_duty(unsigned int duty);
// Software PWM for RB2 will be handled in its timer ISR

unsigned int read_adc_channel(unsigned char channel);
void process_adc_readings(void); // New function to handle peak detection etc.
// float get_battery_voltage(void); // Will be part of process_adc_readings
// float get_ac_output_voltage_peak(void); // Will be part of process_adc_readings
// float get_current_peak(void); // Will be part of process_adc_readings
// float calculate_rms_voltage(float peak_voltage); // Simple macro/inline
// float calculate_rms_current(float peak_current); // Simple macro/inline
// float calculate_wattage(float v_rms, float i_rms); // Simple macro/inline

void adjust_output_voltage_feedback(void);
void check_faults(void);
void shutdown_system(const char* message);

//----------------------------------------------------------------------
// Initialization Functions
//----------------------------------------------------------------------

void init_oscillator(void) {
    // Configuration bits FOSC = HS handles 8MHz external crystal.
    // No specific register setup needed here for this PIC with crystal.
}

void init_ports(void) {
    // Port A: Analog inputs
    // RA0/AN0, RA1/AN1, RA2/AN2 are analog. Set in ADCON1.
    TRISA = 0xFF; // All PORTA as inputs initially. AN0-AN2 will be analog.
    PORTA = 0x00;

    // Port B: H-Bridge (RB0, RB1), Software SPWM (RB2)
    H_BRIDGE_A_TRIS = 0; // RB0 output
    H_BRIDGE_B_TRIS = 0; // RB1 output
    SPWM_SW_TRIS = 0;    // RB2 output
    TRISB |= 0xF8;       // Ensure RB3-RB7 are inputs (0bxxxx1000 -> 0bxxxx1111 if |=)
                         // Better: TRISB = (TRISB & 0xF8) | (PORTB_INPUT_MASK & ~0x07);
                         // Simplest: TRISB = 0b11111000; (RB0,1,2 out, rest in)
    PORTB = 0x00;        // Initialize outputs low

    // Port C: I2C (RC3, RC4 by i2c_lcd.c), CCP1 SPWM (RC5)
    SPWM_CCP1_TRIS = 0;  // RC5 output for CCP1
    TRISC = 0b11010111;  // RC5 out. RC3(SCL), RC4(SDA) initially inputs (I2C lib will manage)
                         // Other pins (RC0,1,2,6,7) as inputs.
    PORTC = 0x00;
}

void init_adc(void) {
    // ADCON0:
    ADCON0bits.ADCS = 0b01; // FOSC/8 ADC clock for 8MHz FOSC -> 1MHz ADC Clock
    ADCON0bits.CHS = 0b000; // Select AN0 initially
    ADCON0bits.ADON = 1;    // Enable ADC

    // ADCON1:
    // PCFG<2:0> for PIC16F73:
    // 010 = AN0, AN1, AN2 are Analog. RA3(AN3), RA5(AN4) are Digital. VDD as Vref.
    ADCON1bits.PCFG = 0b010;
    ADCON1bits.ADFM = 1;     // Right justified result (ADRESH:ADRESL)

    __delay_us(50); // ADC acquisition time delay after setup and channel change
}

void init_pwm_ccp1(void) {
    // Timer2 for PWM frequency (20kHz)
    // PWM Period = (PR2 + 1) * 4 * TOSC * TMR2_Prescale
    // 50us = (PR2 + 1) * 4 * (1/8MHz) * TMR2_Prescale
    // 50us = (PR2 + 1) * 0.5us * TMR2_Prescale
    // 100 = (PR2 + 1) * TMR2_Prescale
    // Choose TMR2_Prescale = 1 (T2CKPS = 00)
    // PR2 + 1 = 100 => PR2 = 99 (0x63)
    PR2 = 99;
    T2CONbits.T2CKPS = 0b00; // Prescaler 1:1
    T2CONbits.TMR2ON = 1;    // Timer2 ON

    // CCP1 for PWM mode
    CCP1CONbits.CCP1M = 0b1100; // PWM mode for CCP1
    // CCP1 pin (RC5) already set as output in init_ports()

    // Initial duty cycle = 0
    CCPR1L = 0;              // Duty cycle MSBs
    CCP1CONbits.DC1B = 0;    // Duty cycle LSBs (DC1B1:DC1B0)
                             // Max 10-bit duty = (PR2+1)*4 = (99+1)*4 = 400
}

// Timer0 for 100Hz interrupt (H-bridge 50Hz toggle, Sine table step)
void init_timer0_h_bridge_sine(void) {
    // FOSC = 8MHz => Instruction Clock = FOSC/4 = 2MHz (0.5us period)
    // Desired Timer0 interrupt period = 10ms (100Hz)
    // TMR0_Period = (256 - TMR0_preload) * Prescaler * (1 / (FOSC/4))
    // 10000us = (256 - TMR0_preload) * Prescaler * 0.5us
    // 20000 = (256 - TMR0_preload) * Prescaler
    // Try Prescaler = 128 (PS<2:0> = 110):
    // 20000 / 128 = 156.25
    // (256 - TMR0_preload) = 156.25 => TMR0_preload = 256 - 156.25 = 99.75. Use 100.
    // Actual counts = 256 - 100 = 156.
    // Actual period = 156 * 128 * 0.5us = 9984us (approx 100.16 Hz)
    OPTION_REGbits.T0CS = 0;   // Timer0 Clock Source = Internal instruction cycle (FOSC/4)
    OPTION_REGbits.PSA = 0;    // Prescaler assigned to Timer0
    OPTION_REGbits.PS = 0b110; // Prescaler 1:128 for Timer0

    TMR0 = 100;                // Preload Timer0 (256 - 156 counts)
    INTCONbits.TMR0IE = 1;     // Enable Timer0 Overflow Interrupt
    INTCONbits.T0IF = 0;       // Clear Timer0 Overflow Interrupt Flag
}

// Timer1 for Software SPWM on RB2 (20kHz interrupt)
void init_timer1_soft_pwm(void) {
    // Desired Timer1 interrupt period = 1 / 20kHz = 50us
    // Timer1 Clock Source = FOSC/4 = 2MHz (0.5us period)
    // TMR1_Period = (65536 - TMR1_preload) * Prescaler_T1 * 0.5us
    // 50us = (65536 - TMR1_preload) * Prescaler_T1 * 0.5us
    // 100 = (65536 - TMR1_preload) * Prescaler_T1
    // Choose Prescaler_T1 = 1:1 (T1CKPS = 00)
    // (65536 - TMR1_preload) = 100 => TMR1_preload = 65536 - 100 = 65436 (0xFF9C)
    T1CONbits.TMR1CS = 0;      // Timer1 Clock Source = Internal (FOSC/4)
    T1CONbits.T1CKPS = 0b00;   // 1:1 Prescaler
    T1CONbits.T1OSCEN = 0;     // Dedicated Timer1 oscillator disabled

    TMR1H = (65436 >> 8);      // Preload TMR1H
    TMR1L = (65436 & 0xFF);    // Preload TMR1L

    PIR1bits.TMR1IF = 0;       // Clear Timer1 Overflow Interrupt Flag
    PIE1bits.TMR1IE = 1;       // Enable Timer1 Overflow Interrupt

    T1CONbits.TMR1ON = 1;      // Start Timer1
}

//----------------------------------------------------------------------
// SPWM Generation Functions
//----------------------------------------------------------------------

void generate_sine_table(void) {
    // Generates a lookup table for a half sine wave (0 to peak to 0).
    // SINE_TABLE_STEPS = 200 (for half wave)
    // Max PWM duty cycle value is (PR2 + 1) * 4 = (99 + 1) * 4 = 400.
    // Values in table should range from 0 to 400.
    unsigned int max_pwm_duty = (PR2 + 1) * 4;

    for (int i = 0; i < SINE_TABLE_STEPS; i++) {
        // Angle for half sine wave: 0 to PI radians (0 to 180 degrees)
        // Current step i covers angle = (i / (SINE_TABLE_STEPS -1)) * PI
        // However, it's easier to map i from 0 to SINE_TABLE_STEPS directly to 0 to PI.
        double angle = ((double)i / (double)SINE_TABLE_STEPS) * M_PI;
        double sin_value = sin(angle); // sin_value is 0 to 1
        sine_table[i] = (unsigned int)(sin_value * (max_pwm_duty -1)); // Scale to 0 -> (max_pwm_duty-1)
                                                                    // -1 to ensure it never exceeds full duty cycle exactly,
                                                                    // though with int conversion, it's fine.
    }
    // Example: SINE_TABLE_STEPS = 200.
    // i=0:   angle=0, sin(0)=0. sine_table[0]=0.
    // i=99:  angle=99/200*PI approx PI/2. sin(PI/2)=1. sine_table[99] approx max_pwm_duty. (This is midpoint of HALF wave)
    // i=100: angle=100/200*PI = PI/2. sin(PI/2)=1. sine_table[100] = max_pwm_duty.
    // i=199: angle=199/200*PI approx PI. sin(PI)=0. sine_table[199] approx 0.
    // This creates a table that goes 0 -> max -> 0 over SINE_TABLE_STEPS entries.
}

void update_spwm_duties(void) {
    // This function is called from Timer0 ISR (100Hz)
    // It uses the global sine_table_index to get the base sine value.
    // It applies the spwm_amplitude_scale for voltage feedback.

    if (sine_table_index >= SINE_TABLE_STEPS) {
        sine_table_index = SINE_TABLE_STEPS - 1; // Should not happen if logic is correct elsewhere
    }

    unsigned int base_duty = sine_table[sine_table_index];

    // Apply amplitude scaling (feedback control)
    // spwm_amplitude_scale can range e.g. 0.5 to 1.5 (or clamped more tightly)
    float scaled_duty_float = (float)base_duty * spwm_amplitude_scale;

    // Clamp duty cycle to prevent overflow/underflow and ensure it's within valid PWM range
    unsigned int max_pwm_duty = (PR2 + 1) * 4; // 400
    if (scaled_duty_float < 0) {
        scaled_duty_float = 0;
    }
    if (scaled_duty_float >= max_pwm_duty) {
        scaled_duty_float = max_pwm_duty -1; // Max value for 10-bit PWM with PR2=99 is 400. Duty can be 0-400.
                                             // (CCPR1L bits + DC1B bits)
    }

    unsigned int final_duty = (unsigned int)scaled_duty_float;

    // For unipolar SPWM, both SPWM outputs get the same |sin(wt)| modulation value.
    // The H-bridge control (RB0, RB1) determines which part of the bridge this applies to.
    spwm_duty_ccp1 = final_duty;
    spwm_duty_sw = final_duty; // Software PWM will use this value (after scaling for its own counter)
}

void set_ccp1_duty(unsigned int duty) {
    // Duty is a 10-bit value (0-400 for PR2=99)
    // CCPR1L holds the upper 8 bits
    // CCP1CON<5:4> (DC1B1:DC1B0) hold the lower 2 bits
    if (duty > (PR2+1)*4) duty = (PR2+1)*4; // Clamp to max possible duty

    CCPR1L = (duty >> 2);         // Upper 8 bits
    CCP1CON = (CCP1CON & 0xCF) | ((duty & 0x03) << 4); // Lower 2 bits (DC1B1:DC1B0 are bits 5:4 of CCP1CON)
                                                      // Mask to preserve CCP1M bits: 0xCF = 0b11001111
}

//----------------------------------------------------------------------
// ADC Reading and Processing Functions
//----------------------------------------------------------------------
#define SQRT_2 1.41421356f
#define AC_PEAK_SAMPLES 50 // Number of samples to take over approx half AC cycle for peak detection
                           // For 50Hz, half cycle is 10ms. 50 samples -> 1 sample every 200us.
                           // ADC conversion takes Tad * (8 or 12 bits + acq time). Tad = 1us for 1MHz ADC clock.
                           // So, ~15-20us per conversion. 200us interval is feasible.

unsigned int read_adc_channel(unsigned char channel) {
    ADCON0bits.CHS = channel & 0x07; // Select ADC channel (0-7)
    __delay_us(50); // Acquisition time delay for channel switch

    ADCON0bits.GO_nDONE = 1; // Start ADC conversion
    while (ADCON0bits.GO_nDONE) { // Wait for conversion to complete
        // Busy wait
    }
    // Result is Right Justified (ADFM=1), so 10-bit result in ADRESH:ADRESL
    return ((unsigned int)ADRESH << 8) | ADRESL;
}

// Processes all ADC readings, calculates derived values, updates globals
void process_adc_readings(void) {
    unsigned int adc_value;
    float voltage_at_adc_pin;

    // 1. Read Battery Voltage (AN0)
    adc_value = read_adc_channel(0); // Channel AN0
    // ADC_RESOLUTION is now 1023.0f
    voltage_at_adc_pin = ((float)adc_value / ADC_RESOLUTION) * ADC_VREF;
    battery_voltage_global = (voltage_at_adc_pin / BAT_VOLT_RATIO_INV);

    // 2. Read AC Output Voltage (AN1) - Peak Detection
    unsigned int max_adc_val_ac_v = 0;
    for (int i = 0; i < AC_PEAK_SAMPLES; i++) {
        adc_value = read_adc_channel(1); // Channel AN1
        if (adc_value > max_adc_val_ac_v) {
            max_adc_val_ac_v = adc_value;
        }
        __delay_us(150);
    }
    ac_output_voltage_peak_raw = ((float)max_adc_val_ac_v / ADC_RESOLUTION) * ADC_VREF; // Vpeak at ADC pin
    float ac_output_voltage_peak_actual = ac_output_voltage_peak_raw / AC_OUT_VOLT_RATIO_INV;
    ac_output_voltage_rms_global = ac_output_voltage_peak_actual / SQRT_2;


    // 3. Read Current (AN2) - Peak Detection
    unsigned int max_adc_val_current = 0;
    for (int i = 0; i < AC_PEAK_SAMPLES; i++) {
        adc_value = read_adc_channel(2); // Channel AN2
        if (adc_value > max_adc_val_current) {
            max_adc_val_current = adc_value;
        }
        __delay_us(150);
    }
    current_peak_raw = ((float)max_adc_val_current / ADC_RESOLUTION) * ADC_VREF; // Vpeak at ADC pin from current sensor
    // With CURRENT_SENSE_RATIO_INV = (1.0f/10000.0f)
    // Actual_Current_Peak = V_at_adc_pin_peak / (1.0/10000.0) = V_at_adc_pin_peak * 10000.0
    float actual_current_peak = current_peak_raw / CURRENT_SENSE_RATIO_INV;
    current_rms_global = actual_current_peak / SQRT_2;

    // 4. Calculate Wattage (assuming Power Factor = 1)
    wattage_global = ac_output_voltage_rms_global * current_rms_global;
    if (wattage_global < 0) wattage_global = 0; // Should not happen with RMS of magnitudes
}

//----------------------------------------------------------------------
// Feedback Control Loop Function
//----------------------------------------------------------------------
#define KP_VOLTAGE_FEEDBACK 0.05f // Proportional gain for voltage feedback. Tune this value.
                                 // If error is 1V, scale changes by 0.05.
#define SPWM_SCALE_MIN      0.2f  // Minimum amplitude scale for SPWM
#define SPWM_SCALE_MAX      1.8f  // Maximum amplitude scale for SPWM (max output might be limited by DC bus / sine table max)

void adjust_output_voltage_feedback(void) {
    if (system_state != STATE_RUNNING) {
        return; // Don't adjust if system is not running
    }

    float error_v = TARGET_AC_VOLTS - ac_output_voltage_rms_global;

    // Proportional control
    float adjustment = error_v * KP_VOLTAGE_FEEDBACK;
    spwm_amplitude_scale += adjustment;

    // Clamp the amplitude scale to prevent instability or extreme values
    if (spwm_amplitude_scale < SPWM_SCALE_MIN) {
        spwm_amplitude_scale = SPWM_SCALE_MIN;
    }
    if (spwm_amplitude_scale > SPWM_SCALE_MAX) {
        spwm_amplitude_scale = SPWM_SCALE_MAX;
    }

    // For debugging, one might print spwm_amplitude_scale or error_v to LCD if space/time permits
    // but this function is called frequently.
}

//----------------------------------------------------------------------
// Fault Detection and Handling Functions
//----------------------------------------------------------------------

void shutdown_system(const char* message) {
    system_state = STATE_FAULT; // Set global system state to FAULT

    // 1. Disable CCP1 PWM (Hardware PWM)
    CCP1CONbits.CCP1M = 0; // Turn off CCP1 module (clears PWM mode)
    SPWM_CCP1_TRIS = 1;    // Set CCP1 pin (RC5) to input to be safe, or ensure it's low.
                           // Setting TRIS to input is safer if external pull-downs aren't guaranteed.
                           // Or PORTCbits.RC5 = 0; if TRISCbits.TRISC5 = 0;
    PORTCbits.RC5 = 0;     // Ensure output is low if still configured as output.

    // 2. Disable Software PWM on RB2
    //    The Timer1 ISR checks system_state. If FAULT, SPWM_SW_PIN will be set low.
    //    We can also explicitly set it low here.
    SPWM_SW_PIN = 0;
    // Optionally disable Timer1 interrupt if it serves no other purpose in fault state
    // PIE1bits.TMR1IE = 0; // Disable Timer1 interrupt

    // 3. Disable H-Bridge Control Pins
    H_BRIDGE_A_PIN = 0;
    H_BRIDGE_B_PIN = 0;
    // Optionally disable Timer0 interrupt if SPWM duties don't need update and H-bridge is off
    // INTCONbits.TMR0IE = 0;

    // 4. Display Fault Message on LCD
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_print_string("SYSTEM FAULT:");
    lcd_set_cursor(1,0);
    // Copy message to global fault_message and ensure it's null-terminated for LCD
    strncpy(fault_message, message, 15); // Max 15 chars for second line (16th is null)
    fault_message[15] = '\0'; // Ensure null termination
    lcd_print_string(fault_message);

    // 5. Halt System (Infinite Loop)
    INTCONbits.GIE = 0; // Disable all interrupts to ensure halt
    while(1) {
        // System halted. Requires manual reset.
        // Could blink an LED or perform other minimal fault indication if desired.
    }
}

void check_faults(void) {
    if (system_state == STATE_FAULT) {
        return; // Already in fault state
    }

    // Check for Low Battery Voltage
    if (battery_voltage_global < LOW_BAT_VOLT) {
        shutdown_system("LOW BATTERY");
        return; // shutdown_system will not return
    }

    // Check for Over Battery Voltage
    if (battery_voltage_global > OVER_BAT_VOLT) {
        shutdown_system("OVER BATTERY");
        return;
    }

    // Check for Overload (Wattage)
    // Ensure wattage is valid (e.g. not negative if calculation could somehow allow it)
    if (wattage_global > OVERLOAD_WATTS && wattage_global > 0) {
        shutdown_system("OVERLOAD");
        return;
    }

    // Check for Short Circuit (Current)
    if (current_rms_global > SHORT_CIRCUIT_AMPS) {
        shutdown_system("SHORT CIRCUIT");
        return;
    }
}

//----------------------------------------------------------------------

void main(void) {
    init_oscillator();
    init_ports();
    init_adc();
    lcd_init(); // From i2c_lcd.c (includes I2C init)
    init_pwm_ccp1();
    init_timer0_h_bridge_sine();
    init_timer1_soft_pwm();

    generate_sine_table(); // Populate the sine lookup table

    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_print_string("Inverter Booting");
    __delay_ms(500);
    lcd_set_cursor(1,0);
    lcd_print_string("System OK");
    __delay_ms(1000);
    lcd_clear();

    // Enable Interrupts
    INTCONbits.PEIE = 1; // Enable Peripheral Interrupts (for Timer1, ADC if used)
    INTCONbits.GIE = 1;  // Enable Global Interrupts (must be last)

    system_state = STATE_RUNNING;

    while(1) {
        if (system_state == STATE_RUNNING) {
            process_adc_readings();
            check_faults();

            if (system_state == STATE_RUNNING) { // Re-check, fault might have occurred
                adjust_output_voltage_feedback();
            }

            // LCD display updates (periodically)
            // process_adc_readings() takes roughly 25ms due to sampling delays.
            // For an LCD update around every 500ms: 500ms / 25ms = 20 iterations.
            static unsigned int lcd_update_count = 0;
            if(++lcd_update_count > 20) {
                lcd_update_count = 0;
                if (system_state == STATE_RUNNING) { // Final check before display
                    char buffer[17];
                    sprintf(buffer, "B:%.1fV AC:%.1fV", battery_voltage_global, ac_output_voltage_rms_global);
                    lcd_set_cursor(0,0);
                    lcd_print_string(buffer);

                    sprintf(buffer, "P:%.0fW I:%.1fA", wattage_global, current_rms_global);
                    lcd_set_cursor(1,0);
                    lcd_print_string(buffer);
                }
            }
        }
        // If system_state is STATE_FAULT, shutdown_system() would have been called
        // and should ideally loop infinitely or handle reset.
        // The main loop effectively stops meaningful operations if a fault occurs.
        // A __delay_ms(1) can be added here if main loop is too tight and causing issues,
        // but generally, interrupt-driven systems don't need it if tasks are managed well.
    }
    return; // Should not be reached
}

void __interrupt() isr(void) {
    // Timer0 Interrupt (100Hz - H-Bridge and Sine Table Index)
    if (INTCONbits.T0IF && INTCONbits.T0IE) {
        INTCONbits.T0IF = 0; // Clear Timer0 interrupt flag
        TMR0 = 100;          // Reload Timer0 preset (approx for 10ms)

        if (system_state == STATE_RUNNING) {
            // H-Bridge Control (50Hz)
            // h_bridge_phase: 0 for 0-10ms, 1 for 10-20ms of a 50Hz cycle
            if (h_bridge_phase == 0) {
                H_BRIDGE_A_PIN = 1; // Drive for positive half-cycle
                H_BRIDGE_B_PIN = 0;
            } else {
                H_BRIDGE_A_PIN = 0; // Drive for negative half-cycle
                H_BRIDGE_B_PIN = 1;
            }
            // This ISR is 100Hz. h_bridge_phase toggles each time.
            // So H_BRIDGE_A/B pins toggle at 50Hz effectively.

            // SPWM Duty Update
            // sine_table has SINE_TABLE_STEPS (200) entries for a half sine wave (0-max-0)
            // Unipolar SPWM: Both SPWM outputs (CCP1, RB2) get the same |sin(wt)| modulation.
            // The H-bridge (RB0, RB1) applies this modulation to alternate output polarities.
            update_spwm_duties(); // Uses global sine_table_index to get value from sine_table
                                  // and applies spwm_amplitude_scale. Stores results in
                                  // spwm_duty_ccp1 and spwm_duty_sw.

            set_ccp1_duty(spwm_duty_ccp1); // Update hardware PWM channel

            // Advance sine table index for next SPWM update
            sine_table_index++;
            if (sine_table_index >= SINE_TABLE_STEPS) {
                sine_table_index = 0; // Reset for next half sine cycle
                h_bridge_phase = !h_bridge_phase; // Toggle phase at the end of each half sine wave
                                                  // This ensures H-Bridge pins are stable during one full 0-max-0 sine modulation.
            }
        }
    }

    // Timer1 Interrupt (20kHz - Software PWM for RB2)
    if (PIR1bits.TMR1IF && PIE1bits.TMR1IE) {
        PIR1bits.TMR1IF = 0;     // Clear Timer1 interrupt flag
        TMR1H = (65436 >> 8);    // Reload Timer1 preset for 50us (0xFF9C)
        TMR1L = (65436 & 0xFF);

        if (system_state == STATE_RUNNING) {
            // Software PWM generation for RB2
            // spwm_duty_sw holds the target duty cycle (0-400, like CCP1)
            // This ISR is the "tick" for the software PWM. Counter goes 0-99 (like PR2 for Timer2)
            // So, spwm_duty_sw needs to be scaled: scaled_duty = spwm_duty_sw / 4.
            static unsigned int soft_pwm_counter_t1 = 0; // Renamed to avoid conflict if any
            unsigned int current_sw_duty_scaled_t1 = spwm_duty_sw / 4; // Scale 0-400 to 0-100

            if (soft_pwm_counter_t1 < current_sw_duty_scaled_t1) {
                SPWM_SW_PIN = 1;
            } else {
                SPWM_SW_PIN = 0;
            }

            soft_pwm_counter_t1++;
            if (soft_pwm_counter_t1 >= 100) { // Period matches (PR2+1) = 100 ticks
                soft_pwm_counter_t1 = 0;
            }
        } else {
            SPWM_SW_PIN = 0; // Ensure PWM is off during fault
        }
    }

    // ADC Interrupt (if using interrupt-driven ADC)
    /* if (PIR1bits.ADIF && PIE1bits.ADIE) {
        PIR1bits.ADIF = 0; // Clear ADC interrupt flag
        // Handle ADC result
    } */
}
