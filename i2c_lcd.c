#include "i2c_lcd.h"
#include <xc.h> // Required for _XTAL_FREQ and pin definitions if not using MCC
// _XTAL_FREQ should be defined in main.c or a global config
// For delays, ensure _XTAL_FREQ is defined before this file is compiled if it's used here directly
// or pass it, or use a common config header.
// We assume _XTAL_FREQ is globally available from main.c's #define

// Define I2C pins if not already defined by device specific header
// For PIC16F73, SCL is RC3 and SDA is RC4
#define SCL_PIN TRISCbits.TRISC3
#define SDA_PIN TRISCbits.TRISC4
#define SCL_PORT PORTCbits.RC3
#define SDA_PORT PORTCbits.RC4

// I2C timing - dependent on _XTAL_FREQ. For 8MHz, a short delay is needed.
// A simple delay, more precise timing might be needed for faster I2C speeds
// or noisy environments.
#define I2C_DELAY() __delay_us(5) // Adjust as needed for 100kHz I2C with 8MHz clock

// Current backlight state
static unsigned char _backlight_val = LCD_BACKLIGHT;

//----------------------------------------------------------------------
// I2C Helper Functions (Bit-banging)
//----------------------------------------------------------------------

void i2c_init(void) {
    SCL_PIN = 1; // SCL as input (open-drain like)
    SDA_PIN = 1; // SDA as input (open-drain like)
    // Actual TRIS setting for output mode will be 0, but they are often switched
    // For bit banging, we manually control levels and TRIS often.
    // Set to output mode to drive low, input mode to let pull-up drive high.
    SCL_PIN = 0; // SCL as output
    SDA_PIN = 0; // SDA as output
    SCL_PORT = 1; // Clock high
    SDA_PORT = 1; // Data high
}

void i2c_start(void) {
    SDA_PIN = 0; // Ensure SDA is output for start condition
    SDA_PORT = 1;
    I2C_DELAY();
    SCL_PORT = 1;
    I2C_DELAY();
    SDA_PORT = 0; // START condition: SDA goes low while SCL is high
    I2C_DELAY();
    SCL_PORT = 0; // Pull SCL low to start data transfer
    I2C_DELAY();
}

void i2c_stop(void) {
    SDA_PIN = 0; // Ensure SDA is output for stop condition
    SCL_PORT = 0;
    I2C_DELAY();
    SDA_PORT = 0;
    I2C_DELAY();
    SCL_PORT = 1;
    I2C_DELAY();
    SDA_PORT = 1; // STOP condition: SDA goes high while SCL is high
    I2C_DELAY();
    SDA_PIN = 1; // Release SDA line
}

// Writes a byte and waits for ACK
void i2c_write(unsigned char data) {
    unsigned char i;
    SDA_PIN = 0; // Ensure SDA is output
    for (i = 0; i < 8; i++) {
        SCL_PORT = 0;
        I2C_DELAY();
        if ((data & 0x80) == 0) // MSB first
            SDA_PORT = 0;
        else
            SDA_PORT = 1;
        data <<= 1;
        I2C_DELAY();
        SCL_PORT = 1; // Clock high, slave reads data
        I2C_DELAY();
    }
    // Get ACK
    SCL_PORT = 0;
    SDA_PIN = 1;  // Release SDA line for slave to ACK
    I2C_DELAY();
    SCL_PORT = 1;
    I2C_DELAY();
    // while(SDA_PORT); // Wait for ACK (SDA low) - can hang if no ACK
    // For robustness, add a timeout or check SDA_PORT here
    // For now, we assume ACK is received.
    SCL_PORT = 0;
    I2C_DELAY();
    SDA_PIN = 0; // Take control of SDA again
}

// Reads a byte and sends ACK/NACK
unsigned char i2c_read(unsigned char ack) {
    unsigned char i, data = 0;
    SDA_PIN = 1; // Release SDA to allow slave to write
    for (i = 0; i < 8; i++) {
        SCL_PORT = 0;
        I2C_DELAY();
        SCL_PORT = 1; // Clock high for slave to output data
        I2C_DELAY();
        data <<= 1;
        if (SDA_PORT) {
            data |= 1;
        }
    }
    // Send ACK/NACK
    SDA_PIN = 0; // Take control of SDA
    SCL_PORT = 0;
    I2C_DELAY();
    if (ack)
        SDA_PORT = 0; // ACK
    else
        SDA_PORT = 1; // NACK
    I2C_DELAY();
    SCL_PORT = 1; // Clock out ACK/NACK
    I2C_DELAY();
    SCL_PORT = 0;
    SDA_PIN = 1; // Release SDA
    I2C_DELAY();
    return data;
}

//----------------------------------------------------------------------
// LCD Specific Functions (using I2C backpack)
//----------------------------------------------------------------------

// Pulse Enable pin on I2C backpack
void lcd_pulse_enable(unsigned char data_with_en) {
    i2c_write(data_with_en | 0x04); // En high
    __delay_us(1); // Enable pulse must be >450ns
    i2c_write(data_with_en & ~0x04); // En low
    __delay_us(50); // commands need > 37us to settle
}

// Send 4 bits (nibble) to LCD via I2C backpack
// data is the nibble, rs_mode is 0 for command, 1 for data
void lcd_send_nibble(unsigned char nibble, unsigned char rs_mode) {
    unsigned char data_to_send;
    data_to_send = (nibble << 4) & 0xF0; // Map nibble to D4-D7
    data_to_send |= (rs_mode ? 0x01 : 0x00); // Set RS bit
    data_to_send |= _backlight_val; // Maintain backlight state

    i2c_write(data_to_send);
    lcd_pulse_enable(data_to_send);
}

// Send a full byte (command or data) to LCD
// byte is the data/command, rs_mode is 0 for command, 1 for data
void lcd_send_byte(unsigned char byte, unsigned char rs_mode) {
    // Send high nibble
    lcd_send_nibble((byte >> 4) & 0x0F, rs_mode);
    // Send low nibble
    lcd_send_nibble(byte & 0x0F, rs_mode);
}

void lcd_command(unsigned char cmd) {
    i2c_start();
    i2c_write((LCD_I2C_ADDRESS << 1) | 0x00); // Address + Write bit
    lcd_send_byte(cmd, 0); // RS=0 for command
    i2c_stop();
    if (cmd == LCD_CLEAR_DISPLAY || cmd == LCD_RETURN_HOME) {
        __delay_ms(2); // These commands take longer
    }
}

void lcd_data(unsigned char data) {
    i2c_start();
    i2c_write((LCD_I2C_ADDRESS << 1) | 0x00); // Address + Write bit
    lcd_send_byte(data, 1); // RS=1 for data
    i2c_stop();
}

void lcd_init(void) {
    i2c_init(); // Initialize I2C pins
    __delay_ms(50); // Wait for LCD to power up

    // Put LCD into 4-bit mode
    // This sequence is from Hitachi HD44780 datasheet
    i2c_start();
    i2c_write((LCD_I2C_ADDRESS << 1) | 0x00);

    lcd_send_nibble(0x03, 0); // Function set (8-bit mode) - 1st time
    __delay_ms(5);
    lcd_send_nibble(0x03, 0); // Function set (8-bit mode) - 2nd time
    __delay_us(150);
    lcd_send_nibble(0x03, 0); // Function set (8-bit mode) - 3rd time
    __delay_us(150);
    lcd_send_nibble(0x02, 0); // Function set (4-bit mode)
    i2c_stop();
    __delay_us(150);

    // Now in 4-bit mode, send standard commands
    lcd_command(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8DOTS);
    lcd_command(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    lcd_command(LCD_CLEAR_DISPLAY);
    __delay_ms(2); // Clear display takes longer
    lcd_command(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
    lcd_command(LCD_RETURN_HOME);
    __delay_ms(2);

    // Turn on backlight (if desired, default is on)
    _backlight_val = LCD_BACKLIGHT; // Set default backlight state
}

void lcd_set_cursor(unsigned char row, unsigned char col) {
    unsigned char row_offsets[] = {0x00, 0x40}; // DDRAM addresses for start of lines
    if (row >= LCD_ROWS) {
        row = LCD_ROWS - 1; // Cap row to max
    }
    if (col >= LCD_COLS) {
        col = LCD_COLS - 1; // Cap col to max
    }
    lcd_command(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

void lcd_print_char(char c) {
    lcd_data(c);
}

void lcd_print_string(const char* str) {
    while (*str) {
        lcd_print_char(*str++);
    }
}

void lcd_clear(void) {
    lcd_command(LCD_CLEAR_DISPLAY);
    __delay_ms(2); // This command needs extra time
}

// Example: Control backlight
void lcd_set_backlight(unsigned char on) {
    if (on) {
        _backlight_val = LCD_BACKLIGHT;
    } else {
        _backlight_val = LCD_NOBACKLIGHT;
    }
    // To make it take effect immediately, we need to send a benign command
    // or just let the next LCD operation update it.
    // Forcing an update (e.g. by re-sending last command/data or a dummy command)
    // For now, it will update on next char/cmd.
    // To force:
    i2c_start();
    i2c_write((LCD_I2C_ADDRESS << 1) | 0x00); // Address + Write bit
    i2c_write(_backlight_val); // Send just the backlight state (PCF8574 raw write)
    i2c_stop();
}
