#ifndef I2C_LCD_H
#define I2C_LCD_H

#include <xc.h> // For _XTAL_FREQ definition and basic types

// Define LCD I2C Address - this can vary, 0x27 is common
#define LCD_I2C_ADDRESS 0x27 // Or 0x3F depending on the backpack

// Define LCD dimensions
#define LCD_ROWS 2
#define LCD_COLS 16

// LCD Commands
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT 0x10
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CGRAM_ADDR 0x40
#define LCD_SET_DDRAM_ADDR 0x80

// Entry Mode Set flags
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// Display Control flags
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

// Function Set flags
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// Backlight control (for I2C backpack)
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// Functions
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(unsigned char data);
unsigned char i2c_read(unsigned char ack);

void lcd_send_nibble(unsigned char nibble, unsigned char rs_mode);
void lcd_send_byte(unsigned char byte, unsigned char rs_mode);
void lcd_command(unsigned char cmd);
void lcd_data(unsigned char data);
void lcd_init(void);
void lcd_set_cursor(unsigned char row, unsigned char col);
void lcd_print_char(char c);
void lcd_print_string(const char* str);
void lcd_clear(void);

#endif // I2C_LCD_H
