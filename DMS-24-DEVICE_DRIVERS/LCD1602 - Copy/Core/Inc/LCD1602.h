/*
 * LCD1602.h
 *
 *  Created on: Dec 21, 2023
 *      Author: Asthon Dudley
 */

#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_


#define RS_PORT GPIOE
#define RS_PIN GPIO_PIN_7
#define RW_PORT GPIOE
#define RW_PIN GPIO_PIN_8
#define EN_PORT GPIOE
#define EN_PIN GPIO_PIN_9
#define D4_PORT GPIOE
#define D4_PIN GPIO_PIN_10
#define D5_PORT GPIOE
#define D5_PIN GPIO_PIN_11
#define D6_PORT GPIOE
#define D6_PIN GPIO_PIN_12
#define D7_PORT GPIOE
#define D7_PIN GPIO_PIN_13



void delay (uint16_t us);

void lcd_send_cmd (char cmd);

void send_to_lcd (char data, int rs);

void lcd_send_data (char data);

void lcd_put_cur(int row, int col);

void lcd_send_string (char *str);

void lcd_init (void);

void lcd_clear (void);




#endif /* INC_LCD1602_H_ */
