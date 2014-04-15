#include <msp430g2553.h>
#include "Symbols.h"
#include "TI_USCI_I2C_master.h"

/* Bit operations */
#define BIT_SET(lval, mask)     ((lval) |= (mask))
#define BIT_CLR(lval, mask)     ((lval) &= ~(mask))
#define BIT_TEST(val, mask)     (((val) & (mask))==(mask))
/* BSL */
#define TXD    					BIT1						// P1.1: BSL TxD
#define RXD    					BIT5						// P1.5: BSL RxD
/* BT */
#define BT_TXD    				BIT1						// P2.1 (Timer1_A.CCI1A): UART BT TxD
#define BT_RXD    				BIT0						// P2.0 (Timer1_A.OUT0): UART BT RxD
#define BT_PWR    				BIT2						// P2.2,P3.2
#define BT_LED    				BIT3						// P3.3
/* LCD */
#define PIN_RESET 				BIT2						// P1.2  RESET
#define PIN_SCE 				BIT3						// P1.3  CS
#define PIN_SDIN 				BIT4						// P1.4  SDA //mosi
#define PIN_SCLK 				BIT1						// P3.1  SCK
#define PIN_LED 				BIT0						// P3.0  ��������� �������
#define LCD_C 					0							// Command
#define LCD_D 					1							// Data
/* Buttons & vibro */
#define B_CENT    				BIT4						// P2.4
#define B_UP    				BIT3						// P2.3
#define B_DOWN    				BIT5						// P2.5
#define vibro   				BIT4						// P3.4
/* System configuration */
#define TIMER1A_CLOCK  			1000000L					// Timer1_A clock rate (1 MHz)
#define UART_BAUD       		9600    					// desired UART baud rate
#define BT_BITTIME 				(TIMER1A_CLOCK/UART_BAUD)   // Bit interval
#define BT_HALF_BT 				((BT_BITTIME+1)/2)       	// Half-bit interval
#define Slave_Address  			0x68  						// address RTC

unsigned char RX[16];
unsigned char TX[17] = 		{  	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						 	 	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 	   };

#define max_menu_item 			7		// ���������� ����� � ����

unsigned int 	tx_data_bt;           	// UART Tx data

unsigned char 	bat_sost = 0, 			// ��������� ������� 0-100% - 4-10%
				current_menu_item = 1, 	// ������� ����� ����
				current_sub_menu_item = 0, // ������� ����� �������
				screens = 0,			// ���������� ������� �������������� ��������� (�������� 3(���� � 0, ������ 2))
				black_text = 0,			// �������� ������
				current_screen = 0, 	// 0 - main watch, 1 - sms|call, 2 - setting_main, 3 - submenu, 4 - multiscreen text;
				char_count = 0,			// ���������� �������� ���������
				count_sec = 1,			// ������� "������"
				count_sec2 = 1,			// ������� "������"
				text_screen = 0; 		// ������� ����� �������������� ���������

int 			i_timer = 0; 			// ������� ������� ������ ������� �����������������

unsigned char	edit_time = 0, 			// ������������� ��������� ����/�������?
				get_time = 0,			// �������� ����� � ��� � �������� �� FLASH ��� ������� �� FLASH?
				set_time = 0, 			// �������� ����� �� ���?
				check_akk = 0, 			// ��������� ��������� ������������?
				v2_5 = 0,				// Vref ��� ADC10 - ref 2.5V?
				bt_on = 0, 				// BT �������?
				bt_connect = 0, 		// BT ���������?
				time_from_phone = 0,	// ��������� ����� � ��������?
				multiscreen = 0,		// ������������� ���������?
				dot_show = 0,			// ���������� �����?
				P2_int = 0,				// ���������� �� P2
				TA1_int = 0,			// ���������� �� TA1
				ADC_int = 0,			// ���������� �� ADC
				btn_pressed = 0,		// 0-������, 1,2,3 - up, cent, down; 4,5,6 - long up, cent, down;
				string_ready = 0,		//
				e_save = 0,				// LPM
				call_true = 0;			// �������� �����

char inputString[314] = "";				// ������ ��������� �� UART

const unsigned char Main_menu[][17] =
			  { "�����           ",
				"�������         ",
				"�������������   ",
				"����� ����������",
				"��� / ���� bt   ",
				"����            ",
				"����� � ��������" }; 	// ���� ��������

										// �� FLASH'�

int 									// ���������� ����� � ���������
				h10 = 1, 	h1 = 7, 	// ����
				m10 = 1, 	m1 = 3, 	// ������
				s10 = 5, 	s1 = 7, 	// �������
				d10 = 1, 	d1 = 3, 	// ����
				mo10 = 0,	mo1 = 3, 	// �����
				ye10 = 1, 	ye1 = 4, 	// ���
				dw = 4;					// ���� ������

int 			contrast = 24, 			// ������������� (0-31)
				pwm_width = 0x0eff, 	// ������� 0x00ff - 0x0fff;
				timer_off = 80;			// ����� �� ����������

/* Forward references */
void init_watch(void);

void check_akkum(void);				// �������� ��������� ������������
void check_bluetooth(void);			// �������� ��������� BT

void uart_tx_bt(char c);			// �������� ������� �� BT
void uart_puts_bt(char const* s);	// �������� ������ �� BT

void get_time_from_rtc(void);		// ��������� ������� � ��� � ������ �� FLASH
void set_time_to_rtc(void);			// ������ ������� �� ���

void LcdCharacter(char character);						// ������� ������
void LcdClear(void);									// ��������� ���� �����
void clear_1(void);										// ��������� �����, ����� ������ ����
void Lcd_set_pos(unsigned char c, unsigned char r);		// ���������� ������� � �������� �� ������
void Lcd_set_pos_pix(unsigned char c, unsigned char r);	// ���������� ������� � �������� �� ������
void lcd_contrast(unsigned char contrast2);				// ���������� ������������� ������
void lcd_dig(unsigned char num, unsigned char pos_x, unsigned char pos_y);	// ����� �����
void lcd_dot(unsigned char num, unsigned char pos_x, unsigned char pos_y);	// ��������� �����
void LcdWrite(unsigned char dc, unsigned char data);	// �������� ��������/������ �� �����
void LcdString(char *characters);						// ������� ������
void lcd_show_sms(unsigned char a);						// ������ ���
void lcd_show_call(unsigned char a);					// ������ ������
void lcd_show_bt(unsigned char a);						// ������ BT
void lcd_show_bat(unsigned char proc);					// ������ ������������
void lcd_set_time_big(void);							// ������� ������� ����
void lcd_set_time_small(void);							// ������� ����� ���� (� ������ ����)
void lcd_show_main(void);								// �������� ������� ����� (���� � ������� ����)

void menu_setting(unsigned char submenu);				// ����
void down_sub_menu(void);								// ������������ �� ����
void up_sub_menu(void);									//		� ��������� � �������
void parse_string(void);								// ������ �������� ������
//
// ����������
//
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_isr(void) {}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_isr(void) {}

#pragma vector=COMPARATORA_VECTOR
__interrupt void COMPARATOR_isr(void) {}

#pragma vector=PORT1_VECTOR
__interrupt void P1_isr(void) {}

#pragma vector=NMI_VECTOR
__interrupt void NMI_isr(void) {}

#pragma vector=TIMER1_A0_VECTOR						// ����� ������ �� UART �� BT
__interrupt void TIMER1_A0_isr(void) {
	static unsigned char rx_bitc1 = 8; 			 	// Rx bit count register
	static unsigned char rx_data1;      			// Rx data register

	BIT_CLR(IE1, WDTIE);
	BIT_CLR(TA1CCTL2, CCIE);
	BIT_CLR(ADC10CTL0, ADC10IE);					//���������� ����������

	TA1CCR0 += BT_BITTIME;         					// add offset to CCR
	if (TA1CCTL0 & CAP) {
		BIT_CLR(TA1CCTL0, CAP);
		TA1CCR0 += BT_HALF_BT;     					// point CCR to middle of D0
	} else {
		rx_data1 >>= 1;
		if (TA1CCTL0 & SCCI) {
			rx_data1 |= 0x80;
		}
		if (--rx_bitc1 == 0) {
			BIT_SET(TA1CCTL0, CAP);
			if ((char_count == 0) && ((rx_data1 < 0x31) || (rx_data1 > 0x36))) {	// ����������� �����
			} else {
				if (rx_data1 == '\n')						// �������� ������� ������ �� ������
					rx_data1 = ' ';							// �������� ����� �� ������
				inputString[char_count++] = rx_data1;
				if (rx_data1 == 0x00) {						// ����� ������������� ���������
					BIT_SET(TACCTL2, OUTMOD_6);				// ��������� ���������
					i_timer = 0;							// ��������� �������� ������� ������ ������� �����������������
					if (char_count > 105) {					// ��������� �� ���������� �� ���� �����
						multiscreen = 1;
						text_screen = 0;
						screens = 1;
						if (char_count > 208)				// ��������� �� ���������� �� ��� ������
							screens = 2;
					} else
						multiscreen = 0;
					inputString[char_count++] = 0x00;
					inputString[char_count++] = 0x00;
					inputString[char_count++] = 0x00;
					string_ready = 1;
					BIT_SET(IE1, WDTIE);
					BIT_SET(TA1CCTL2, CCIE);
					BIT_SET(ADC10CTL0, ADC10IE);			// ��������� ����������
					char_count = 0;
					e_save = 0;
					__bic_SR_register_on_exit(LPM3_bits);	// ����� �� ������ ����������������
				}
			}
			rx_bitc1 = 8;            // re-load bit counter
		}
	}
}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_isr(void) {
	static unsigned char tx_bitc1 = 10;  			// Tx bit count register

	switch (__even_in_range(TA1IV, TA1IV_TAIFG)) {
	case TA1IV_TACCR1:								// ��������� ������ �� UART �� BT
		if (tx_bitc1--) {
			BIT_CLR(IE1, WDTIE);
			BIT_CLR(TA1CCTL2, CCIE);
			BIT_CLR(ADC10CTL0, ADC10IE);			// ���������� ����������
			TA1CCR1 += BT_BITTIME;
			if (tx_data_bt & 0x01)
				BIT_CLR(TA1CCTL1, OUTMOD_6);
			else
				BIT_SET(TA1CCTL1, OUTMOD_5);
			tx_data_bt >>= 1;
		} else {
			TA1CCTL1 = OUT;              			// TxD idle as Mark, disable Tx int
			tx_bitc1 = 10;               			// reset bit counter
			BIT_SET(IE1, WDTIE);
			BIT_SET(TA1CCTL2, CCIE);
			BIT_SET(ADC10CTL0, ADC10IE);			// ��������� ����������
		}
		break;
	case TA1IV_TACCR2:								// ������� ������� ������ ������� �����������������
		TA1_int = 1;
		break;
	}
}

#pragma vector = WDT_VECTOR
__interrupt void WDT_isr(void) {					// ���������� WDT � ������ ������������� ������� (�������� ���������)
	BIT_CLR(TA1CCTL2, CCIE);
	BIT_CLR(ADC10CTL0, ADC10IE);					// ���������� ����������
	if (e_save)
	{
		if (current_screen == 0){
			dot_show++;								// ������� ��� ���� ��������
			if (dot_show >1)						// ������� �������
				dot_show = 0;
			lcd_dot(dot_show, 46, 2);
		}
		if (++count_sec2>19){						// ������ ������ ������� �������
			count_sec2 = 1;							// � ������ ����������������
			if (m1<9)								// �.�. ��� ����� � ���
				m1++;
			else
				if (m10<5){
					m10++;
					m1=0;;
				}
				else
				{
					m10=0;
					m1=0;
					if (h10<2)
						if (h1<9)
							h1++;
						else{
							h1=0;
							h10++;
						}
					else
					{
						if (h1<3)
							h1++;
						else{
							h10=0;
							h1=0;
						}
					}
				}
			lcd_set_time_small();
		}
	}
	check_akk = 1;									// �������� ������
	if (!edit_time){
		get_time = 1;
	}												// ������� �����
	if (bt_connect) {
		check_bluetooth();							// �������� BT
		if (bt_on == 0) {							// BT ��������
			e_save = 0;
			__bic_SR_register_on_exit(LPM3_bits);	// ����� �� ����� LPM3
			BIT_SET(TACCTL2, OUTMOD_6);				// ��������� ���������
		}
	}
	BIT_SET(TA1CCTL2, CCIE);
	BIT_SET(ADC10CTL0, ADC10IE);					// ��������� ����������
}

#pragma vector=PORT2_VECTOR
__interrupt void P2_isr() // ���������� ����������
{
	if (P2IFG & BT_RXD) {					// �� UART'� � BT ���� ������ � ������ ������
		get_time = 0;
		set_time = 0;
		check_akk = 0;
		ADC_int = 0;
		P2_int = 0;							// ������� ��� �������� � ����������� ����������
		BIT_CLR(IE1, WDTIE);
		BIT_CLR(TA1CCTL2, CCIE);
		BIT_CLR(ADC10CTL0, ADC10IE);		// ���������� ����������
		BIT_CLR(P2IE, BT_RXD);
		BIT_CLR(P2IFG, BT_RXD);
		BIT_SET(P2SEL, BT_RXD);				// ����������� ��� BT_RXD �� ���� �������
		TA1CCR0 = TA1R;
	} else {								// ���������� �� ������� ������
		BIT_CLR(P2IE, B_CENT|B_UP|B_DOWN);
		BIT_CLR(IE1, WDTIE);
		BIT_CLR(TA1CCTL2, CCIE);
		BIT_CLR(ADC10CTL0, ADC10IE);		// ���������� ����������
		BIT_CLR(P2IE, BT_RXD);
		BIT_CLR(P2IFG, BT_RXD);
		BIT_SET(P2SEL, BT_RXD);				// ����������� ��� BT_RXD �� ���� �������
		char long_press = 1;
		unsigned long a;
		BIT_SET(TACCTL2, OUTMOD_6);			// ��������� ��������� � �������
		i_timer = 0;  						// ��������� ��������
		if (P2IFG & B_CENT) { 				// ���������� �� B_CENT
			for (a = 200000; a > 0; a--)	// ����������� � �������� �������� �������
				if (P2IN & B_CENT)
					long_press = 0;
			if (long_press)
				btn_pressed = 5;
			else
				btn_pressed = 2;
		}
		if (P2IFG & B_DOWN) {  					// ���������� �� B_DOWN
			for (a = 200000; a > 0; a--)		// ����������� � �������� �������� �������
				if (P2IN & B_DOWN)
					long_press = 0;
			if (long_press)
				btn_pressed = 6;
			else
				btn_pressed = 3;
		}
		if (P2IFG & B_UP) {  							// ���������� �� B_UP
			for (a = 200000; a > 0; a--)				// ����������� � �������� �������� �������
				if (P2IN & B_UP)
					long_press = 0;
			if (long_press)
				btn_pressed = 4;
			else
				btn_pressed = 1;
		}
		P2_int = 1;
		BIT_SET(P2IE, B_CENT|B_UP|B_DOWN);
		BIT_CLR(P2IFG, B_CENT|B_UP|B_DOWN);
		BIT_SET(IE1, WDTIE);
		BIT_SET(TA1CCTL2, CCIE);
		BIT_SET(ADC10CTL0, ADC10IE);
	}
	e_save = 0;
	__bic_SR_register_on_exit(LPM3_bits);			// ����� �� LPM3

}

#pragma vector=ADC10_VECTOR							// ���������� ���������� �������������� ���
__interrupt void ADC10_isr(void) {
	ADC_int = 1;
}
// ����������

void check_akkum(void) {
	unsigned int adc;
	adc = ADC10MEM;
	if (v2_5) {						// ��������� ��������� ������ �� ������
		bat_sost = 4;
	} else {
		bat_sost = 4;			// ��������� ����������� � ���������� ����������
		if (adc > 0x034e)
			bat_sost = 3;
		if (adc > 0x035a)
			bat_sost = 2;
		if (adc > 0x036f)
			bat_sost = 1;
		if (adc > 0x0389)
			bat_sost = 0;
	}
	if (adc >= 0x03ff) {			// ��� �������, ������� �� ������ ����� �������� ������ 2,5 V
		v2_5 = 0;
	}
	if (adc < 0x030c) {				// ��� �������, ������� �� ������ ����� �������� ������ 2,5 V
		v2_5 = 1;
	}
	lcd_show_bat(bat_sost);
}

void check_bluetooth(void) {		// �������� �������� BT
	unsigned int i;
	char disconnect = 0;
	for (i = 65000; i > 0; i--)		// ���� BT ��������, �� BT_LED ���. 1, ����� 10101010
		if (!(P3IN & BT_LED)) {		// ����� ���. 0, ���� ��������, ������ BT ����������
			disconnect = 1;
		}
	if (disconnect) {
		i_timer = 0;
		BIT_CLR(P2OUT, BT_PWR);
		BIT_CLR(P3OUT, BT_PWR);		// ��������� ������� BT
		lcd_show_bt(0);
		lcd_show_sms(0);
		lcd_show_call(0);
		bt_on = 0;
		bt_connect = 0;
		current_screen = 1;
		clear_1();
		Lcd_set_pos(0, 2);
		LcdString("BLUETOOTH      ");
		Lcd_set_pos(0, 3);
		LcdString("    ����������!");
		BIT_SET(P3OUT, vibro);
		__delay_cycles(800000);
		BIT_CLR(P3OUT, vibro);
		__delay_cycles(800000);
		BIT_SET(P3OUT, vibro);
		__delay_cycles(800000);
		BIT_CLR(P3OUT, vibro);
		__delay_cycles(800000);
		BIT_SET(P3OUT, vibro);
		__delay_cycles(800000);
		BIT_CLR(P3OUT, vibro);
		__delay_cycles(800000);
		BIT_SET(P3OUT, vibro);
		__delay_cycles(800000);
		BIT_CLR(P3OUT, vibro);			// ������������� �� ����������� BT
		BIT_SET(IE1, WDTIE);
		BIT_SET(TA1CCTL2, CCIE);
		BIT_SET(ADC10CTL0, ADC10IE);	// ��������� ����������
		BIT_CLR(P2IE, BT_RXD);
		BIT_CLR(P2IFG, BT_RXD);
		BIT_SET(P2SEL, BT_RXD);			// BT_RXD ��� ���� ��� �������
	}
}

void main(void) {
	/* stop WDT */
	WDTCTL = WDTPW + WDTHOLD;
	init_watch();
	for (;;) {
		if (string_ready)
		{
			string_ready=0;
			parse_string();
		}
		if (P2_int){		// ���������� �� P2
			P2_int = 0;
			switch (btn_pressed) {
			case 0:
				break;
			case 1:		// up
				if (current_screen == 2) { 			// ����� ������  � ����
					current_menu_item--;
					if (current_menu_item <= 0)
						current_menu_item = max_menu_item;
					menu_setting(0);
				} else if (current_screen == 3)   			// ��������� � �������
					up_sub_menu();
				else if (current_screen == 4) {				// ������������� ���������, �������� ���������� �����
					unsigned int l, k = 15, z = 1;
					if (text_screen - 1 > 0) {				// �� ������ �����
						clear_1();
						Lcd_set_pos(0, 1);
						text_screen--;
						l = 1;
						while (l < 104) {
							LcdCharacter(inputString[l + 104]);
							if (++l > k){
								k+=15;
								Lcd_set_pos(0, ++z);
							}
						}
						Lcd_set_pos(13, 7);
						LcdCharacter(0x7e);
						LcdCharacter(0x7f);
					} else if (text_screen - 1 == 0) {		// ������ �����
						clear_1();
						Lcd_set_pos(0, 1);
						text_screen--;
						l = 1;
						while (l < 105) {
							LcdCharacter(inputString[l]);
							if (++l > k){
								k+=15;
								Lcd_set_pos(0, ++z);
							}
						}
						Lcd_set_pos(14, 7);
						LcdCharacter(0x7f);
					}
				}
				break;
			case 2:		// cent
				if (current_screen == 2) {		// ���� -> �������
					current_sub_menu_item = 0;
					menu_setting(current_menu_item);
				} else if (current_screen == 3) {	// ������� -> ����. �������
					current_sub_menu_item++;
					menu_setting(current_menu_item);
				}
				break;
			case 3:		// down
				if (current_screen == 2) { 	// ����������� �� ����
					current_menu_item++;
					if (current_menu_item > max_menu_item)
						current_menu_item = 1;
					menu_setting(0);
				} else if (current_screen == 3)   	// ��������� � �������
					down_sub_menu();
				else if (current_screen == 4) {		// ������������� ���������, �������� ��������� �����
					unsigned int l, k , z = 1, m = 15;
					if ((text_screen + 1) < screens) {			// �� ��������� �����
						clear_1();
						Lcd_set_pos(0, 1);
						text_screen++;
						l = 1;
						while (l < 104) {
							LcdCharacter(inputString[l + 104]);
							if (++l > m){
								Lcd_set_pos(0, ++z);
								m += 15;
							}
						}
						Lcd_set_pos(13, 7);
						LcdCharacter(0x7e);
						LcdCharacter(0x7f);
					} else if ((text_screen + 1) == screens) {	// ��������� �����
						clear_1();
						Lcd_set_pos(0, 1);
						text_screen++;
						l = 1;
						if (screens == 1)
							k = 104;
						else
							k = 208;
						while (inputString[l + k] != 0x00) {
							LcdCharacter(inputString[l + k]);
							if (++l > m){
								Lcd_set_pos(0, ++z);
								m += 15;
							}
						}
						Lcd_set_pos(14, 7);
						LcdCharacter(0x7e);
					}
				}
				break;
			case 4:		// long up
				if (call_true == 1){
					uart_tx_bt('2');
				}
				if (current_screen == 0)				// ��������� ����� -> ����
					menu_setting(0);
				else {									// ����� ����� -> ��������� �����
					multiscreen = 0;
					unsigned int il;
					for (il = 313; il > 0; il--)
						inputString[il] = 0;
					lcd_show_main();
					edit_time = 0;
					get_time = 1;
				}
				break;
			case 5:		// long cent
				if (call_true == 1){
					uart_tx_bt('2');
				}
				if (current_screen == 0)		// ��������� ����� -> ����
					menu_setting(0);
				else {							// ����� ����� -> ��������� �����
					multiscreen = 0;
					unsigned int il;
					for (il = 313; il > 0; il--)
						inputString[il] = 0;
					lcd_show_main();
					edit_time = 0;
					get_time = 1;				// ������� �����
				}
				break;
			case 6:		// long down
				if (call_true == 1){
					uart_tx_bt('2');
				}
				if (current_screen == 0)		// ��������� ����� -> ����
					menu_setting(0);
				else {							// ����� ����� -> ��������� �����
					multiscreen = 0;
					unsigned int il;
					for (il = 313; il > 0; il--)
						inputString[il] = 0;
					lcd_show_main();
					edit_time = 0;
					get_time = 1;
				}
				break;
			}
		}
		if (TA1_int){		// ���������� �� TA1
			TA1_int = 0;
			if (!e_save){
				if (++count_sec == 15){
					if (current_screen == 0){
						dot_show++;
						if (dot_show >1)
							dot_show = 0;
						lcd_dot(dot_show, 46, 2);
					}
					count_sec = 1;
				}
				if (++i_timer == timer_off) {				// ����� �� ���������� ������
					if ((bt_on == 1) && (bt_connect == 0)) {
						i_timer = 0;
					} else {
						BIT_CLR(ADC10CTL0, ADC10IE);
						i_timer = -1;
						BIT_CLR(TACCTL2, OUTMOD_6);			// ��������� ���������
						BIT_CLR(TACCTL2, OUT);				// ��������� ���������
						count_sec2 = s10*10+s1;
						if (count_sec2>45)
							count_sec2 = 17;
						else
							if (count_sec2>30)
								count_sec2 = 11;
							else
								if (count_sec2>15)
									count_sec2 = 6;
								else
									count_sec2 = 2;
						BIT_CLR(P2SEL, BT_RXD);
						BIT_SET(P2IES, BT_RXD);
						BIT_CLR(P2IFG, BT_RXD);
						BIT_SET(P2IE, BT_RXD);				// ����������� ��� BT_RXD �� ���������� (� ������ LPM3 SMCLK ��������, ������ �� ��������)
						e_save = 1;
						LPM3;
					}
				}
			}
		}
		if (ADC_int){		// ���������� �� ADC
			ADC_int = 0;
			BIT_CLR(IE1, WDTIE);							// ���������� ����������
			BIT_CLR(TA1CCTL2, CCIE);
			check_akkum();									// ��������� ��������� ��� � �������� ������ ������
			BIT_SET(IE1, WDTIE);
			BIT_SET(TA1CCTL2, CCIE);						// ��������� ����������
		}
		if (set_time) {		// �������� ����� � ���
			set_time_to_rtc();
			__delay_cycles(2000000);
			get_time = 1;
			edit_time = 0;
			set_time = 0;
		}
		if (get_time) {		// ������� ����� �� ���
			get_time_from_rtc();
			edit_time = 0;
			get_time = 0;
		}
		if (check_akk) {	// ��������� ����. ������
			if (!v2_5) {
				ADC10CTL0 = 0;
				ADC10CTL0 = SREF_0 + ADC10SR + ADC10SHT_2 + ADC10ON + ADC10IE + ENC + ADC10SC; // Vref = Vcc
			}
			if (v2_5) {
				ADC10CTL0 = 0;
				ADC10CTL0 = SREF_1 + ADC10SR + REF2_5V + ADC10SHT_2 + REFON	+ ADC10ON + ADC10IE + ENC + ADC10SC; // Vref = 2.5 v
			}
			check_akk = 0;
		}
		if (time_from_phone) {// ��������� ����� � ��������
			uart_tx_bt('1');
			time_from_phone = 0;
		}
	}
}


void init_watch(void){
	/* initialize hardware */
	// init clocks
	BCSCTL1 = CALBC1_8MHZ;
	DCOCTL = CALDCO_8MHZ;
	BCSCTL2 = 0;                    // MCLK = 8MHz/1,SMCLK = 8MHz/1
	//for WDT+
	BCSCTL3 = LFXT1S_2;				//Mode 2 for LFXT1 : VLO = 12kHz

	__delay_cycles(800000);

	// ������ BSL ���� �� ����� � ���. 0
	BIT_SET(P1DIR, TXD | RXD);
	BIT_CLR(P1OUT, TXD | RXD);

	// init LCD
	edit_time = 0;
	// ����� ���� � ���������� ������ �� FLASH'�
	char *a;
	a = (char*) 0x104d;
	contrast = *a++;
	pwm_width = 0x00ff | (*a++) << 8;
	timer_off = ((*a++) & 0x00ff) << 8;
	timer_off |= ((*a++) & 0x00ff);
	// ��������� ������ ��� LCD
	BIT_SET(P1DIR, PIN_RESET + PIN_SCE + PIN_SDIN);
	BIT_SET(P3DIR, PIN_SCLK);
	BIT_CLR(P1OUT, PIN_RESET + PIN_SDIN);
	BIT_CLR(P3OUT, PIN_SCLK);
	BIT_SET(P1OUT, PIN_SCE);
	BIT_SET(P3DIR, PIN_LED);
	BIT_SET(P3OUT, PIN_LED);
	BIT_SET(P1OUT, PIN_RESET);
	// ������������������ �������������
	LcdWrite(LCD_C, 0xE2); 								// ����� ����������
	LcdWrite(LCD_C, 0x3D); 								// Charge pump ON
	LcdWrite(LCD_C, 0x01); 								// Charge pump=4
	LcdWrite(LCD_C, 0xA4); 								//
	LcdWrite(LCD_C, 0x2F); 								//
	//LcdWrite(LCD_C, 0xC8);							// ��������� ����-���
	LcdWrite(LCD_C, 0xC0); 								// ���������� ����-���
	//LcdWrite(LCD_C, 0xA1);							// ��������� ����-�����
	LcdWrite(LCD_C, 0xA0); 								// ���������� ����-�����
	__delay_cycles(800000);
	LcdWrite(LCD_C, 0xAF); 								// Display ON
	LcdClear();
	lcd_contrast(contrast);
	BIT_SET(P3SEL, PIN_LED);
	BIT_SET(P3DIR, PIN_LED);
	// ������ ��� ��� ���������
	TACTL = TASSEL_2 + ID_0 + MC_1 + TACLR;		// SMCLK +  divider /1 + ������ ���� �� TACCR0 + counter clear
	TACCR0 = 0x0fff;									// ������ ���
	TACCR2 = pwm_width;									// ���������� ��� ��� ���������
	TACCTL2 = OUTMOD_6;							// ����� ���

	// init Bluetooth
	BIT_SET(P3DIR, BT_PWR);
	BIT_CLR(P3REN, BT_PWR);
	BIT_SET(P3OUT, BT_PWR);
	BIT_SET(P2DIR, BT_PWR);
	BIT_CLR(P2REN, BT_PWR);
	BIT_SET(P2OUT, BT_PWR);	//������� �� ON
	BIT_CLR(P3DIR, BT_LED);	//��������� ��
	bt_on = 1;
	lcd_show_bt(1);
	//TimerA_1 ��� uart
	TA1CTL = TASSEL_2 + ID_3 + MC_2 + TAIE;	//TASSEL_2=SMCLK + input divider: 3 - /8 + mode control: 2 - Continous up + interrupt enable
	TA1CCTL1 = OUT; //Tx
	TA1CCTL0 = CM_2 + SCS + CAP + CCIE; //Rx
	//����� Rx � TX ��� ��
	BIT_SET(P2SEL, BT_TXD + BT_RXD);
	BIT_SET(P2DIR, BT_TXD);

	//������ ��� ����. ������
	BIT_SET(TA1CCTL2, CCIE);

	// ��������� ������
	BIT_CLR(P2DIR, B_CENT|B_UP|B_DOWN);
	// ����������� ����� - IN
	BIT_SET(P2REN, B_CENT|B_UP|B_DOWN);
	// ����������� ����������
	BIT_SET(P2IE, B_CENT|B_UP|B_DOWN);
	// ���������� ����������
	BIT_SET(P2IES, B_CENT|B_UP|B_DOWN);
	// ���������� ���������� �� 1/0 (����������/�������)
	BIT_CLR(P2IFG, B_CENT|B_UP|B_DOWN);
	// ������� ����� ����������

	// WDT+ ��� ������������ ������, ������� 12 ��� - ACLK - VLO
	WDTCTL = WDTPW + WDTTMSEL + WDTSSEL;
	BIT_SET(IE1, WDTIE);

	// vibro - �����
	BIT_SET(P3DIR, vibro);
	BIT_CLR(P3OUT, vibro);

	lcd_show_main();		// ������� �����
	__enable_interrupt();
	edit_time = 0;
	get_time = 0;
	set_time = 0;
	get_time_from_rtc();	// ��������� �����
	__delay_cycles(8000000);
	set_time_to_rtc();		// ��������� �����

	// init ADC
	ADC10CTL0 = SREF_1 + ADC10SR + REF2_5V + ADC10SHT_2 + REFON + ADC10ON + ADC10IE;// �������� ���+ REFON
	ADC10CTL1 = INCH_0; //A0
	ADC10AE0 |= 0x01; // ������������� 0-�� ��� ��� ���� ��� ���
}


void parse_string(void) {					// ������ ���������� ���������
	unsigned int il;
	unsigned char z,k;

	switch (inputString[0]) {
		case '1': {						// ��������� ������� ��������������� � BT ��������
			BIT_SET(P3OUT, vibro);
			__delay_cycles(1600000);
			BIT_CLR(P3OUT, vibro);
			__delay_cycles(1600000);
			BIT_SET(P3OUT, vibro);
			__delay_cycles(1600000);
			BIT_CLR(P3OUT, vibro);
			__delay_cycles(1600000);
			BIT_SET(P3OUT, vibro);
			__delay_cycles(1600000);
			BIT_CLR(P3OUT, vibro);
			lcd_show_bt(2);
			bt_connect = 1;
			break;
		}
		case '2': {						// ��������� ��������� ���
			current_screen = 1;
			lcd_show_sms(1);
			lcd_show_call(0);
			clear_1();
			Lcd_set_pos(0, 1);
			z = 1;
			il = 1;
			k = 15;
			if (multiscreen) {
				current_screen = 4;
				while (il < 105) {
					LcdCharacter(inputString[il]);
					if (++il > k){
						Lcd_set_pos(0, ++z);
						k += 15;
					}
				}
				LcdCharacter(0x7f);
			} else
				while (inputString[il] != 0x00) {
					LcdCharacter(inputString[il]);
					if (++il > k){
						Lcd_set_pos(0, ++z);
						k += 15;
					}
				}
			BIT_SET(P3OUT, vibro);
			__delay_cycles(2800000);
			BIT_CLR(P3OUT, vibro);
			__delay_cycles(2400000);
			BIT_SET(P3OUT, vibro);
			__delay_cycles(8000000);
			BIT_CLR(P3OUT, vibro);
			break;
		}
		case '3': {					// ��������� ��������� ������
			current_screen = 1;
			lcd_show_sms(0);
			lcd_show_call(1);
			clear_1();
			Lcd_set_pos(0, 2);
			il = 1;
			z = 2;
			k = 15;
			while (inputString[il] != 0x00) {
				LcdCharacter(inputString[il]);
				if (++il > k){
					Lcd_set_pos(0, ++z);
					k += 15;
				}
			}
			call_true = 1;
			BIT_SET(P3OUT, vibro);
			__delay_cycles(1600000);
			BIT_CLR(P3OUT, vibro);
			__delay_cycles(800000);
			BIT_SET(P3OUT, vibro);
			__delay_cycles(8000000);
			BIT_CLR(P3OUT, vibro);
			__delay_cycles(1600000);
			BIT_SET(P3OUT, vibro);
			__delay_cycles(1600000);
			BIT_CLR(P3OUT, vibro);
			__delay_cycles(800000);
			BIT_SET(P3OUT, vibro);
			__delay_cycles(8000000);
			BIT_CLR(P3OUT, vibro);
			break;
		}
		case '4': {					// ����������� ����������� ������
			current_screen = 1;
			lcd_show_sms(0);
			lcd_show_call(0);
			clear_1();
			Lcd_set_pos(0, 1);
			il = 1;
			z = 1;
			k = 15;
			if (multiscreen) {
				current_screen = 4;
				while (il < 105) {
					LcdCharacter(inputString[il]);
					if (++il > k){
						Lcd_set_pos(0, ++z);
						k += 15;
					}
				}
				LcdCharacter(0x7f);
			} else
				while (inputString[il] != 0x00) {
					LcdCharacter(inputString[il]);
					if (++il > k){
						Lcd_set_pos(0, ++z);
						k += 15;
					}
				}
			break;
		}
		case '5': {				  // ���������� ����������� �������
			edit_time = 1;
			s10 = inputString[1] & 0x0f;
			s1 = inputString[2] & 0x0f;
			m10 = inputString[3] & 0x0f;
			m1 = inputString[4] & 0x0f;
			h10 = inputString[5] & 0x0f;
			h1 = inputString[6] & 0x0f;
			dw = (inputString[7] & 0x0f) + 1 ;
			d10 = inputString[8] & 0x0f;
			d1 = inputString[9] & 0x0f;
			mo10 = inputString[10] & 0x0f;
			mo1 = inputString[11] & 0x0f;
			ye10 = inputString[14] & 0x0f;
			ye1 = inputString[15] & 0x0f;
			set_time = 1;
			break;
		}
		case '6':			// reinit LCD
		{
			LcdWrite(LCD_C, 0xAE); 								// Display ON
			LcdWrite(LCD_C, 0xE2); 								// ����� ����������
			LcdWrite(LCD_C, 0x3D); 								// Charge pump ON
			LcdWrite(LCD_C, 0x01); 								// Charge pump=4
			LcdWrite(LCD_C, 0xA4); 								//
			LcdWrite(LCD_C, 0x2F); 								//
			LcdWrite(LCD_C, 0xC0); 								// ���������� ����-���
			LcdWrite(LCD_C, 0xA0); 								// ���������� ����-�����
			__delay_cycles(800000);
			LcdWrite(LCD_C, 0xAF); 								// Display ON
			lcd_contrast(contrast);
			lcd_show_main();
			break;
		}
		default: {
			break;
		}
	}
	if (!multiscreen)
		for (il = 313; il > 0; il--)
			inputString[il] = 0;
}

//
//������ � uart ��� bt
//
void uart_puts_bt(char const* s) { 		// ��������� ������
	while (*s)
		uart_tx_bt(*s++);
}

void uart_tx_bt(char c) {				// ��������� ������
	while (TA1CCTL1 & CCIE)
		;
	tx_data_bt = (c | 0xFF00);
	__disable_interrupt();
	TA1CCR1 = TA1R + BT_HALF_BT;
	TA1CCTL1 = OUTMOD_5 + CCIE;
	__enable_interrupt();
}
//
//������ � uart ��� bt
//

//
//������ � �������
//
void LcdCharacter(char character) {							// ����������� �������
	unsigned char tmp = 0x20;
	unsigned char index;

	if ((character & 0xff) >= 0xc0) {						// ���� ��������
		tmp = 0x60;
	}
	if ((character & 0xff) == 0xb8) {						// ���� �
		character = 0xe5;
		tmp = 0x60;
	}
	if ((character & 0xff) == 0xa8) {						// ���� �
		character = 0xc5;
		tmp = 0x60;
	}
	if (black_text) {										// ����� �� ������ ����
		LcdWrite(LCD_D, 0xff);
		for (index = 5; index > 0; index--)
			LcdWrite(LCD_D, ~ASCII[(character & 0xff) - tmp][5 - index]);	// ����������� ������ �������
	} else {
		LcdWrite(LCD_D, 0x00);
		for (index = 5; index > 0; index--)
			LcdWrite(LCD_D, ASCII[(character & 0xff) - tmp][5 - index]);	// ����������� ������ �������
	}
}

void LcdClear(void) {									// ������� ������
	unsigned int index = 0;

	for (index = 864; index > 0; index--)
		LcdWrite(LCD_D, 0x00);
}

void clear_1() {								// ������� �������� ���� ������
	unsigned int index = 0;

	Lcd_set_pos(0, 1);
	for (index = 864; index > 95; index--)
		LcdWrite(LCD_D, 0x00);
}

void Lcd_set_pos(unsigned char c, unsigned char r) {// ���������� ������� �� ������ c x r � ��������
	c = c * 6;
	Lcd_set_pos_pix(c, r);
}

void Lcd_set_pos_pix(unsigned char c, unsigned char r) {// ���������� ������� �� ������ c x r � ��������
	LcdWrite(LCD_C, 0xB0 | (r & 0x0F));					// Page address set
	LcdWrite(LCD_C, 0x10 | ((c >> 4) & 0x07));			// Column address set Upper bit address
	LcdWrite(LCD_C, 0x00 | (c & 0x0F));					// Column address set Lower bit address
}

void lcd_contrast(unsigned char contrast2) {			// ��������� ���������
	contrast2 = contrast2 & 0x1F;
	LcdWrite(LCD_C, 0x80 + contrast2); 					// ������������� (0-31)
}

void lcd_dig(unsigned char num, unsigned char pos_x, unsigned char pos_y) { // ������� ����� ��� �����
	unsigned char l = 0;
	unsigned char z;
	unsigned char k;
	unsigned char index;

	for (z = 4; z > 0; z--) {
		Lcd_set_pos_pix(pos_x, pos_y + (4 - z) + 2);
		for (index = 9; index > 0; index--)
			if (index == 5) {
				for (k = 9; k > 0; k--)
					LcdWrite(LCD_D, (dig_9x32[num][9 - index] >> l) & 0xff);
			} else
				LcdWrite(LCD_D, (dig_9x32[num][9 - index] >> l) & 0xff);
		l += 8;
	}
}
void lcd_dig_clr(unsigned char num, unsigned char pos_x, unsigned char pos_y) { 	// ��������� ��� �����
	unsigned char l = 0;
	unsigned char z;
	unsigned char index;

	for (z = 4; z > 0; z--) {
		Lcd_set_pos_pix(pos_x, pos_y + (4 - z) + 2);
		index = 0;
		for (index = num; index > 0; index--)
			LcdWrite(LCD_D, 0x00);
		l += 8;
	}
}

void lcd_dot(unsigned char num, unsigned char pos_x, unsigned char pos_y) { 	// ��������� ��� �����
	unsigned char l = 0;
	unsigned char z;
	unsigned char index;

	for (z = 4; z > 0; z--) {
		Lcd_set_pos_pix(pos_x, pos_y + (4 - z) + 2);
		index = 0;
		for (index = 4; index > 0; index--)
			LcdWrite(LCD_D, (dot[num][4 - index] >> l) & 0xff);
		l += 8;
	}
}

void LcdWrite(unsigned char dc, unsigned char data) { // ����������� ���������� spi
	if (dc == 1)
		BIT_SET(P1OUT, PIN_SDIN);
	else
		BIT_CLR(P1OUT, PIN_SDIN);
	BIT_CLR(P1OUT, PIN_SCE);
	BIT_SET(P3OUT, PIN_SCLK);
	BIT_CLR(P3OUT, PIN_SCLK);
	char i = 0;
	while (i < 8) {
		if (data & (0x80 >> i++))
			BIT_SET(P1OUT, PIN_SDIN);
		else
			BIT_CLR(P1OUT, PIN_SDIN);
		BIT_SET(P3OUT, PIN_SCLK);
		BIT_CLR(P3OUT, PIN_SCLK);
	}
	BIT_SET(P1OUT, PIN_SCE);
}

void LcdString(char *characters) {						// ������� ������ �� �����
	while (*characters)
		LcdCharacter(*characters++);
}

void lcd_show_sms(unsigned char a) {					// ������� ���� ���
	unsigned char k;

	Lcd_set_pos_pix(36, 0);
	for (k = 9; k > 0; k--)
		if (a)
			LcdWrite(LCD_D, sms_ico[0][9 - k]);
		else
			LcdWrite(LCD_D, 0x00);
}

void lcd_show_call(unsigned char a) {					// ������� ���� ������
	unsigned char k;

	Lcd_set_pos_pix(49, 0);
	for (k = 12; k > 0; k--)
		if (a)
			LcdWrite(LCD_D, call_ico[0][12 - k]);
		else
			LcdWrite(LCD_D, 0x00);
}

void lcd_show_bt(unsigned char a) {						// ������� ���� bt
	unsigned char k;

	Lcd_set_pos_pix(63, 0);
	switch (a) {
	case 0:							// bt ��������
		for (k = 10; k > 0; k--)
			LcdWrite(LCD_D, 0x00);
		break;
	case 1:							// bt �������
		for (k = 10; k > 0; k--)
			LcdWrite(LCD_D, bt_ico[0][10 - k]);
		break;
	case 2:							// bt ������� � ���������
		for (k = 10; k > 0; k--)
			LcdWrite(LCD_D, bt_ico[1][10 - k]);
		break;
	}
}

void lcd_show_bat(unsigned char proc) {					// ������� ���� �������
	unsigned char k;

	Lcd_set_pos_pix(76, 0);
	for (k = 18; k > 0; k--)
		LcdWrite(LCD_D, bat_ico[proc][18 - k]);
}

void lcd_set_time_big() { // ����������� ������� �� ��������� ������
	Lcd_set_pos(1, 2);
	LcdCharacter(dofw[dw-1][0]);
	LcdCharacter(dofw[dw-1][1]);
	LcdCharacter(' ');
	LcdCharacter(' ');
	LcdCharacter(d10 + 0x30);
	LcdCharacter(d1 + 0x30);
	LcdCharacter('/');
	LcdCharacter(mo10 + 0x30);
	LcdCharacter(mo1 + 0x30);
	LcdCharacter('/');
	LcdCharacter('2');
	LcdCharacter('0');
	LcdCharacter(ye10 + 0x30);
	LcdCharacter(ye1 + 0x30);
	lcd_dig_clr(6,0,2);
	lcd_dig(h10, 6, 2);
	lcd_dig_clr(3,23,2);
	lcd_dig(h1, 26, 2);
	lcd_dig_clr(3,43,2);
	lcd_dot(dot_show, 46, 2);
	lcd_dig_clr(3,50,2);
	lcd_dig(m10, 53, 2);
	lcd_dig_clr(3,70,2);
	lcd_dig(m1, 73, 2);

}

void lcd_set_time_small() { // ����������� ������� � ����
	unsigned char bukva;

	Lcd_set_pos(0, 0);
	bukva = h10 + 0x30;
	LcdCharacter(bukva);
	bukva = h1 + 0x30;
	LcdCharacter(bukva);
	LcdString(":");
	bukva = m10 + 0x30;
	LcdCharacter(bukva);
	bukva = m1 + 0x30;
	LcdCharacter(bukva);
	if (current_screen == 0)
		lcd_set_time_big();
}

void lcd_show_main() {		// ������� ����� - ����
	current_screen = 0;
	LcdClear();
	lcd_show_sms(0);
	lcd_show_call(0);
	if (bt_on)
		if (bt_connect)
			lcd_show_bt(2);
		else
			lcd_show_bt(1);
	else
		lcd_show_bt(0);
	lcd_show_bat(bat_sost);
	lcd_set_time_small();
	call_true = 0;
}
//
//������ � �������
//

// i2c
void get_time_from_rtc(void) {
	TI_USCI_I2C_receiveinit(Slave_Address, 80);
	TI_USCI_I2C_receive(16, RX);
	__delay_cycles(80000);
	TI_USCI_I2C_receiveinit(Slave_Address, 80);
	TI_USCI_I2C_receive(16, RX);
	__delay_cycles(80000);
	TI_USCI_I2C_receiveinit(Slave_Address, 80);
	TI_USCI_I2C_receive(16, RX);					// ������ ������ � ���
	if ((RX[6] & 0xff) != 0x00) {					// ��� ������ ��� ������ �� ���
		s10 = (RX[0] & 0x70) >> 4;
		s1 = (RX[0] & 0x0f);
		m10 = (RX[1] & 0x70) >> 4;
		m1 = (RX[1] & 0x0f);
		h10 = (RX[2] & 0x30) >> 4;
		h1 = (RX[2] & 0x0f);
		dw = (RX[3] & 0x07);
		d10 = (RX[4] & 0x30) >> 4;
		d1 = (RX[4] & 0x0f);
		mo10 = (RX[5] & 0x10) >> 4;
		mo1 = (RX[5] & 0x0f);
		ye10 = (RX[6] & 0xf0) >> 4;
		ye1 = (RX[6] & 0x0f);
		char *Flash_ptrC;							// ������ ������ �� FLASH
		Flash_ptrC = (char *) 0x1040;             	// Point to beginning of seg C
		FCTL2 = FWKEY + FSSEL_1 + FN1;      		// MCLK/3 for Flash Timing Generator
		FCTL1 = FWKEY + ERASE;                    	// Set Erase bit
		FCTL3 = FWKEY ;                    			// Clear LOCK
		*Flash_ptrC = 0x00;                  		// Dummy write to erase Flash seg C
		FCTL1 = FWKEY + WRT;                  		// Set WRT bit for write operation
		Flash_ptrC = (char *) 0x1040;        		// Point to beginning
		*Flash_ptrC++ = s10;	//0x1040
		*Flash_ptrC++ = s1;		//0x1041
		*Flash_ptrC++ = m10;	//0x1042
		*Flash_ptrC++ = m1;		//0x1043
		*Flash_ptrC++ = h10;	//0x1044
		*Flash_ptrC++ = h1;		//0x1045
		*Flash_ptrC++ = dw;		//0x1046
		*Flash_ptrC++ = d10;	//0x1047
		*Flash_ptrC++ = d1;		//0x1048
		*Flash_ptrC++ = mo10;	//0x1049
		*Flash_ptrC++ = mo1;	//0x104a
		*Flash_ptrC++ = ye10;	//0x104b
		*Flash_ptrC++ = ye1;	//0x104c
		*Flash_ptrC++ = contrast;					//0x104d
		*Flash_ptrC++ = (pwm_width & 0xff00) >> 8;	//0x104e
		*Flash_ptrC++ = (timer_off & 0xff00) >> 8;	//0x104f
		*Flash_ptrC++ = (timer_off & 0x00ff);		//0x1050
		FCTL1 = FWKEY;                            	// Clear WRT bit
		FCTL3 = FWKEY + LOCK;             			// Set LOCK
	} else {
		char *a;								// ������ ������ �� FLASH
		a = (char*) 0x1040;
		s10 = *a++;
		s1 = *a++;
		m10 = *a++;
		m1 = *a++;
		h10 = *a++;
		h1 = *a++;
		dw = *a++;
		d10 = *a++;
		d1 = *a++;
		mo10 = *a++;
		mo1 = *a++;
		ye10 = *a++;
		ye1 = *a++;
	}
	lcd_set_time_small();
}

void set_time_to_rtc(void) {			// ������� ������ � ���
	unsigned int counter = 0;

	TX[0] = 0x00; // �����
	TX[1] = ((s10 & 0x07) << 4) | (s1 & 0x0f); //���
	TX[2] = ((m10 & 0x07) << 4) | (m1 & 0x0f); //���
	TX[3] = ((h10 & 0x03) << 4) | (h1 & 0x0f); //���
	TX[4] = (dw & 0x07); //���� ������
	TX[5] = ((d10 & 0x03) << 4) | (d1 & 0x0f); // ����
	TX[6] = ((mo10 & 0x01) << 4) | (mo1 & 0x0f); // �����
	TX[7] = ((ye10 & 0x0f) << 4) | (ye1 & 0x0f); // ���
	TX[8] = 0x00; //�1
	TX[9] = 0x00; //�1
	TX[10] = 0x00; //�1
	TX[11] = 0x00; //�1
	TX[12] = 0x00; //�2
	TX[13] = 0x00; //�2
	TX[14] = 0x00; //�2
	TX[15] = 0x00; //00 - 1��
	TX[16] = 0x00; // 00 -��� ��
	TI_USCI_I2C_transmitinit(Slave_Address, 80);
	while ((TI_USCI_I2C_notready()) && (counter++ < 65500))
		;
	TI_USCI_I2C_transmit(17, TX);
}
// i2c

// ���� ������
void down_sub_menu(void) {				// ���������� � �������, ������ ����
	switch (current_menu_item) {
	case 1:	// �����
		switch (current_sub_menu_item) {
		case 0:
			if (--h10 < 0)
				h10 = 2;
			break;
		case 1:
			if (--h1 < 0)
				if (h10 > 1)
					h1 = 4;
			if (h1 < 0)
				h1 = 9;
			break;
		case 2:
			if (--m10 < 0)
				m10 = 5;
			break;
		case 3:
			if (--m1 < 0)
				m1 = 9;
			break;
		case 4:
			if (--s10 < 0)
				s10 = 5;
			break;
		case 5:
			if (--s1 < 0)
				s1 = 9;
			break;
		}
		menu_setting(current_menu_item);
		break;
	case 2:	// ���������
		if (pwm_width == 0x0000)
			pwm_width = 0x0fff;
		else
			if (pwm_width == 0x01ff)
				pwm_width = 0x0000;
			else
				pwm_width -= 0x0100;
		TACCR2 = pwm_width;
		menu_setting(current_menu_item);
		break;
	case 3:	// ��������
		contrast--;
		if (contrast < 0)
			contrast = 31;
		lcd_contrast(contrast);
		menu_setting(current_menu_item);
		break;
	case 4:	// ����� ���������� ���������
		timer_off -= 32;
		if (timer_off < 16)
			timer_off = 16;
		menu_setting(current_menu_item);
		break;
	case 6:	// ����
		switch (current_sub_menu_item) {
		case 0:
			if (--dw < 1)
				dw = 7;
			break;
		case 1:
			if (--d10 < 0)
				d10 = 3;
			if ((mo10 == 0) && (mo1 == 2))
				if (d10 == 3)
					d10 = 2;
			break;
		case 2:
			d1--;
			if ((mo10 == 0) && (mo1 == 2) && (d10 == 2))
				if ((ye10 * 10 + ye1) & (0x03) == 0x00) {		//����������
					if (d1 < 0)
						d1 = 9;
				} else {
					if (d1 < 0)
						d1 = 8;
				}
			if (d10 == 3) {
				if (mo10 == 0)
					if ((mo1 == 1) || (mo1 == 3) || (mo1 == 5) || (mo1 == 7)
							|| (mo1 == 8)) {
						if (d1 < 0)
							d1 = 1;
					} else {
						d1 = 0;
					}
				if (mo10 == 1)
					if ((mo1 == 0) || (mo1 == 2)) {
						if (d1 < 0)
							d1 = 1;
					} else {
						d1 = 0;
					}
			}
			if (d1 < 0)
				d1 = 9;
			break;
		case 3:
			if (--mo10 < 0)
				mo10 = 1;
			break;
		case 4:
			mo1--;
			if (mo10 == 1)
				if (mo1 < 0)
					mo1 = 2;
			if (mo10 == 0)
				if (mo1 < 0)
					mo1 = 9;
			break;
		case 5:
			if (--ye10 < 0)
				ye10 = 9;
			break;
		case 6:
			if (--ye1 < 0)
				ye1 = 9;
			break;
		}
		menu_setting(current_menu_item);
		break;
	}

}

void up_sub_menu(void) {					// ���������� � �������, ������ �����
	switch (current_menu_item) {
	case 1:		// �����
		switch (current_sub_menu_item) {
		case 0:
			if (++h10 > 2)
				h10 = 0;
			break;
		case 1:
			if (++h1 > 3)
				if (h10 > 1)
					h1 = 0;
			if (h1 > 9)
				h1 = 0;
			break;
		case 2:
			if (++m10 > 5)
				m10 = 0;
			break;
		case 3:
			if (++m1 > 9)
				m1 = 0;
			break;
		case 4:
			if (++s10 > 5)
				s10 = 0;
			break;
		case 5:
			if (++s1 > 9)
				s1 = 0;
			break;
		}
		menu_setting(current_menu_item);
		break;
	case 2:	// ���������
		if (pwm_width == 0x0000)
			pwm_width = 0x00ff;
		pwm_width += 0x0100;
		if (pwm_width > 0x0fff)
			pwm_width = 0x0000;
		TACCR2 = pwm_width;
		menu_setting(current_menu_item);
		break;
	case 3:	// ��������
		contrast++;
		if (contrast > 31)
			contrast = 0;
		lcd_contrast(contrast);
		menu_setting(current_menu_item);
		break;
	case 4:	// ����� ���������� ���������
		timer_off += 32;
		if (timer_off > 1584)
			timer_off = 1584;
		menu_setting(current_menu_item);
		break;
	case 6:	// ����
		switch (current_sub_menu_item) {
		case 0:
			if (++dw > 7)
				dw = 1;
			break;
		case 1:
			if (++d10 > 3)
				d10 = 0;
			if ((mo10 == 0) && (mo1 == 2))
				if (d10 > 2)
					d10 = 0;
			break;
		case 2:
			d1++;
			if ((mo10 == 0) && (mo1 == 2) && (d10 == 2))
				if ((ye10 * 10 + ye1) & (0x03) == 0x00) {		//����������
					if (d1 > 9)
						d1 = 0;
				} else {
					if (d1 > 8)
						d1 = 0;
				}
			if (d10 == 3) {
				if (mo10 == 0)
					if ((mo1 == 1) || (mo1 == 3) || (mo1 == 5) || (mo1 == 7)
							|| (mo1 == 8)) {
						if (d1 > 1)
							d1 = 0;
					} else {
						d1 = 0;
					}
				if (mo10 == 1)
					if ((mo1 == 0) || (mo1 == 2)) {
						if (d1 > 1)
							d1 = 0;
					} else {
						d1 = 0;
					}
			}
			if (d1 > 9)
				d1 = 0;
			break;
		case 3:
			if (++mo10 > 1)
				mo10 = 0;
			break;
		case 4:
			mo1++;
			if (mo10 == 1)
				if (mo1 > 2)
					mo1 = 0;
			if (mo10 == 0)
				if (mo1 > 9)
					mo1 = 0;
			break;
		case 5:
			if (++ye10 > 9)
				ye10 = 0;
			break;
		case 6:
			if (++ye1 > 9)
				ye1 = 0;
			break;
		}
		menu_setting(current_menu_item);
		break;
	}
}

void menu_setting(unsigned char submenu) {						// ����������� �� ����
	unsigned int stroka;

	switch (submenu) {
	case 0:												// ���� ����
		clear_1();
		current_screen = 2;
		for (stroka = 8; stroka > 1; stroka--) {
			Lcd_set_pos(0, 9 - stroka);
			if (current_menu_item == 9 - stroka) {
				black_text = 1;
				LcdString((char*) Main_menu[8 - stroka]);
				black_text = 0;
			} else
				LcdString((char*) Main_menu[8 - stroka]);
		}
		break;
	case 1:												// �����
		current_screen = 3;
		if (current_sub_menu_item == 6) {
			set_time = 1;
			menu_setting(0);
		} else {
			edit_time = 1;
			set_time = 0;
			get_time = 0;
			clear_1();
			switch (current_sub_menu_item) {
			case 0:
				Lcd_set_pos(4, 3);
				LcdCharacter('_');
				break;
			case 1:
				Lcd_set_pos(5, 3);
				LcdCharacter('_');
				break;
			case 2:
				Lcd_set_pos(7, 3);
				LcdCharacter('_');
				break;
			case 3:
				Lcd_set_pos(8, 3);
				LcdCharacter('_');
				break;
			case 4:
				Lcd_set_pos(10, 3);
				LcdCharacter('_');
				break;
			case 5:
				Lcd_set_pos(11, 3);
				LcdCharacter('_');
				break;
			}
			Lcd_set_pos(4, 4);
			LcdCharacter(h10 + 0x30);
			LcdCharacter(h1 + 0x30);
			LcdCharacter(':');
			LcdCharacter(m10 + 0x30);
			LcdCharacter(m1 + 0x30);
			LcdCharacter(':');
			LcdCharacter(s10 + 0x30);
			LcdCharacter(s1 + 0x30);
		}
		break;
	case 2:												// �������
		clear_1();
		current_screen = 3;
		if (current_sub_menu_item == 1)
			menu_setting(0);
		else {
			Lcd_set_pos(8, 4);
			unsigned char a;
			a = (pwm_width >> 8) & 0x0F;
			if (a < 10)
				LcdCharacter(a + 0x30);
			else {
				Lcd_set_pos(7, 4);
				LcdCharacter(1 + 0x30);
				LcdCharacter(a - 10 + 0x30);
			}
		}
		break;
	case 3:												// �������������
		clear_1();
		current_screen = 3;
		if (current_sub_menu_item == 1)
			menu_setting(0);
		else {
			Lcd_set_pos(8, 4);
			if (contrast < 10)
				LcdCharacter(contrast + 0x30);
			else {
				Lcd_set_pos(7, 4);
				if (contrast < 20) {
					LcdCharacter(1 + 0x30);
					LcdCharacter(contrast - 10 + 0x30);
				} else if (contrast < 30) {
					LcdCharacter(2 + 0x30);
					LcdCharacter(contrast - 20 + 0x30);
				} else {
					LcdCharacter(3 + 0x30);
					LcdCharacter(contrast - 30 + 0x30);
				}
			}
		}
		break;
	case 4:												// ����� ����
		clear_1();
		current_screen = 3;
		if (current_sub_menu_item == 1)
			menu_setting(0);
		else {
			Lcd_set_pos(8, 4);
			unsigned char a;
			unsigned int d10=0;
			a = (timer_off >> 4) & 0xFF;
			if (a < 10)
				LcdCharacter(a + 0x30);
			else {
				Lcd_set_pos(7, 4);
				while (a>10){
					d10++;
					a -= 10;
				}
				LcdCharacter(d10 + 0x30);
				LcdCharacter(a + 0x30);
			}
		}
		break;
	case 5:												// �� ����
		if (BIT_TEST(P2OUT, BT_PWR)) {
			BIT_CLR(P2OUT, BT_PWR);
			BIT_CLR(P3OUT, BT_PWR);
			lcd_show_bt(0);
			bt_on = 0;
			bt_connect = 0;
		} else {
			BIT_SET(P2OUT, BT_PWR);
			BIT_SET(P3OUT, BT_PWR);
			lcd_show_bt(1);
			bt_on = 1;
		}
		break;
	case 6:												// ����
		current_screen = 3;
		if (current_sub_menu_item == 7) {
			set_time = 1;
			menu_setting(0);
		} else {
			edit_time = 1;
			set_time = 0;
			get_time = 0;
			clear_1();
			switch (current_sub_menu_item) {
			case 0:
				Lcd_set_pos(1, 3);
				LcdCharacter('_');
				LcdCharacter('_');
				break;
			case 1:
				Lcd_set_pos(5, 3);
				LcdCharacter('_');
				break;
			case 2:
				Lcd_set_pos(6, 3);
				LcdCharacter('_');
				break;
			case 3:
				Lcd_set_pos(8, 3);
				LcdCharacter('_');
				break;
			case 4:
				Lcd_set_pos(9, 3);
				LcdCharacter('_');
				break;
			case 5:
				Lcd_set_pos(13, 3);
				LcdCharacter('_');
				break;
			case 6:
				Lcd_set_pos(14, 3);
				LcdCharacter('_');
				break;
			}
			Lcd_set_pos(1, 4);
			LcdCharacter(dofw[dw-1][0]);
			LcdCharacter(dofw[dw-1][1]);
			LcdCharacter(' ');
			LcdCharacter(' ');
			LcdCharacter(d10 + 0x30);
			LcdCharacter(d1 + 0x30);
			LcdCharacter('/');
			LcdCharacter(mo10 + 0x30);
			LcdCharacter(mo1 + 0x30);
			LcdCharacter('/');
			LcdCharacter('2');
			LcdCharacter('0');
			LcdCharacter(ye10 + 0x30);
			LcdCharacter(ye1 + 0x30);
		}
		break;
	case 7:												// ����� � ��������
		time_from_phone = 1;
		break;
	}
}
// ���� ������
