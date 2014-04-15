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
#define PIN_LED 				BIT0						// P3.0  подсветка дисплея
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

#define max_menu_item 			7		// количество строк в меню

unsigned int 	tx_data_bt;           	// UART Tx data

unsigned char 	bat_sost = 0, 			// состояние батареи 0-100% - 4-10%
				current_menu_item = 1, 	// текущий пункт меню
				current_sub_menu_item = 0, // текущий пункт подменю
				screens = 0,			// количество экранов многоэкранного сообщения (максимум 3(счет с 0, значит 2))
				black_text = 0,			// инверсия шрифта
				current_screen = 0, 	// 0 - main watch, 1 - sms|call, 2 - setting_main, 3 - submenu, 4 - multiscreen text;
				char_count = 0,			// количество символов сообщения
				count_sec = 1,			// счетчик "секунд"
				count_sec2 = 1,			// счетчик "секунд"
				text_screen = 0; 		// текущий экран многоэкранного сообщения

int 			i_timer = 0; 			// счетчик таймера режима низкого энергопотребления

unsigned char	edit_time = 0, 			// редактируются настройки даты/времени?
				get_time = 0,			// получить время с ЧРВ и записать во FLASH или считать из FLASH?
				set_time = 0, 			// записать время на ЧРВ?
				check_akk = 0, 			// проверить состояние аккумулятора?
				v2_5 = 0,				// Vref для ADC10 - ref 2.5V?
				bt_on = 0, 				// BT включен?
				bt_connect = 0, 		// BT подключен?
				time_from_phone = 0,	// запросить время с телефона?
				multiscreen = 0,		// многоэкранное сообщение?
				dot_show = 0,			// отображать точки?
				P2_int = 0,				// прерывание от P2
				TA1_int = 0,			// прерывание от TA1
				ADC_int = 0,			// прерывание от ADC
				btn_pressed = 0,		// 0-ничего, 1,2,3 - up, cent, down; 4,5,6 - long up, cent, down;
				string_ready = 0,		//
				e_save = 0,				// LPM
				call_true = 0;			// входящий вызов

char inputString[314] = "";				// строка сообщения из UART

const unsigned char Main_menu[][17] =
			  { "Время           ",
				"Яркость         ",
				"Контрастность   ",
				"Время выключения",
				"Вкл / выкл bt   ",
				"Дата            ",
				"Время с телефона" }; 	// меню настроек

										// ВО FLASH'е

int 									// переменные часов и календаря
				h10 = 1, 	h1 = 7, 	// часы
				m10 = 1, 	m1 = 3, 	// минуты
				s10 = 5, 	s1 = 7, 	// секунды
				d10 = 1, 	d1 = 3, 	// день
				mo10 = 0,	mo1 = 3, 	// месяц
				ye10 = 1, 	ye1 = 4, 	// год
				dw = 4;					// день недели

int 			contrast = 24, 			// контрастность (0-31)
				pwm_width = 0x0eff, 	// яркость 0x00ff - 0x0fff;
				timer_off = 80;			// время до выключения

/* Forward references */
void init_watch(void);

void check_akkum(void);				// проверка состояния аккумулятора
void check_bluetooth(void);			// проверка состояния BT

void uart_tx_bt(char c);			// отправка символа по BT
void uart_puts_bt(char const* s);	// отправка строки по BT

void get_time_from_rtc(void);		// получение времени с ЧРВ и запись во FLASH
void set_time_to_rtc(void);			// запись времени на ЧРВ

void LcdCharacter(char character);						// вывести символ
void LcdClear(void);									// отчистить весь экран
void clear_1(void);										// отчистить экран, кроме статус бара
void Lcd_set_pos(unsigned char c, unsigned char r);		// установить позицию в символах на строку
void Lcd_set_pos_pix(unsigned char c, unsigned char r);	// установить позицию в пикселях на строку
void lcd_contrast(unsigned char contrast2);				// установить контрастность экрана
void lcd_dig(unsigned char num, unsigned char pos_x, unsigned char pos_y);	// цифры часов
void lcd_dot(unsigned char num, unsigned char pos_x, unsigned char pos_y);	// двоеточия часов
void LcdWrite(unsigned char dc, unsigned char data);	// отправка комманды/данных на экран
void LcdString(char *characters);						// вывести строку
void lcd_show_sms(unsigned char a);						// значек смс
void lcd_show_call(unsigned char a);					// значек звонка
void lcd_show_bt(unsigned char a);						// значек BT
void lcd_show_bat(unsigned char proc);					// значек аккумулятора
void lcd_set_time_big(void);							// вывести большие часы
void lcd_set_time_small(void);							// вывести малые часы (в статус баре)
void lcd_show_main(void);								// показать главный экран (дата и большие часы)

void menu_setting(unsigned char submenu);				// меню
void down_sub_menu(void);								// передвижение по меню
void up_sub_menu(void);									//		и изменения в подменю
void parse_string(void);								// разбор принятой строки
//
// прерывания
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

#pragma vector=TIMER1_A0_VECTOR						// прием данных по UART от BT
__interrupt void TIMER1_A0_isr(void) {
	static unsigned char rx_bitc1 = 8; 			 	// Rx bit count register
	static unsigned char rx_data1;      			// Rx data register

	BIT_CLR(IE1, WDTIE);
	BIT_CLR(TA1CCTL2, CCIE);
	BIT_CLR(ADC10CTL0, ADC10IE);					//отключение прерываний

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
			if ((char_count == 0) && ((rx_data1 < 0x31) || (rx_data1 > 0x36))) {	// отбрасываем мусор
			} else {
				if (rx_data1 == '\n')						// заменяем перенос строки на пробел
					rx_data1 = ' ';							// экономим место на экране
				inputString[char_count++] = rx_data1;
				if (rx_data1 == 0x00) {						// конец передаваемого сообщения
					BIT_SET(TACCTL2, OUTMOD_6);				// включение подсветки
					i_timer = 0;							// обнуление счетчика таймера режима низкого энергопотребления
					if (char_count > 105) {					// сообщение не поместится на один экран
						multiscreen = 1;
						text_screen = 0;
						screens = 1;
						if (char_count > 208)				// сообщение не поместится на два экрана
							screens = 2;
					} else
						multiscreen = 0;
					inputString[char_count++] = 0x00;
					inputString[char_count++] = 0x00;
					inputString[char_count++] = 0x00;
					string_ready = 1;
					BIT_SET(IE1, WDTIE);
					BIT_SET(TA1CCTL2, CCIE);
					BIT_SET(ADC10CTL0, ADC10IE);			// включение прерываний
					char_count = 0;
					e_save = 0;
					__bic_SR_register_on_exit(LPM3_bits);	// выход из режима энергосбережения
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
	case TA1IV_TACCR1:								// прередача данных по UART на BT
		if (tx_bitc1--) {
			BIT_CLR(IE1, WDTIE);
			BIT_CLR(TA1CCTL2, CCIE);
			BIT_CLR(ADC10CTL0, ADC10IE);			// выключение прерываний
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
			BIT_SET(ADC10CTL0, ADC10IE);			// включение прерываний
		}
		break;
	case TA1IV_TACCR2:								// счетчик таймера режима низкого энергопотребления
		TA1_int = 1;
		break;
	}
}

#pragma vector = WDT_VECTOR
__interrupt void WDT_isr(void) {					// прерывание WDT в режиме интервального таймера (проверка состояния)
	BIT_CLR(TA1CCTL2, CCIE);
	BIT_CLR(ADC10CTL0, ADC10IE);					// выключение прерываний
	if (e_save)
	{
		if (current_screen == 0){
			dot_show++;								// покажем что часы работают
			if (dot_show >1)						// моргаем точками
				dot_show = 0;
			lcd_dot(dot_show, 46, 2);
		}
		if (++count_sec2>19){						// ручной грубый подсчет времени
			count_sec2 = 1;							// в режиме энергосбережения
			if (m1<9)								// т.к. нет связи с ЧРВ
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
	check_akk = 1;									// проверка аккума
	if (!edit_time){
		get_time = 1;
	}												// обновим время
	if (bt_connect) {
		check_bluetooth();							// проверка BT
		if (bt_on == 0) {							// BT отключен
			e_save = 0;
			__bic_SR_register_on_exit(LPM3_bits);	// выход из реима LPM3
			BIT_SET(TACCTL2, OUTMOD_6);				// включение подсветки
		}
	}
	BIT_SET(TA1CCTL2, CCIE);
	BIT_SET(ADC10CTL0, ADC10IE);					// включение прерываний
}

#pragma vector=PORT2_VECTOR
__interrupt void P2_isr() // Обработчик прерывания
{
	if (P2IFG & BT_RXD) {					// по UART'у с BT идут данные в спящем режиме
		get_time = 0;
		set_time = 0;
		check_akk = 0;
		ADC_int = 0;
		P2_int = 0;							// убираем все проверки и обработчики прерываний
		BIT_CLR(IE1, WDTIE);
		BIT_CLR(TA1CCTL2, CCIE);
		BIT_CLR(ADC10CTL0, ADC10IE);		// выключение прерываний
		BIT_CLR(P2IE, BT_RXD);
		BIT_CLR(P2IFG, BT_RXD);
		BIT_SET(P2SEL, BT_RXD);				// настраиваем пин BT_RXD на вход таймера
		TA1CCR0 = TA1R;
	} else {								// прерывание от нажатия кнопок
		BIT_CLR(P2IE, B_CENT|B_UP|B_DOWN);
		BIT_CLR(IE1, WDTIE);
		BIT_CLR(TA1CCTL2, CCIE);
		BIT_CLR(ADC10CTL0, ADC10IE);		// выключение прерываний
		BIT_CLR(P2IE, BT_RXD);
		BIT_CLR(P2IFG, BT_RXD);
		BIT_SET(P2SEL, BT_RXD);				// настраиваем пин BT_RXD на вход таймера
		char long_press = 1;
		unsigned long a;
		BIT_SET(TACCTL2, OUTMOD_6);			// включение подсветки и диспеля
		i_timer = 0;  						// обнуление счетчика
		if (P2IFG & B_CENT) { 				// прерывание от B_CENT
			for (a = 200000; a > 0; a--)	// антидребезг и проверка длинного нажатия
				if (P2IN & B_CENT)
					long_press = 0;
			if (long_press)
				btn_pressed = 5;
			else
				btn_pressed = 2;
		}
		if (P2IFG & B_DOWN) {  					// прерывание от B_DOWN
			for (a = 200000; a > 0; a--)		// антидребезг и проверка длинного нажатия
				if (P2IN & B_DOWN)
					long_press = 0;
			if (long_press)
				btn_pressed = 6;
			else
				btn_pressed = 3;
		}
		if (P2IFG & B_UP) {  							// прерывание от B_UP
			for (a = 200000; a > 0; a--)				// антидребезг и проверка длинного нажатия
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
	__bic_SR_register_on_exit(LPM3_bits);			// выход из LPM3

}

#pragma vector=ADC10_VECTOR							// прерывание готовности преобразования АЦП
__interrupt void ADC10_isr(void) {
	ADC_int = 1;
}
// прерывания

void check_akkum(void) {
	unsigned int adc;
	adc = ADC10MEM;
	if (v2_5) {						// отобразим состояние аккума на экране
		bat_sost = 4;
	} else {
		bat_sost = 4;			// константы вычисленные в результате наблюдений
		if (adc > 0x034e)
			bat_sost = 3;
		if (adc > 0x035a)
			bat_sost = 2;
		if (adc > 0x036f)
			bat_sost = 1;
		if (adc > 0x0389)
			bat_sost = 0;
	}
	if (adc >= 0x03ff) {			// при зарядке, напруга на аккуме через делитель больше 2,5 V
		v2_5 = 0;
	}
	if (adc < 0x030c) {				// при разряде, напруга на аккуме через делитель меньше 2,5 V
		v2_5 = 1;
	}
	lcd_show_bat(bat_sost);
}

void check_bluetooth(void) {		// проверка коннекта BT
	unsigned int i;
	char disconnect = 0;
	for (i = 65000; i > 0; i--)		// если BT соединен, то BT_LED лог. 1, иначе 10101010
		if (!(P3IN & BT_LED)) {		// ловим лог. 0, если появился, значит BT отсоединен
			disconnect = 1;
		}
	if (disconnect) {
		i_timer = 0;
		BIT_CLR(P2OUT, BT_PWR);
		BIT_CLR(P3OUT, BT_PWR);		// выключаем питание BT
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
		LcdString("    отсоеденен!");
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
		BIT_CLR(P3OUT, vibro);			// сигнализируем об отключенном BT
		BIT_SET(IE1, WDTIE);
		BIT_SET(TA1CCTL2, CCIE);
		BIT_SET(ADC10CTL0, ADC10IE);	// включение прерываний
		BIT_CLR(P2IE, BT_RXD);
		BIT_CLR(P2IFG, BT_RXD);
		BIT_SET(P2SEL, BT_RXD);			// BT_RXD как вход для таймера
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
		if (P2_int){		// прерывание от P2
			P2_int = 0;
			switch (btn_pressed) {
			case 0:
				break;
			case 1:		// up
				if (current_screen == 2) { 			// выбор строки  в меню
					current_menu_item--;
					if (current_menu_item <= 0)
						current_menu_item = max_menu_item;
					menu_setting(0);
				} else if (current_screen == 3)   			// изменение в подменю
					up_sub_menu();
				else if (current_screen == 4) {				// многоэкранное сообщение, показать предыдущий экран
					unsigned int l, k = 15, z = 1;
					if (text_screen - 1 > 0) {				// не первый экран
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
					} else if (text_screen - 1 == 0) {		// первый экран
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
				if (current_screen == 2) {		// меню -> подменю
					current_sub_menu_item = 0;
					menu_setting(current_menu_item);
				} else if (current_screen == 3) {	// подменю -> след. элемент
					current_sub_menu_item++;
					menu_setting(current_menu_item);
				}
				break;
			case 3:		// down
				if (current_screen == 2) { 	// перемешение по меню
					current_menu_item++;
					if (current_menu_item > max_menu_item)
						current_menu_item = 1;
					menu_setting(0);
				} else if (current_screen == 3)   	// изменение в подменю
					down_sub_menu();
				else if (current_screen == 4) {		// многоэкранное сообщение, показать следующий экран
					unsigned int l, k , z = 1, m = 15;
					if ((text_screen + 1) < screens) {			// не последний экран
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
					} else if ((text_screen + 1) == screens) {	// последний экран
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
				if (current_screen == 0)				// начальный экран -> меню
					menu_setting(0);
				else {									// любой экран -> начальный экран
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
				if (current_screen == 0)		// начальный экран -> меню
					menu_setting(0);
				else {							// любой экран -> начальный экран
					multiscreen = 0;
					unsigned int il;
					for (il = 313; il > 0; il--)
						inputString[il] = 0;
					lcd_show_main();
					edit_time = 0;
					get_time = 1;				// обновим время
				}
				break;
			case 6:		// long down
				if (call_true == 1){
					uart_tx_bt('2');
				}
				if (current_screen == 0)		// начальный экран -> меню
					menu_setting(0);
				else {							// любой экран -> начальный экран
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
		if (TA1_int){		// прерывание от TA1
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
				if (++i_timer == timer_off) {				// время до выключения экрана
					if ((bt_on == 1) && (bt_connect == 0)) {
						i_timer = 0;
					} else {
						BIT_CLR(ADC10CTL0, ADC10IE);
						i_timer = -1;
						BIT_CLR(TACCTL2, OUTMOD_6);			// отключаем подсветку
						BIT_CLR(TACCTL2, OUT);				// отключаем подсветку
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
						BIT_SET(P2IE, BT_RXD);				// настраиваем пин BT_RXD на прерывание (в режиме LPM3 SMCLK отключен, таймер не работает)
						e_save = 1;
						LPM3;
					}
				}
			}
		}
		if (ADC_int){		// прерывание от ADC
			ADC_int = 0;
			BIT_CLR(IE1, WDTIE);							// выключение прерываний
			BIT_CLR(TA1CCTL2, CCIE);
			check_akkum();									// соотносим показания АЦП с таблицей заряда аккума
			BIT_SET(IE1, WDTIE);
			BIT_SET(TA1CCTL2, CCIE);						// включение прерываний
		}
		if (set_time) {		// записать время в ЧРВ
			set_time_to_rtc();
			__delay_cycles(2000000);
			get_time = 1;
			edit_time = 0;
			set_time = 0;
		}
		if (get_time) {		// считать время из ЧРВ
			get_time_from_rtc();
			edit_time = 0;
			get_time = 0;
		}
		if (check_akk) {	// проверить сост. аккума
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
		if (time_from_phone) {// запросить время с телефона
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

	// сажаем BSL ноги на выход и лог. 0
	BIT_SET(P1DIR, TXD | RXD);
	BIT_CLR(P1OUT, TXD | RXD);

	// init LCD
	edit_time = 0;
	// берем инфу о настройках экрана из FLASH'а
	char *a;
	a = (char*) 0x104d;
	contrast = *a++;
	pwm_width = 0x00ff | (*a++) << 8;
	timer_off = ((*a++) & 0x00ff) << 8;
	timer_off |= ((*a++) & 0x00ff);
	// настройка портов для LCD
	BIT_SET(P1DIR, PIN_RESET + PIN_SCE + PIN_SDIN);
	BIT_SET(P3DIR, PIN_SCLK);
	BIT_CLR(P1OUT, PIN_RESET + PIN_SDIN);
	BIT_CLR(P3OUT, PIN_SCLK);
	BIT_SET(P1OUT, PIN_SCE);
	BIT_SET(P3DIR, PIN_LED);
	BIT_SET(P3OUT, PIN_LED);
	BIT_SET(P1OUT, PIN_RESET);
	// последовательность инициализации
	LcdWrite(LCD_C, 0xE2); 								// сброс програмный
	LcdWrite(LCD_C, 0x3D); 								// Charge pump ON
	LcdWrite(LCD_C, 0x01); 								// Charge pump=4
	LcdWrite(LCD_C, 0xA4); 								//
	LcdWrite(LCD_C, 0x2F); 								//
	//LcdWrite(LCD_C, 0xC8);							// зеркально верх-низ
	LcdWrite(LCD_C, 0xC0); 								// нормальное верх-низ
	//LcdWrite(LCD_C, 0xA1);							// зеркально лево-право
	LcdWrite(LCD_C, 0xA0); 								// нормальное лево-право
	__delay_cycles(800000);
	LcdWrite(LCD_C, 0xAF); 								// Display ON
	LcdClear();
	lcd_contrast(contrast);
	BIT_SET(P3SEL, PIN_LED);
	BIT_SET(P3DIR, PIN_LED);
	// таймер ШИМ для подсветки
	TACTL = TASSEL_2 + ID_0 + MC_1 + TACLR;		// SMCLK +  divider /1 + прямой счет до TACCR0 + counter clear
	TACCR0 = 0x0fff;									// Период ШИМ
	TACCR2 = pwm_width;									// заполнение ШИМ для подсветки
	TACCTL2 = OUTMOD_6;							// Выход ШИМ

	// init Bluetooth
	BIT_SET(P3DIR, BT_PWR);
	BIT_CLR(P3REN, BT_PWR);
	BIT_SET(P3OUT, BT_PWR);
	BIT_SET(P2DIR, BT_PWR);
	BIT_CLR(P2REN, BT_PWR);
	BIT_SET(P2OUT, BT_PWR);	//питание БТ ON
	BIT_CLR(P3DIR, BT_LED);	//состояние бт
	bt_on = 1;
	lcd_show_bt(1);
	//TimerA_1 для uart
	TA1CTL = TASSEL_2 + ID_3 + MC_2 + TAIE;	//TASSEL_2=SMCLK + input divider: 3 - /8 + mode control: 2 - Continous up + interrupt enable
	TA1CCTL1 = OUT; //Tx
	TA1CCTL0 = CM_2 + SCS + CAP + CCIE; //Rx
	//Порты Rx и TX для БТ
	BIT_SET(P2SEL, BT_TXD + BT_RXD);
	BIT_SET(P2DIR, BT_TXD);

	//таймер для выкл. экрана
	BIT_SET(TA1CCTL2, CCIE);

	// Настройка кнопок
	BIT_CLR(P2DIR, B_CENT|B_UP|B_DOWN);
	// направление пинов - IN
	BIT_SET(P2REN, B_CENT|B_UP|B_DOWN);
	// подключение резисторов
	BIT_SET(P2IE, B_CENT|B_UP|B_DOWN);
	// Разрешение прерываний
	BIT_SET(P2IES, B_CENT|B_UP|B_DOWN);
	// Прерывание происходит по 1/0 (отпусканию/нажатию)
	BIT_CLR(P2IFG, B_CENT|B_UP|B_DOWN);
	// Очистка флага прерываний

	// WDT+ как интервальный таймер, частота 12 КГц - ACLK - VLO
	WDTCTL = WDTPW + WDTTMSEL + WDTSSEL;
	BIT_SET(IE1, WDTIE);

	// vibro - выход
	BIT_SET(P3DIR, vibro);
	BIT_CLR(P3OUT, vibro);

	lcd_show_main();		// главный экран
	__enable_interrupt();
	edit_time = 0;
	get_time = 0;
	set_time = 0;
	get_time_from_rtc();	// обновляем время
	__delay_cycles(8000000);
	set_time_to_rtc();		// обновляем время

	// init ADC
	ADC10CTL0 = SREF_1 + ADC10SR + REF2_5V + ADC10SHT_2 + REFON + ADC10ON + ADC10IE;// включаем АЦП+ REFON
	ADC10CTL1 = INCH_0; //A0
	ADC10AE0 |= 0x01; // устанавливаем 0-ой пин как вход для АЦП
}


void parse_string(void) {					// разбор пришедшего сообщения
	unsigned int il;
	unsigned char z,k;

	switch (inputString[0]) {
		case '1': {						// индикация успешно подключившегося к BT телефона
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
		case '2': {						// индикация входящего смс
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
		case '3': {					// индикация входящего звонка
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
		case '4': {					// отображение присланного текста
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
		case '5': {				  // сохранение присланного времени
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
			LcdWrite(LCD_C, 0xE2); 								// сброс програмный
			LcdWrite(LCD_C, 0x3D); 								// Charge pump ON
			LcdWrite(LCD_C, 0x01); 								// Charge pump=4
			LcdWrite(LCD_C, 0xA4); 								//
			LcdWrite(LCD_C, 0x2F); 								//
			LcdWrite(LCD_C, 0xC0); 								// нормальное верх-низ
			LcdWrite(LCD_C, 0xA0); 								// нормальное лево-право
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
//работа с uart для bt
//
void uart_puts_bt(char const* s) { 		// отправить строку
	while (*s)
		uart_tx_bt(*s++);
}

void uart_tx_bt(char c) {				// отправить символ
	while (TA1CCTL1 & CCIE)
		;
	tx_data_bt = (c | 0xFF00);
	__disable_interrupt();
	TA1CCR1 = TA1R + BT_HALF_BT;
	TA1CCTL1 = OUTMOD_5 + CCIE;
	__enable_interrupt();
}
//
//работа с uart для bt
//

//
//работа с экраном
//
void LcdCharacter(char character) {							// отображение символа
	unsigned char tmp = 0x20;
	unsigned char index;

	if ((character & 0xff) >= 0xc0) {						// если кирилица
		tmp = 0x60;
	}
	if ((character & 0xff) == 0xb8) {						// если ё
		character = 0xe5;
		tmp = 0x60;
	}
	if ((character & 0xff) == 0xa8) {						// если Ё
		character = 0xc5;
		tmp = 0x60;
	}
	if (black_text) {										// текст на черном фоне
		LcdWrite(LCD_D, 0xff);
		for (index = 5; index > 0; index--)
			LcdWrite(LCD_D, ~ASCII[(character & 0xff) - tmp][5 - index]);	// попиксельно рисуем символы
	} else {
		LcdWrite(LCD_D, 0x00);
		for (index = 5; index > 0; index--)
			LcdWrite(LCD_D, ASCII[(character & 0xff) - tmp][5 - index]);	// попиксельно рисуем символы
	}
}

void LcdClear(void) {									// очистка экрана
	unsigned int index = 0;

	for (index = 864; index > 0; index--)
		LcdWrite(LCD_D, 0x00);
}

void clear_1() {								// очистка рабочего поля экрана
	unsigned int index = 0;

	Lcd_set_pos(0, 1);
	for (index = 864; index > 95; index--)
		LcdWrite(LCD_D, 0x00);
}

void Lcd_set_pos(unsigned char c, unsigned char r) {// Установить позицию на экране c x r в символах
	c = c * 6;
	Lcd_set_pos_pix(c, r);
}

void Lcd_set_pos_pix(unsigned char c, unsigned char r) {// Установить позицию на экране c x r в пикселях
	LcdWrite(LCD_C, 0xB0 | (r & 0x0F));					// Page address set
	LcdWrite(LCD_C, 0x10 | ((c >> 4) & 0x07));			// Column address set Upper bit address
	LcdWrite(LCD_C, 0x00 | (c & 0x0F));					// Column address set Lower bit address
}

void lcd_contrast(unsigned char contrast2) {			// установка контраста
	contrast2 = contrast2 & 0x1F;
	LcdWrite(LCD_C, 0x80 + contrast2); 					// контрастность (0-31)
}

void lcd_dig(unsigned char num, unsigned char pos_x, unsigned char pos_y) { // большие цифры для часов
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
void lcd_dig_clr(unsigned char num, unsigned char pos_x, unsigned char pos_y) { 	// двоеточие для часов
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

void lcd_dot(unsigned char num, unsigned char pos_x, unsigned char pos_y) { 	// двоеточие для часов
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

void LcdWrite(unsigned char dc, unsigned char data) { // программная реализация spi
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

void LcdString(char *characters) {						// вывести строку на экран
	while (*characters)
		LcdCharacter(*characters++);
}

void lcd_show_sms(unsigned char a) {					// вывести знак смс
	unsigned char k;

	Lcd_set_pos_pix(36, 0);
	for (k = 9; k > 0; k--)
		if (a)
			LcdWrite(LCD_D, sms_ico[0][9 - k]);
		else
			LcdWrite(LCD_D, 0x00);
}

void lcd_show_call(unsigned char a) {					// вывести знак звонка
	unsigned char k;

	Lcd_set_pos_pix(49, 0);
	for (k = 12; k > 0; k--)
		if (a)
			LcdWrite(LCD_D, call_ico[0][12 - k]);
		else
			LcdWrite(LCD_D, 0x00);
}

void lcd_show_bt(unsigned char a) {						// вывести знак bt
	unsigned char k;

	Lcd_set_pos_pix(63, 0);
	switch (a) {
	case 0:							// bt выключен
		for (k = 10; k > 0; k--)
			LcdWrite(LCD_D, 0x00);
		break;
	case 1:							// bt включен
		for (k = 10; k > 0; k--)
			LcdWrite(LCD_D, bt_ico[0][10 - k]);
		break;
	case 2:							// bt включен и подключен
		for (k = 10; k > 0; k--)
			LcdWrite(LCD_D, bt_ico[1][10 - k]);
		break;
	}
}

void lcd_show_bat(unsigned char proc) {					// вывести знак батареи
	unsigned char k;

	Lcd_set_pos_pix(76, 0);
	for (k = 18; k > 0; k--)
		LcdWrite(LCD_D, bat_ico[proc][18 - k]);
}

void lcd_set_time_big() { // отображение времени на начальном экране
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

void lcd_set_time_small() { // отображение времени в трее
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

void lcd_show_main() {		// главный экран - часы
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
//работа с экраном
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
	TI_USCI_I2C_receive(16, RX);					// чтение данных с ЧРВ
	if ((RX[6] & 0xff) != 0x00) {					// нет ошибки при чтении из ЧРВ
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
		char *Flash_ptrC;							// запись данных во FLASH
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
		char *a;								// чтение данных из FLASH
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

void set_time_to_rtc(void) {			// запишем данные в ЧРВ
	unsigned int counter = 0;

	TX[0] = 0x00; // адрес
	TX[1] = ((s10 & 0x07) << 4) | (s1 & 0x0f); //сек
	TX[2] = ((m10 & 0x07) << 4) | (m1 & 0x0f); //мин
	TX[3] = ((h10 & 0x03) << 4) | (h1 & 0x0f); //час
	TX[4] = (dw & 0x07); //день недели
	TX[5] = ((d10 & 0x03) << 4) | (d1 & 0x0f); // дата
	TX[6] = ((mo10 & 0x01) << 4) | (mo1 & 0x0f); // месяц
	TX[7] = ((ye10 & 0x0f) << 4) | (ye1 & 0x0f); // год
	TX[8] = 0x00; //а1
	TX[9] = 0x00; //а1
	TX[10] = 0x00; //а1
	TX[11] = 0x00; //а1
	TX[12] = 0x00; //а2
	TX[13] = 0x00; //а2
	TX[14] = 0x00; //а2
	TX[15] = 0x00; //00 - 1Гц
	TX[16] = 0x00; // 00 -все ок
	TI_USCI_I2C_transmitinit(Slave_Address, 80);
	while ((TI_USCI_I2C_notready()) && (counter++ < 65500))
		;
	TI_USCI_I2C_transmit(17, TX);
}
// i2c

// меню экрана
void down_sub_menu(void) {				// измменения в подменю, кнопка вниз
	switch (current_menu_item) {
	case 1:	// время
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
	case 2:	// подсветка
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
	case 3:	// контраст
		contrast--;
		if (contrast < 0)
			contrast = 31;
		lcd_contrast(contrast);
		menu_setting(current_menu_item);
		break;
	case 4:	// время выключения подсветки
		timer_off -= 32;
		if (timer_off < 16)
			timer_off = 16;
		menu_setting(current_menu_item);
		break;
	case 6:	// дата
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
				if ((ye10 * 10 + ye1) & (0x03) == 0x00) {		//высокосный
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

void up_sub_menu(void) {					// измменения в подменю, кнопка вверх
	switch (current_menu_item) {
	case 1:		// время
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
	case 2:	// подвсетка
		if (pwm_width == 0x0000)
			pwm_width = 0x00ff;
		pwm_width += 0x0100;
		if (pwm_width > 0x0fff)
			pwm_width = 0x0000;
		TACCR2 = pwm_width;
		menu_setting(current_menu_item);
		break;
	case 3:	// контраст
		contrast++;
		if (contrast > 31)
			contrast = 0;
		lcd_contrast(contrast);
		menu_setting(current_menu_item);
		break;
	case 4:	// время выключения подсветки
		timer_off += 32;
		if (timer_off > 1584)
			timer_off = 1584;
		menu_setting(current_menu_item);
		break;
	case 6:	// дата
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
				if ((ye10 * 10 + ye1) & (0x03) == 0x00) {		//высокосный
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

void menu_setting(unsigned char submenu) {						// перемешение по меню
	unsigned int stroka;

	switch (submenu) {
	case 0:												// само меню
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
	case 1:												// время
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
	case 2:												// яркость
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
	case 3:												// контрастность
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
	case 4:												// время выкл
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
	case 5:												// бт выкл
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
	case 6:												// дата
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
	case 7:												// время с телефона
		time_from_phone = 1;
		break;
	}
}
// меню экрана
