#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define CPU_F 16000000UL
#define LED_BIT  (1<<7)
#define BUTTON_PORT (1<<5)
#define ROTOR_PORTS (3<<0)
#define TXC0_BUFFER_SIZE 32 //должен быть обизательно 2 в степени n, циклический буфер
#define TXC0_BUFFER_MASK (TXC0_BUFFER_SIZE-1) // для 256-чарного буфера маска не нужна, но мы о ней помним

#define TWI_PORTS (3<<0)
#define ADD_R (1<<5)

volatile unsigned long int usec = 0; // переменная которая считает микросекунды в таймеры
volatile unsigned  int msec = 0; // переменная которая считает милисекунды в таймере
unsigned char flag_sec = 0; // Переменная в которой сохраняется выбор таймера

volatile unsigned char NewState = 0; // Переменная которые сохраняет состояния портов енкодера
volatile unsigned char OldState = 0; // если перенести в функцию encoder_read, для которой они локальны
// енкодер иногда перестает работать в одну сторону, почему непонятно

volatile unsigned char txc0buf_read_point = 0; // точка считывания из буфера
volatile unsigned char txc0buf_write_point = 0; // точка записи в буфер
char buffer_txc0[TXC0_BUFFER_SIZE]; // циклический буфер на отправку
volatile unsigned char txc0_udr0_statte = 0; // состояние буфера на запись равно 0, запрещено 1

volatile char encoder_state = 0; // Переменная которая хранит флаг изменения состояния енкодера, используется для отправки данных в усарт
unsigned char state_led = 0; // Переменная которая сохраняет состояния леда
volatile unsigned char brightness = 0; // можно обойтись и без этой переменной и использовать только OCR0A, но так понятней
unsigned char led_intensity = 0; // тут храниться яккость диода в чаре
char str_bri[] = "Brightness = 000%\n\r";  // тут храниться яркость диода в строке

char twi_status = 0;
char led_int_High_Byte = 0; // data from light sensor
char led_int_Low_Byte = 0; // data from light sensor
volatile char st_signals_count = 0; // flag for vorking with mistakes
volatile char twi_bri_error = 0;
volatile char twi_device = 0; // twi device flag


//-------------my_delay_block----------------------------

void timer_on_us(void){
	/*Включаем 16 битный таймер 1*/
	TCCR1B |= (1<<WGM12); //  режим СТС для таймера
	TCCR1B |= (1<<CS11);  //  делитель на 8 нужен чтобы правильно подобрать время
	TIMSK1 |= (1<<OCIE1A); // включаем режим сравнения TCCN1 c OCR1A
	OCR1AH = 0;
	OCR1AL = 2;
	/*16MГц делим на делитель 8 и делим на 2 (записано в OCR1AH и OCR1AL) получаем 1000000 Гц
	тоесть время которое потрачено на один цикл 0,000001 сек = 1 микросек, приблизительно конечно*/
}

void timer_on_ms(void){
	/*Включаем 16 битный таймер 1*/
	TCCR1B |= (1<<WGM12); //  режим СТС для таймера
	TCCR1B |= (1<<CS11);  //  делитель на 8 нужен чтобы правильно подобрать время
	TIMSK1 |= (1<<OCIE1A); // включаем режим сравнения TCCN1 c OCR1A
	OCR1AH = 0b00000111;
	OCR1AL = 0b11010000;
	/*16MГц делим на делитель 8 и делим на 2000 (записано в OCR1AH и OCR1AL) получаем 1000000 Гц
	тоесть время которое потрачено на один цикл 0,001 сек = 1 милиосек, приблизительно конечно*/
}

void timer_of(void){
	/* Виключаем таймер, зачем ему лишний раз тикать и кидать прерывания если он не нужен*/
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<CS11);
	TIMSK1 &= ~(1<<OCIE1A);
	OCR1AH = 0;
	OCR1AL = 0;
}

void my_delay_us(volatile unsigned long int usec1){
	/* Мой делей, каждый цикл занимает 1 микросекунду, eсли включены другие интрапты работает очень нестабильно по времени(((*/
	flag_sec = 0;
	timer_on_us(); // Запустили таймер он делает интерапт раз в 1 микросек прибавляя 1 к usec
	while((4*usec)<usec1){ // из-за приведения типов, сложения, сравнения емпирически получилось в 4 раза медленее гдето
	; // мы выйдим из этого цикла только когда количество циклов таймера сравняется с аргументом msec1
	}
	usec = 0; // обнулили msec для следующих вызовов делея
	timer_of(); // Включили таймер, что бы небыло лишних прерываний
}

void my_delay_ms(volatile unsigned long int msec1){
	/* Мой делей, каждый цикл занимает 1 милисекунду */
	flag_sec = 1;
	timer_on_ms(); // Запустили таймер он делает интерапт раз в 1 милисек прибавляя 1 к usec
	while(msec<msec1){ // из-за приведения типов, сложения, сравнения емпирически получилось в 4 раза медленее гдето
	; // мы выйдим из этого цикла только когда количество циклов таймера сравняется с аргументом msec1
	}
	msec = 0; // обнулили msec для следующих вызовов делея
	timer_of(); // Включили таймер, что бы небыло лишних прерываний
}

ISR(TIMER1_COMPA_vect){
	if (flag_sec){
		msec++; // каждую милисекунду это число становитсья больше на 1
	}
	else {
		usec++; // каждую микросекунду это число становитсья больше на 1
	}
}

//---------------------LED_block-----------------------------------------------

void led_ini_13(void){
	DDRB |= LED_BIT; // Включили порт на выход
}

void led_blink_13(unsigned int hmtimes, volatile unsigned long int tfb_ms){
	// Простая функция моргания ледом, использует мой делей, принимает два аргумента
	// hmtimes сколько раз моргнуть и  tfb_ms время на одно моргание
	led_intensity = 0;
	unsigned int t = 0;
	while (t<(hmtimes*2)) {
		PORTB ^= LED_BIT; // меняет значение порта на противоположное
		my_delay_ms((tfb_ms/2));
		t++;
	}
}

//---------------------LED_PWM_block---------------------------------------------

void led_13_disable_pwm(void){ //виключаем pwm
	TCCR0A &= ~(1<<COM0A1);
	TCCR0A &= ~(1<<COM0A0);
	TCCR0A &= ~(1<<WGM01);
	TCCR0A &= ~(1<<WGM00);
}

void led_13_enable_pwm(void) { //включаем pwm
	TCNT0 = 0x00;
	TIMSK0 |= 0x00;
	TCCR0B |= (1<<CS02); // 256 prescaler тут можна поиграться, это частота передачи импульсов на лед
	TCCR0A |= (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00);
}

void led_slowly_13_pwm(volatile unsigned int speed_ms){
	/* Функция которая плавно зажигает или гасит лед
	 * принимает аргумент время зажигания или тушения а в милисекундах, дискретно 25*/
	led_intensity = 0;
	if (state_led == 1) {
		if(!(PORTB&LED_BIT)){ //Если диод был выключен
			OCR0A = 255;
			led_13_enable_pwm();
			while(OCR0A>5){ // медленно увеличиваем напряжение на леде
				if (state_led != 1) { // при нажатии кнопки позволит выйти из этой функции, а не ожидать пока она дойдет
					return;
				}
				OCR0A -= 10;
				my_delay_ms(speed_ms/25); // после каждого повышения делаем делей
			}
			led_13_disable_pwm();
			PORTB |= LED_BIT;
			 // лед мы зажигали так пусть он горит
		}
		else { //Если диод был включен
			OCR0A = 5;
			led_13_enable_pwm();
			while(OCR0A<255){ // медленно уменшаем напряжение на леде
				if (state_led != 1) { // при нажатии кнопки позволит выйти из этой функции, а не ожидать пока она дойдет
					return;
				}
				OCR0A += 10;
				my_delay_ms(speed_ms/25); // после каждого повышения делаем делей
			}
		led_13_disable_pwm();  //отключили pwm
		PORTB &= ~LED_BIT; // лед мы потушили, так пусть он не горит
		}
	}
}

//---------------------USART_NO_INTERUPT_block-------------------------

void ini_usart_no_int(const unsigned long speed){ //виключаем usart без прерываний
	const unsigned int ubrr = CPU_F/8/speed-1;
	UCSR0A |=(1<<U2X0); // speed 2X
	UCSR0B |=(1<<TXEN0); // |(1<<UDRIE0);
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)(ubrr);
	UCSR0C |= (1<<UCSZ10)|(1<<UCSZ00); //8-bit mode
}

unsigned char send_char_nointr(volatile const unsigned char data){
	timer_on_ms();
	volatile const unsigned char time_to_send = 100;
	while(!(UCSR0A&(1<<UDRE0))) {
		if (time_to_send < msec) { // если в течении 100 милисекунд отправка не произошла, то выходим из функции что бы не повиснуть
			timer_of();
			msec = 0;
			sei();
			return 0;
		}
	}
	UDR0 = data;
	timer_of();
	msec = 0;
	sei();
	return data;
}

int send_string_nointr(volatile const char* s){
	volatile char *p;
    p =(char*)s; // making cast
    int length = 0;
    if (s != NULL) {
    	do {
        	if (send_char_nointr(*p)){
        		length++;
        		p++;
        	}
        	else {
        		return -1;
        	}
    	} while (*p);
    }
    return length;
}

//---------------------USART_INTERUPT_block-------------------------
void ini_usart_int(const unsigned long speed){ // включаем usart с прерываниями
	const unsigned int ubrr = CPU_F/8/speed-1;
	UCSR0A |=(1<<U2X0); // speed 2X
	UCSR0B |=(1<<TXEN0)|(1<<TXCIE0);
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)(ubrr);
	UCSR0C |= (1<<UCSZ10)|(1<<UCSZ00); //8-bit mode
}

unsigned char add_char_txc0_buf_intr(volatile const unsigned char data){
	volatile const unsigned char time_to_send = 100;
	if (buffer_txc0[txc0buf_write_point] != '\0'){ // если в буфере не осталось места
		txc0_udr0_statte = 1; // выключаем передачу в буфер
		UDR0 = buffer_txc0[txc0buf_read_point]; // принудительный вызов выталкивания буфера
		buffer_txc0[txc0buf_read_point] = '\0';
		txc0buf_read_point++;
		txc0buf_read_point &= TXC0_BUFFER_MASK;
//		UCSR0B |= (1<<UDRIE0); // используется если делать через другое прерывание
	}
	if (buffer_txc0[txc0buf_write_point] != '\0'){ //безсмысленная проверка, но если ISR не отослала данные, что бы их не затереть мы страхуемся
		return 0;
	}
	timer_on_ms();
    while(txc0_udr0_statte >= 1){
		if (time_to_send < msec) { // если в течении 100 милисекунд буфер не пустел, то выходим из функции что бы не повиснуть
			timer_of();
			msec = 0;
			return 0;
		}
	}
    timer_of();
	buffer_txc0[txc0buf_write_point] = data;
	txc0buf_write_point++;
	txc0buf_write_point &= TXC0_BUFFER_MASK;
	return data;
}

int send_string_intr_with_buff(volatile const char* s){
	volatile char *p;
    p =(char*)s; // making cast
    int length = 0;
    if (s != NULL) {
    	do {
        	if (add_char_txc0_buf_intr(*p)){
        		length++;
        		p++;
        	}
        	else {
        		return -1;
        	}
    	} while (*p);
    }
    if (buffer_txc0[txc0buf_read_point] != '\0'){
    UDR0 = buffer_txc0[txc0buf_read_point]; // принудительный вызов выталкивания буфера
	buffer_txc0[txc0buf_read_point] = '\0';
	txc0buf_read_point++;
	txc0buf_read_point &= TXC0_BUFFER_MASK;
//	UCSR0B |= (1<<UDRIE0); // используется если делать через другое прерывание
    }
    return length;
}

ISR(USART0_TX_vect){
	if (buffer_txc0[txc0buf_read_point] != '\0'){
		UDR0 = buffer_txc0[txc0buf_read_point];
		buffer_txc0[txc0buf_read_point] = '\0';
		txc0buf_read_point++;
		txc0buf_read_point &= TXC0_BUFFER_MASK;
	}
	else {
		txc0_udr0_statte = 0;
	}
}

//ISR(USART0_UDRE_vect){ при передачи нескольких строк подряд без делея, делает ошибки
//	if (buffer_txc0[txc0buf_read_point] == '\0'){
//		UCSR0B &= ~(1<<UDRIE0);
//		txc0_udr0_statte = 0; // когда буфер пустой то разрешаем в него писать
//	}
//	else {
//		UDR0 = buffer_txc0[txc0buf_read_point];
//		buffer_txc0[txc0buf_read_point] = '\0';
//		txc0buf_read_point++;
//		txc0buf_read_point &= TXC0_BUFFER_MASK;
//	}
//}

//---------------------ROTOR_block-----------------------------------


void ini_rotor_8_9(void){
	DDRK &= ~ROTOR_PORTS; // включаем порты на прием
	PORTK &= ~ ROTOR_PORTS; // ставим землю
	PCICR |= (1<<PCIE2); // Разрешаем интерапты по PCIN16-PCIN23, если включена соответствующая маска
	PCMSK2 |= (1<<PCINT16); //Разрешаем интерапты по PCIN16
}

char digit_to_char(volatile unsigned char nu){
	if (nu == 0) return '0';
	if (nu == 1) return '1';
	if (nu == 2) return '2';
	if (nu == 3) return '3';
	if (nu == 4) return '4';
	if (nu == 5) return '5';
	if (nu == 6) return '6';
	if (nu == 7) return '7';
	if (nu == 8) return '8';
	if (nu == 9) return '9';

	return '0';
}

char* brightness_to_string(volatile unsigned char br){
	unsigned char num = 0;

	if ((num = digit_to_char(br/100)) !='0') { // для красоты первые нули меняем на пробелы
		str_bri[13] = num;
		str_bri[14] = digit_to_char((br%100)/10);
	}
	else{
		if ((num = digit_to_char((br%100)/10)) !='0'){
			str_bri[13] = ' ';
			str_bri[14] = num;
		}
		else {
			str_bri[13] = ' ';
			str_bri[14] = ' ';
		}
	}
	str_bri[15] = digit_to_char(((br%100)%10)/1);
	 return str_bri;
}

void encoder_read(void) {
	if(encoder_state >=2) {
		encoder_state = 0;
	}
	else{
		encoder_state++;
	}
	if (state_led == 0){
		state_led = 2; // включаем режим енкодера
		if(!(PORTB&LED_BIT)){ // если мы диодом мограги будем его гасить кодером из его последнего состояния
		brightness = 255;
		}
		else {
			brightness = 0;
		}
		led_13_enable_pwm(); // включаем pwm
	}
	if ((state_led == 1)||(state_led == 3)){
		state_led = 2;
		brightness = OCR0A; // будем гасить или зажигать от последней доступной яркости
		led_13_enable_pwm();
	}
	NewState =(PINK&ROTOR_PORTS); // Читаем пины по двум портам ендодера, 4 соостояния
	if(NewState!=OldState){ // работает и без этой проверки, но один раз на 20 может ошибиться стороной, а так все четко
		switch(OldState){
			case 2:{
				if(NewState == 1){
					if(brightness>10){
					brightness -=10;
					}
					else {
						brightness = 0;
					}

				}
				break;
			}
			case 0:{
				if(NewState == 3){
					if(brightness<245){
					brightness +=10;
					}
					else {
						brightness = 255;
					}
				}
				break;
			}
			case 1:{
				if(NewState == 2){
					if(brightness>10){
					brightness -=10;
					}
					else {
						brightness = 0;
					}
				}
				break;
			}
			case 3:{
				if(NewState == 0){
					if(brightness<245){
					brightness +=10;
					}
					else {
						brightness = 255;
					}
				}
				break;
			}
			default:
				break;
			}
		OldState=NewState;
	}
	OCR0A = brightness; // выставляем OCR0A для pwm, можно былло обойтись только OCR0A, но так понятней визуально
    led_intensity = 100 - (4 *(brightness/10)); // высчитуем и записываем яркость
}

ISR(PCINT2_vect){
	// Обработчик интерапта смены пина в лапке PCINT5
	// В данном случае вызывает обработчик нажатия на кнопку, максимально быстро выходим из обработчика прерываний
	encoder_read();
}
//---------------------BUTTON_block-----------------------------------

void init_button_11(void) {
	/* Функция которая включает ПинЧейнж интерапт по лапке PCIN5 на плате №11
	 * другими будем использовать этот порт для подключения кнопки от */
	DDRB &= ~BUTTON_PORT; // включаем порт на прием
	PORTB |= BUTTON_PORT; // Подключаем Хай-зед уровень
	PCICR |= (1<<PCIE0); // Разрешаем интерапты по PCIN0-PCIN7, если включена соответствующая маска
	PCMSK0 |= (1<<PCINT5); //Разрешаем интерапты по PCIN5
}

void on_button_click_11(void){
	/* Функция обработчик интераптов от кнопки подключенной к PCIN5*/
	unsigned char butcount = 0; // Переменная помогающая бороться с дребежаннием контактов кнопки
	unsigned char safe_mode = 0; // Если вдруг пришла единичная смена пина, нужно что бы незастять в вечном цикле
	while(safe_mode<100) {
		if(!(PINB&BUTTON_PORT)){
// проверяем если нажата кнопка, если она 10 раз проверяеться как нажата, значит врядли это случайный пин пришел
// написано специально под ручную кнопку с ее дребезжанием контактов
			butcount++;
			if (butcount>10){ // если сигнал не ошибочный значит меняем режим диода
				switch(state_led) {
					case 0:{
						state_led = 1; // если были в блике идем в диминг
						break;
					}
					case 1: { // если были в диминге идем на датчик света
						state_led = 3;
						break;
					}
					default: { // иначе все режимы переключаем в блик
						led_13_disable_pwm(); // когда перехдим в режим мигания, выключаем pwm
						state_led = 0;
						break;
					}
				}
				break; // должны выйти здесь, но если не успеем то через safe_mode
			}
		}
		safe_mode++; // после интерапта входим в этот цикл максимум на 100 раз что бы независнуть
	}
	safe_mode = 0; // обнуляем защитную переменную, хоть она и локальна и обнулиться сама, так спокойней на душе
}

ISR(PCINT0_vect){
	// Обработчик интерапта смены пина в лапке PCINT5
	// В данном случае вызывает обработчик нажатия на кнопку
	on_button_click_11();
}

//---------------------TWI_Brietness_block-----------------------------------

void twi_init_ports_20_21_8(void){
    DDRD &= ~(TWI_PORTS); // включаем D0 и D1 они же на борде 20-21
    PORTD |= TWI_PORTS;
    PORTH &= ~(ADD_R); // включаем режим H-resolution mode, путем передачи нуля на ADDR (H5, на борде 8)
    DDRH |= ADD_R;
    TWBR = 72;     // Настроим битрейт 100000, может передавать сюда скорость
    TWSR = 0x00;   // Паша написал 100000 на лекции вот я от нее и плясал, но ее можно варьировать
}

void twi_init_start(void){
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWSTA);
}

int twi_read_light_sensor(void){
	twi_device = 1; // указываем с каким девайсом работаем по twi, 1 - датчик освещения
	twi_bri_error = 0; // выставляем статус ошибок в 0 перед началом измерения
	twi_init_start();
	my_delay_ms(5); // даем время шине указать режим работы датчику хоть там все и на прерываниях, но это для подстраховки
	if (st_signals_count==1) { // если тут мы получили 1, значит все ок, старт мы запустили, режим работы указали спот отправили и оказались здесь
		my_delay_ms(140); // подождем 120-180 мс
			twi_init_start(); //снова отправим старт
			my_delay_ms(5); // даем время шине указать режим чтения датчику и его прочитать, хоть там все и на прерываниях, но это для подстраховки
		}
	else { // иначе что-то пошло не так и надо вернуть ошибку
		return -1;
	}
//	if (twi_bri_error){ // где-то я перемудрил, надо отлаживать в twi_brit_interupt_read, времени не хватило
//		return -1; // если где-то закралась ошибка показываем это программе
//	}
	if ((!led_int_High_Byte)&&(!(led_int_Low_Byte&0b11100000))){ // если света нету вовсе лед не горит
		brightness = 255;
		led_intensity = 100 - (4 *(brightness/10));
		OCR0A = brightness;
		led_13_enable_pwm();
		return 0;
	}
	if (!(led_int_High_Byte&0b11100000)){ // если света немного, то лед горит на 12% мощности
		brightness = 215;
	}
	else {
		brightness = 255 - led_int_High_Byte; // если света много, то лед горит в зависимости от освещения
	}
	led_intensity = 100 - (4 *(brightness/10));
	OCR0A = brightness;
	led_13_enable_pwm();
	return 0;
}

void twi_brit_interupt_read(){
	if (st_signals_count==3){ // если мы удачно приняли два байта ставим в ноль флаг что бы начать сначала
		st_signals_count = 0;
		TWCR = 0;
		return; // возвращаемся, так так тут нам больше делать нечего
	}
	twi_status = TWSR; // запишим текущий статус TWI порта.
	switch(twi_status){
		case 0x08:{
			if (!st_signals_count){ // получили ACK на первый старт
				TWDR = 0b01000110; // отправили SLA+W
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
			}
			else { // всюду если что пошло не так, сразу ставим флаг ошибки
				twi_bri_error = 1;
			}
			break;
		}
		case 0x10:{ // получили ACK на второй старт именно 0x10, а не 0x08!!!
			if (st_signals_count == 1){
				TWDR = 0b01000111; // отправили SLA+R
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
			}
			else {
				twi_bri_error = 1;
			}
			break;
		}
		case 0x18:{ // получили ACK от SLA+W
			if (!st_signals_count){
				TWDR = 0b00010000; // отправили режим работы Opecode
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
			}
			else {
				twi_bri_error = 1;
			}
			break;
			}
		case 0x28:{ // получили ACK на отправленный режим работы, он принят
			if (!st_signals_count){
				st_signals_count++; // теперь после повторного старта мы будем знать что надо оптравить режим чтения
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(TWSTO); // отправили стоп сигнал
			}
			else {
				twi_bri_error = 1;
			}
			break;
			}
		case 0x30:{ // получем после стоп сигнала или если гте-то увидим NACK, но st_signals_count нас направит
			if (st_signals_count == 1){ // первый стоп передан и мы получили NACK
			;
			}
			else {
				twi_bri_error = 1; // иначе что-то где-то пошло не так
			}
			break;
		}
		case 0x40:{ // получили ACK после отправки SLA+R
			if (st_signals_count==1){
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);  // (1<<TWEA) разрешаем отправку АСК после получения байта
			}
			else {
				twi_bri_error = 1;
			}
			break;
		}
		case 0x50:{ // получили байт и отправили ACK
			if (st_signals_count==2){
				led_int_Low_Byte = TWDR; // приняли Low Byte
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(TWSTO); // после приема второго байта автоматом был отправлен АСК, отправляем теперь стоп
				st_signals_count++;  // после принятия Low Byte st_signals_count будет равно 3
			}
			if (st_signals_count==1){
			led_int_High_Byte = TWDR; // приняли High Byte
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);
			st_signals_count++; // после принятия High Byte st_signals_count будет равно 2
			}
			if (!st_signals_count){
				twi_bri_error = 1;
			}
			break;
		}
		default: {
			twi_bri_error = 1;
			break;
		}
	}
}

ISR(TWI_vect){
	switch(twi_device){ // смотрим с каким девайсом мы сейчас работаем по twi
		case 1:{ // это датчик освещения
			twi_brit_interupt_read();
		}
	}
}
//---------------------MAIN_block-----------------------------------

int main() {
	led_ini_13();
	init_button_11();
//	ini_usart_no_int(115200);
	ini_usart_int(115200);
	ini_rotor_8_9();
	twi_init_ports_20_21_8;
	sei();
	PORTB &= ~LED_BIT;
	my_delay_ms(500);
	while(1) {
		switch(state_led){
			case 0:{
				led_blink_13(2, 250);
				send_string_intr_with_buff(brightness_to_string(led_intensity));
				break;
			}
			case 1:{
				led_slowly_13_pwm(500);
				send_string_intr_with_buff(brightness_to_string(led_intensity));
				break;
			}
			case 2:{
				if (!encoder_state){
					send_string_intr_with_buff(brightness_to_string(led_intensity));
					encoder_state++;
				}
				break;
			}
			case 3:{
				twi_read_light_sensor();
				my_delay_ms(250);
				send_string_intr_with_buff(brightness_to_string(led_intensity));
				break;
			}
		}
	}
	return 0;
}

