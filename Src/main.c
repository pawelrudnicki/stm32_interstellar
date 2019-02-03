
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SLAVE_ADDRESS_LCD 0x3F << 1
#define BUFF_SIZE 2000
#define SLIP_START			'/'
#define SLIP_END			';'
#define SLIP_ESC			'&'
#define SLIP_ESC_START		'('
#define SLIP_ESC_END		')'
#define SLIP_ESC_ESC		'!'
#define MASTER_ADDRESS		"01"
#define SLAVE_ADDRESS		"72"
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

volatile uint32_t speed_dot = 250;
volatile uint32_t speed_dash = 750;
volatile uint8_t index_timing = 0;
volatile uint32_t timer_ms = 0;
volatile uint32_t count_morse = 0;
volatile uint32_t gap_time = 0;

const char *latin = "**ETIANMSURWDKGOHVF*L*PJBXCYZQ**";

char dotsNdashes[255] = {'\0'};

char MORSE_BUFF_RX[253] = {'\0'};
char SLIP_RX [259] = {'\0'};
char SLIP_MASTER_ADDR[2] = {'\0'};
char SLIP_SLAVE_ADDR[2] = {'\0'};
char SLIP_PAYLOAD[255] = {'\0'};
char Buff_Rx [BUFF_SIZE] = {'\0'};
char Buff_Tx [BUFF_SIZE] = {'\0'};

char txMorsetable[253] = {'\0'};
volatile uint8_t emptytxmorsetable=0;
volatile uint8_t busytxmorsetable=0;

int TIMING_TABLE[255] = {0};

volatile uint8_t Busy_Morse_Rx = 0;
volatile uint8_t Empty_Morse_Rx = 0;

volatile uint8_t ReaderDecode = 0;

volatile uint8_t Busy_Rx = 0;
volatile uint8_t Empty_Rx = 0;
volatile uint8_t Busy_Tx = 0;
volatile uint8_t Empty_Tx = 0;

uint8_t isCounting = 1;


volatile uint32_t xms20 = 0;
volatile uint32_t xms1 = 0;
volatile uint8_t x02s;
volatile uint8_t x001s;

volatile uint32_t xmsdot = 0;
volatile uint32_t xmsdash = 0;
volatile uint32_t xmsspace = 0;

volatile static uint8_t isStart = 0;
volatile static uint8_t isEnd = 0;
volatile static uint8_t isEsc = 0;

volatile uint32_t kropka;
volatile uint32_t kreska;
volatile uint32_t spacja;

volatile uint8_t isDot=0;
volatile uint8_t isDash=0;

volatile uint8_t dotFlag=0;
volatile uint8_t dashFlag=0;
volatile uint8_t spaceFlag=0;

volatile uint8_t x_dot=0;
volatile uint8_t x_dash=0;
volatile uint8_t x_space=0;

volatile uint8_t alpha = 0;

char testowa[249] = {'\0'};
/*START OF CALLBACKS*/


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		alpha = 1;
		if(index_timing >= 255) index_timing = 0;
		TIMING_TABLE[index_timing] = count_morse;
		index_timing++;
		count_morse=0;
		isCounting = 1;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		if(Empty_Tx != Busy_Tx){
			uint8_t tmp = Buff_Tx[Busy_Tx];
			Busy_Tx++;
			if(Busy_Tx >= BUFF_SIZE)Busy_Tx=0;
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		Empty_Rx++;
		if(Empty_Rx >= BUFF_SIZE)Empty_Rx = 0;
		HAL_UART_Receive_IT(&huart2,&Buff_Rx[Empty_Rx],1);
	}
}
/*END OF CALLBACKS*/

/*LCD START*/
void lcd_send_cmd (char cmd) // funkcja wysylajaca  w dwoch czesciach, gorna i dolna
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);	// wybierz gorny polbajt->nibble
	data_l = ((cmd<<4)&0xf0);	// tylko jeden dolny nibble
	data_t[0] = data_u|0x0C;	//en=1, rs=0
	data_t[1] = data_u|0x08;	//en=0, rs=0
	data_t[2] = data_l|0x0C;	//en=1, rs=0
	data_t[3] = data_l|0x08;	//en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
	//HAL_I2C_Master_Transmit(&handle, address, data, size, timeout);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void)
{
	lcd_send_cmd(0x01);
	lcd_send_cmd (0x02); //return home
	lcd_send_cmd (0x28);
	lcd_send_cmd (0x0c);
	lcd_send_cmd (0x80); //line1
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
/*LCD END*/


void changeTimeToMorse(){
	uint8_t tmpindex;
	uint8_t smallest = TIMING_TABLE[0];

	for(volatile int i = 1; i<index_timing; i++){
		if(TIMING_TABLE[i] < smallest && TIMING_TABLE[i] != 1){
			smallest = TIMING_TABLE[i];
		}
	}

	if(smallest >= 100){
		kropka = smallest;
		kreska = (kropka*3);
	} else{
		kropka = 250;
		kreska = 750;
	}

	for(tmpindex=0; tmpindex<index_timing; tmpindex++){
		if(((kropka - 50) <= TIMING_TABLE[tmpindex]) && (TIMING_TABLE[tmpindex] < kreska) && (TIMING_TABLE[tmpindex] > 1)){
			dotsNdashes[tmpindex] = '.';
		}

		else if(TIMING_TABLE[tmpindex] >= kreska){
			dotsNdashes[tmpindex] = '-';
		}
		else if(TIMING_TABLE[tmpindex] == 1){
			dotsNdashes[tmpindex] = 'q';
		}
	}
}

void USART_fsend(char* format,...){
char tmp_rs[128];
int i;
__IO int idx;
va_list arglist;
	va_start(arglist, format);
	vsprintf(tmp_rs, format, arglist);
	va_end(arglist);
	idx = Empty_Tx;
	for(i=0;i<strlen(tmp_rs); i++){
		Buff_Tx[idx]=tmp_rs[i];
		idx++;
		if(idx >= BUFF_SIZE)idx=0;
	}
	__disable_irq();
	if((Empty_Tx==Busy_Tx)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){
		Empty_Tx=idx;
		uint8_t tmp = Buff_Tx[Busy_Tx];
		Busy_Tx++;
		if(Busy_Tx >= BUFF_SIZE)Busy_Tx=0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	}else{
		Empty_Tx=idx;
	}
	__enable_irq();
}

void dash() {
	isDash=1;
	if(isDash==1){
		txMorsetable[emptytxmorsetable]='-';
		emptytxmorsetable++;
		txMorsetable[emptytxmorsetable]='y';
		emptytxmorsetable++;
		if(emptytxmorsetable>=253)emptytxmorsetable=0;
		isDash=0;
	}
}

void dot() {
	isDot=1;
	if(isDot==1){
		txMorsetable[emptytxmorsetable]='.';
		emptytxmorsetable++;
		txMorsetable[emptytxmorsetable]='x';
		emptytxmorsetable++;
		if(emptytxmorsetable>=253)emptytxmorsetable=0;
		isDot=0;
	}
}

void conv(uint8_t decimal) {
	//lcd_send_cmd(0x01);
	if (decimal) {
		conv(decimal/2);
			if (decimal != 1){
				if(decimal%2){
					 dash();
				}
				else{
					 dot();
				}
			}
	}
}

void morse(char c) {
	if (c >= 'a' && c <= 'z') c -= 32;
	if (c < 'A' || c > 'Z') return;
	uint8_t i = 0;
	while (latin[++i] != c);
	conv(i);
	txMorsetable[emptytxmorsetable]=' ';
	emptytxmorsetable++;
	if(emptytxmorsetable >= 253)emptytxmorsetable=0;
}


void HAL_SYSTICK_Callback(void){
	xmsdot++;
	xmsdash++;
	xmsspace++;

	if(xmsdot >= 100){
		xmsdot=0;
		dotFlag=1;
	}

	if(xmsdash >= 300){
		xmsdash=0;
		dashFlag=1;
	}

	if(xmsspace >= 300){
		xmsspace=0;
		spaceFlag=1;
	}
}

uint8_t czypustyRx(){
	if(Empty_Rx == Busy_Rx){
		return 0;
	}
	else{
		return 1;
	}
}

uint8_t morse_kbhit(){
	if(Empty_Morse_Rx == Busy_Morse_Rx){
		return 0;
	} else{
		return 1;
	}
}

uint8_t USART_getchar(){
	volatile uint8_t tmp;

	if(Empty_Rx != Busy_Rx){
		tmp = Buff_Rx[Busy_Rx];
		Busy_Rx++;
		if(Busy_Rx >= BUFF_SIZE)
			Busy_Rx=0;
		return tmp;
	}
	else{
		return 0;
	}
}

uint8_t MORSE_getchar(){
	volatile uint8_t tmpx;

	if(Empty_Morse_Rx != Busy_Morse_Rx){
		tmpx = MORSE_BUFF_RX[Busy_Morse_Rx];
		Busy_Morse_Rx++;
		if(Busy_Morse_Rx >= 253)
			Busy_Morse_Rx=0;
		return tmpx;
	} else{
		return 0;
	}
}


void morse_encode(){
	for(volatile uint8_t i=6; i<=253; i++){
		MORSE_BUFF_RX[Empty_Morse_Rx] = SLIP_PAYLOAD[i];
		morse(SLIP_PAYLOAD[i]);
		//USART_fsend(" ");
		if(SLIP_PAYLOAD[i] == '\0'){
			Empty_Morse_Rx=0;
			break;
		} else{
			Empty_Morse_Rx++;
		}
	}
}

typedef struct
{
    char* morse;
    char* ascii;
} morse_table_t;

uint8_t MORSE_DECODE(){
    char input[249];
    volatile uint8_t x = 0;
    morse_table_t table[] = { {".-", "A"},
                              {"-...", "B"},
                              {"-.-.", "C"},
							  {"-..", "D"},
							  {".", "E"},
							  {"..-.", "F"},
							  {"--.", "G"},
							  {"....", "H"},
							  {"..", "I"},
							  {".---", "J"},
							  {"-.-", "K"},
							  {".-..", "L"},
							  {"--", "M"},
							  {"-.", "N"},
							  {"---", "O"},
							  {".--.", "P"},
							  {"--.-", "Q"},
							  {".-.", "R"},
							  {"...", "S"},
							  {"-", "T"},
							  {"..-", "U"},
							  {"...-", "V"},
							  {".--", "W"},
							  {"-..-", "X"},
							  {"-.--", "Y"},
							  {"--..", "Z"},
							  {"-----", "0"},
							  {".----", "1"},
							  {"..---", "2"},
							  {"...--", "3"},
							  {"....-", "4"},
							  {".....", "5"},
							  {"-....", "6"},
							  {"--...", "7"},
							  {"---..", "8"},
							  {"----.", "9"}
    };

	for(int i=6; i<249; i++){
		input[x] = SLIP_PAYLOAD[i];
		if(SLIP_PAYLOAD[i] == '\0'){
			x=0;
			break;
		} else{
			x++;
		}
	}
	memset(&SLIP_PAYLOAD[0], 0, sizeof(SLIP_PAYLOAD));
	USART_fsend("Odebrano: ");
    char* segment;
    volatile static int i=0;
    volatile static int j=0;
    segment = strtok(input, " ");

    lcd_init();
    lcd_send_cmd(0x01);
    while(segment)
    {
        for(i=0; i<35; ++i)
        {
            if (!strcmp(segment, table[i].morse))
            {
            	USART_fsend(table[i].ascii);
            	testowa[j] = *(table[i].ascii);
            	j++;
            }
        }
        segment = strtok(NULL, " ");
    }
    USART_fsend("\r\n");
    lcd_send_string(testowa);
    return 0;
}

uint8_t MORSE_DECODE_FOR_BUTTON(){
    char input[249];
    morse_table_t table[] = { {".-", "A"},
                              {"-...", "B"},
                              {"-.-.", "C"},
							  {"-..", "D"},
							  {".", "E"},
							  {"..-.", "F"},
							  {"--.", "G"},
							  {"....", "H"},
							  {"..", "I"},
							  {".---", "J"},
							  {"-.-", "K"},
							  {".-..", "L"},
							  {"--", "M"},
							  {"-.", "N"},
							  {"---", "O"},
							  {".--.", "P"},
							  {"--.-", "Q"},
							  {".-.", "R"},
							  {"...", "S"},
							  {"-", "T"},
							  {"..-", "U"},
							  {"...-", "V"},
							  {".--", "W"},
							  {"-..-", "X"},
							  {"-.--", "Y"},
							  {"--..", "Z"},
							  {"-----", "0"},
							  {".----", "1"},
							  {"..---", "2"},
							  {"...--", "3"},
							  {"....-", "4"},
							  {".....", "5"},
							  {"-....", "6"},
							  {"--...", "7"},
							  {"---..", "8"},
							  {"----.", "9"},
    };

	for(int i=0; i<249; i++){
		input[Empty_Morse_Rx] = dotsNdashes[i];
		if(dotsNdashes[i] == '\0'){
			Empty_Morse_Rx=0;
			break;
		} else{
			Empty_Morse_Rx++;
		}
	}

	memset(&dotsNdashes[0], 0, sizeof(dotsNdashes));
	USART_fsend("Odebrano: ");

    char* segment;
    int i;
    segment = strtok(input, "q");


    while(segment)
    {
        for(i=0; i<35; ++i)
        {
            if (!strcmp(segment, table[i].morse)) USART_fsend(table[i].ascii);
        }
        segment = strtok(NULL, "q");
    }
    USART_fsend("\r\n");
    return 0;
}

uint8_t SLIP_getline(char *buff){
	static uint8_t bf[128];
	volatile static uint8_t idx=0;
	uint8_t ret;

	while(czypustyRx()){
		bf[idx] = USART_getchar();

		if(bf[idx] == SLIP_START){		//przychodzi start -> '/'
			isStart = 1;
			idx=0;
			bf[idx]=0;
		}
		else if(bf[idx] == SLIP_ESC){	//przychodzi esc -> '&'
			isEsc = 1;
			bf[idx] = USART_getchar();
		}
		else if(bf[idx] == SLIP_ESC_START && isEsc == 1){		//przychodzi esc_start -> '('
			bf[idx] = SLIP_START;
			idx++;
		}
		else if(bf[idx] == SLIP_ESC_ESC && isEsc == 1){	//przychodzi esc_esc -> '!'
			bf[idx] = SLIP_ESC;
			idx++;
		}
		else if(bf[idx] == SLIP_ESC_END && isEsc == 1){	//przychodzi esc_end -> ')'
			bf[idx] = SLIP_END;
			idx++;
		}
		else if(bf[idx] == SLIP_END){	//przychodzi end -> ';'
			isEnd = 1;
			bf[idx]=0;
			for(int i=0; i<=idx; i++){
				buff[i]=bf[i];
			}
			ret=idx;
			idx=0;
			return ret;
		}
		else{
			isEsc=0;
			idx++;
			if(idx>=128)idx=0;
		}
	}
	return 0;
}

void SLIP_explore(){
	volatile int j=0;
	volatile int k=0;
	for(int i=0; i<=1;i++){				// master address copying
		SLIP_MASTER_ADDR[i] = SLIP_RX[i];
	}
	for(int i=2; i<=3; i++){			// slave address ..
		SLIP_SLAVE_ADDR[j]=SLIP_RX[i];
		j++;
		if(j>=2)j=0;
	}
	for(int i=4; i<=255; i++){			// getting payload
		SLIP_PAYLOAD[k]=SLIP_RX[i];
		k++;
		if(k>=255)k=0;
	}
}

void SLIP_processline(){
	SLIP_getline(SLIP_RX);

	if(isStart == 1 && isEnd == 1){
		isEsc=0;
		isStart=0;
		isEnd=0;
		SLIP_explore();
		if(strcmp(SLIP_MASTER_ADDR, MASTER_ADDRESS) == 0){
			if(strcmp(SLIP_SLAVE_ADDR, SLAVE_ADDRESS) == 0){
				if(strcmp(SLIP_PAYLOAD, "on") == 0){
					alpha = 1;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					  memset(&txMorsetable[0], 0, sizeof(txMorsetable));
					  emptytxmorsetable=0;
					  busytxmorsetable=0;
				}
				else if(strcmp(SLIP_PAYLOAD, "off") == 0){
					alpha = 1;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					  memset(&txMorsetable[0], 0, sizeof(txMorsetable));
					  emptytxmorsetable=0;
					  busytxmorsetable=0;
				}
				else if(strncmp(SLIP_PAYLOAD, "morse:", 6) == 0){
					lcd_send_cmd(0x01);
					morse_encode();
					USART_fsend("%s\r\n", txMorsetable);
					alpha = 0;
				}
				else if(strncmp(SLIP_PAYLOAD, "alpha:", 6) == 0){
					alpha = 1;
					MORSE_DECODE();
					memset(&txMorsetable[0], 0, sizeof(txMorsetable));
					emptytxmorsetable=0;
					busytxmorsetable=0;
				}
				else if(strcmp(SLIP_PAYLOAD, "odbierz") == 0){
					alpha = 1;
					memset(&txMorsetable[0], 0, sizeof(txMorsetable));
					emptytxmorsetable=0;
					busytxmorsetable=0;
					changeTimeToMorse();
					MORSE_DECODE_FOR_BUTTON();
					memset(&dotsNdashes[0], 0, sizeof(dotsNdashes));
					memset(&TIMING_TABLE[0], 0, sizeof(TIMING_TABLE));
					isCounting = 0;
					index_timing = 0;
					gap_time = 0;
				}
				else if(sscanf(SLIP_PAYLOAD, "speed:%d", &speed_dot) == 1){
					alpha = 1;
					memset(&txMorsetable[0], 0, sizeof(txMorsetable));
					emptytxmorsetable=0;
					busytxmorsetable=0;
					if(speed_dot >= 100 && speed_dot <= 3000){
						speed_dash = (speed_dot*3);
					} else{
						speed_dot = 250;
						speed_dash = 750;
					}
				}
			}
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  SysTick_Config(SystemCoreClock / 1000);
  HAL_UART_Receive_IT(&huart2, &Buff_Rx[Empty_Rx], 1);
  lcd_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  SLIP_processline();

	  volatile char c = txMorsetable[busytxmorsetable];

	  if(txMorsetable[busytxmorsetable] == '.' && alpha < 1){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		  if(dotFlag > 0){
			  xmsdash=0;
			  xmsspace=0;
			  dashFlag=0;
			  spaceFlag=0;
			  USART_fsend("%c", c);
			  dotFlag=0;
			  lcd_send_string(".");
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			  busytxmorsetable++;
		  }
	  }

	  if(txMorsetable[busytxmorsetable] == 'x' && alpha < 1){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		  if(dotFlag > 0){
			  xmsdash=0;
			  xmsspace=0;
			  dashFlag=0;
			  spaceFlag=0;
			  dotFlag=0;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			  busytxmorsetable++;
		  }
	  }

	  if(txMorsetable[busytxmorsetable] == '-' && alpha < 1){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		  if(dashFlag > 0){
			  xmsdot=0;
			  xmsspace=0;
			  dotFlag=0;
			  spaceFlag=0;
			  USART_fsend("%c", c);
			  dashFlag=0;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			  lcd_send_string("-");
			  busytxmorsetable++;
		  }
	  }

	  if(txMorsetable[busytxmorsetable] == 'y' && alpha < 1){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		  if(dashFlag > 0){
			  xmsdot=0;
			  xmsspace=0;
			  dotFlag=0;
			  spaceFlag=0;
			  dashFlag=0;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			  busytxmorsetable++;
		  }
	  }

	  if(txMorsetable[busytxmorsetable] == ' ' && alpha < 1){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		  if(spaceFlag > 0){
			  xmsdot=0;
			  xmsdash=0;
			  dashFlag=0;
			  dotFlag=0;
			  USART_fsend("%c", c);
			  spaceFlag=0;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			  lcd_send_string(" ");
			  busytxmorsetable++;
		  }
	  }

	  if(txMorsetable[busytxmorsetable] == '\0' && alpha < 1){
		  lcd_send_cmd(0x01);
		  alpha = 1;
		  memset(&txMorsetable[0], 0, sizeof(txMorsetable));
		  emptytxmorsetable=0;
		  busytxmorsetable=0;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
