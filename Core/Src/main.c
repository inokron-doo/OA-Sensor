/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Inokron d.o.o., Gusztav Velez Voros
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// How to setup Visual Studio code
// https://github.com/MaJerle/stm32-cube-cmake-vscode
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_DEVICE_ID	1	//UNIQUE sensor ID, change this to unique for sensor nodes
#define NUMBER_OF_DISTANCE_SENSORS_INSERTED_IN_PCB 1  //change this number according to used number of ultrasonic sensors [1,2,3,4]

#define DIST_BUF_SIZE   10		// number of samples for average calculation
#define SEND_TO_LORA_NET_EVERY_10_SEC 100  //100ms unit

#define ONE_ULTRASONIC_SENSOR 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define SHT85_ADDR         (0x44 << 1) // 8-bit address for HAL
#define CMD_MEASURE_HIGH   0x2400
uint8_t sht85_data[6];
float temperature, humidity;
int8_t int_temperature, int_humidity;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
int send_data_to_LoRA_network(uint32_t value_to_send);
void set_error_LED(uint8_t val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t rx_buffer[10];               	// RX buffer for 4-byte packet
uint16_t distance_mm=0;               	// Latest decoded distance
uint16_t avg_distance_mm=0;				// Average of values stored in the buffer
volatile uint8_t distance_sensor_packet_ready = 0;  // Flag when a new packet is available

// helper functions for debugging
void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

void my_printf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint( fmt, argp);
    va_end(argp);
}
uint32_t MCU_Unique_ID()
{
	// Read the STM32F051 unique ID (96-bit)
	uint32_t uid0 = *(uint32_t *)0x1FFFF7AC;
	uint32_t uid1 = *(uint32_t *)0x1FFFF7B0;
	uint32_t uid2 = *(uint32_t *)0x1FFFF7B4;

	// Mix the values to create a good seed
	uint32_t seed = uid0 ^ uid1 ^ uid2;
	return seed;
}

// Ultrasonic distance measurement
// ---------- Decode AJ-SR04M packet ----------
int decode_ajsr04m(uint8_t *buf, uint16_t *distance_mm)
{
    if (buf[0] != 0xFF) {
    	set_error_LED(1);
        return -1; // Invalid header
    }

    uint8_t msb = buf[1];
    uint8_t lsb = buf[2];
    uint8_t crc = buf[3];
    uint8_t calc_crc = (0xFF + msb + lsb) & 0xFF;

    if (calc_crc != crc) {
        return -2; // CRC mismatch
    }

    *distance_mm = ((uint16_t)msb << 8) | lsb;
    return 0; // OK
}

// https://www.dfrobot.com/product-1934.html
// https://wiki.dfrobot.com/A01NYUB%20Waterproof%20Ultrasonic%20Sensor%20SKU_SEN0313
// -- connection to LoRA Sensor Network PCB V 0.1 --
// Sensor     	PCB SENSOR[1,2,3,4] port
// Red=5V 		+5V
// Black=GND    GND
// Blue=Rx      RX
// Green=Tx     TX
//------------------------
int decode_distance_A01NYUB(uint8_t *buf, uint16_t *distance_mm)
{
	if (buf[0] != 0xFF) {
	    	set_error_LED(1);
	        return -1; // Invalid header
	    }

	uint8_t dist_H,dist_L,dist_CRC, calc_CRC;
	dist_H = buf[1];
	dist_L = buf[2];
	dist_CRC = buf[3];

	 //check for CRC
	calc_CRC = (0xFF+dist_H+dist_L)&0x00FF;
	if(calc_CRC!=dist_CRC)
	{
		my_printf("ERROR:Ultrasonic A01NYUB sensor CRC error");
	    *distance_mm=0;
	    return -2;
	}
	else
	{
		*distance_mm = ((uint16_t)dist_H << 8) | dist_L;
	}
	return 0;
}


typedef struct {
    uint16_t buf[DIST_BUF_SIZE];
    uint8_t head;
    uint8_t count;
} DistanceBuffer_t;

DistanceBuffer_t distBuf[4];  //buffer for 1,2,3,4 distance sensors
/**
 * Add a new distance value (mm) into the circular buffer.
 */
void DistanceBuffer_Add(uint8_t i,uint16_t value)
{
    distBuf[i].buf[distBuf[i].head] = value;

    distBuf[i].head++;
    if (distBuf[i].head >= DIST_BUF_SIZE)
        distBuf[i].head = 0;

    if (distBuf[i].count < DIST_BUF_SIZE)
        distBuf[i].count++;
}

/**
 * Calculate the average of values stored in the buffer.
 * Returns 0 if buffer is empty.
 */
uint16_t DistanceBuffer_GetAverage(uint8_t i)
{
    if (distBuf[i].count == 0)
        return 0;

    uint32_t sum = 0;

    for (uint8_t j = 0; j < distBuf[i].count; j++) {
        sum += distBuf[i].buf[j];
    }

    return (uint16_t)(sum / distBuf[i].count);
}
void DistanceBuffer_Clear(uint8_t idx)
{
    for (uint8_t j = 0; j < DIST_BUF_SIZE; j++) {
        distBuf[idx].buf[j] = 0;
    }

    distBuf[idx].head = 0;
    distBuf[idx].count = 0;
}
/**
 * Get the number of valid items currently stored in the buffer.
 */
uint8_t DistanceBuffer_GetCount(uint8_t i)
{
    return distBuf[i].count;
}

// select one distance sensor using multiplexer chip
void select_distance_sensor(uint8_t sensor_nr_to_select)
{
	switch (sensor_nr_to_select) {
		case 1:
			// Multiplexer select the SENSOR 1
			HAL_GPIO_WritePin(MPX_select_2_GPIO_Port,  MPX_select_2_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(MPX_select_1_GPIO_Port,  MPX_select_1_Pin,  GPIO_PIN_SET);
			break;
		case 2:
			// Multiplexer select the SENSOR 2
			HAL_GPIO_WritePin(MPX_select_2_GPIO_Port,  MPX_select_2_Pin,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MPX_select_1_GPIO_Port,  MPX_select_1_Pin,  GPIO_PIN_SET);
			break;
		case 3:
			// Multiplexer select the SENSOR 3
			HAL_GPIO_WritePin(MPX_select_2_GPIO_Port,  MPX_select_2_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(MPX_select_1_GPIO_Port,  MPX_select_1_Pin,  GPIO_PIN_RESET);
			break;
		case 4:
			// Multiplexer select the SENSOR 4
			HAL_GPIO_WritePin(MPX_select_2_GPIO_Port,  MPX_select_2_Pin,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MPX_select_1_GPIO_Port,  MPX_select_1_Pin,  GPIO_PIN_RESET);
			break;
		default:
			// Multiplexer select the SENSOR 1
			HAL_GPIO_WritePin(MPX_select_2_GPIO_Port,  MPX_select_2_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(MPX_select_1_GPIO_Port,  MPX_select_1_Pin,  GPIO_PIN_SET);
			break;
	}


}

uint8_t float_to_abs_u8(float x)
{
    float v = fabsf(x);        // absolute value

    if (v > 255.0f) v = 255.0f; // clamp to uint8_t range

    return (uint8_t)v;         // truncates toward 0
}

// read temperature and humidity from an SHT85 sensor using the I2C interface (hi2c1)
void SHT85_Read(void)
{
    uint8_t cmd[2] = {0x24, 0x00}; // Single shot, high repeatability, no clock stretching

    // Send measurement command
    HAL_I2C_Master_Transmit(&hi2c1, SHT85_ADDR, cmd, 2, HAL_MAX_DELAY);

    // Wait for measurement (~15 ms typical)
    HAL_Delay(20);

    // Read 6 bytes (temperature + humidity + CRCs)
    if (HAL_I2C_Master_Receive(&hi2c1, SHT85_ADDR, sht85_data, 6, 100) == HAL_OK)
    {
        // Combine bytes
        uint16_t rawTemp = (sht85_data[0] << 8) | sht85_data[1];
        uint16_t rawHum  = (sht85_data[3] << 8) | sht85_data[4];

        // Convert to temperature (°C)
        temperature = -45.0f + 175.0f * ((float)rawTemp / 65535.0f);

        // Convert to humidity (%RH)
        humidity = 100.0f * ((float)rawHum / 65535.0f);
    }
    else
    {
        temperature = 0.0f;
        humidity = 0.0f;
    }

    if(temperature<0)
    {
    	int_temperature=((int8_t)float_to_abs_u8(temperature))|0x80;
    }
    else
    {
    	int_temperature=((int8_t)temperature)&0x7F;
    }
    int_humidity=(int8_t) humidity;

}

#define DISABLE_USART1 USART1->CR1&=~USART_CR1_UE;
#define ENABLE_USART1  USART1->CR1|=USART_CR1_UE;



void sensors_data_sending_to_LoRA_concentrator()
{
	static uint8_t sensor_type_to_send=0;
	uint16_t value=0;
	switch (sensor_type_to_send) {
		case 0:	// distance sensor
			my_printf("Sensor TX to LoRA avg distance:%d [mm]\n\r",avg_distance_mm);
			value=avg_distance_mm;
			break;
		case 1:	// temperature
			//my_printf("Temperature: %d °C, Humidity: %d %%\r\n", int_temperature, int_humidity);
			if((int_temperature&0x80)>0)
				my_printf("Sensor TX to LoRA temperature:-%d [C]\n\r",int_temperature&0x7F);
			else
				my_printf("Sensor TX to LoRA temperature:%d [C]\n\r",int_temperature&0x7F);
			value=int_temperature&0xFF;
			break;
		case 2: // humidity
			my_printf("Sensor TX to LoRA humidity:%d [%%]\n\r",int_humidity);
			value=int_humidity;
			break;

		default:
			value=0;
			break;
	}
	uint32_t data = encode_message(SENSOR_DEVICE_ID, sensor_type_to_send, value);
	send_data_to_LoRA_network(data);
	sensor_type_to_send++;
	if (sensor_type_to_send>2)
		sensor_type_to_send=0;  //reset

}



// ---------- UART RX from selected distance sensor complete callback ----------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        //if (decode_ajsr04m(rx_buffer, &distance_mm) == 0)      	// selected ajsr04m sensor
    	if (decode_distance_A01NYUB(rx_buffer, &distance_mm) == 0)  // selected DFROBOR A01NYUB sensor
        {
        	distance_sensor_packet_ready = 1; // New valid measurement
        }
        // Restart reception
        HAL_UART_Receive_IT(&huart2, rx_buffer, 4);
    }
}
uint32_t global_100ms_counter=0;
//timer period elapsed callback function
//Period elapsed callback in non blocking mode

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t divider_counter = 0;
    if(htim == &htim6)
    {	//timer 6 overflow

    }
    if(htim == &htim14)
	{ //every 10 ms
    	divider_counter++;
		if(divider_counter%10==0)
		{  //every 100 ms
			divider_counter=0;
			global_100ms_counter++;

			HAL_GPIO_WritePin(GPIO_IO1_GPIO_Port,  GPIO_IO1_Pin,  GPIO_PIN_SET);  //Signal trace
			HAL_GPIO_WritePin(GPIO_IO1_GPIO_Port,  GPIO_IO1_Pin,  GPIO_PIN_RESET);
		}
	}
}

void setup_HW()
{
	// LED indicator set to OFF
	HAL_GPIO_WritePin(R_ERROR_GPIO_Port, R_ERROR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(G_WRK_GPIO_Port,   G_WRK_Pin,   GPIO_PIN_RESET);
	HAL_GPIO_WritePin(G_COMM_GPIO_Port,  G_COMM_Pin,  GPIO_PIN_RESET);

	// Multiplexer select the SENSOR 1
	HAL_GPIO_WritePin(MPX_select_2_GPIO_Port,  MPX_select_2_Pin,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(MPX_select_1_GPIO_Port,  MPX_select_1_Pin,  GPIO_PIN_SET);

	// LoRA module setup
	HAL_GPIO_WritePin(MODULE_M0_GPIO_Port,  MODULE_M0_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODULE_M1_GPIO_Port,  MODULE_M1_Pin,  GPIO_PIN_RESET);

	// signal TRACE pin
	HAL_GPIO_WritePin(GPIO_IO1_GPIO_Port,  GPIO_IO1_Pin,  GPIO_PIN_RESET);
}

void toggle_LED_WRK_indicator()
{
	HAL_GPIO_TogglePin(G_WRK_GPIO_Port, G_WRK_Pin);
}
void set_error_LED(uint8_t val)
{
	if(val==1)
		HAL_GPIO_WritePin(R_ERROR_GPIO_Port, R_ERROR_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(R_ERROR_GPIO_Port, R_ERROR_Pin, GPIO_PIN_RESET);
}
void LED_indicator_init()
{
	  HAL_GPIO_WritePin(G_WRK_GPIO_Port, G_WRK_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(R_ERROR_GPIO_Port, R_ERROR_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(G_COMM_GPIO_Port,  G_COMM_Pin,  GPIO_PIN_SET);
	  HAL_Delay(200);	//ms delay
	  HAL_GPIO_WritePin(G_WRK_GPIO_Port, G_WRK_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(R_ERROR_GPIO_Port, R_ERROR_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(G_COMM_GPIO_Port,  G_COMM_Pin,  GPIO_PIN_RESET);
	  HAL_Delay(200);	//ms delay

}

// ENCODING message
uint8_t crc3(uint32_t data) {
    uint8_t crc = 0x07;
    for (int i = 0; i < 21; i++) { // 21 bits before CRC
        uint8_t bit = ((data >> (23 - i)) & 1) ^ ((crc >> 2) & 1);
        crc <<= 1;
        if (bit) crc ^= 0x03;
        crc &= 0x07;
    }
    return crc;
}

uint32_t encode_message(uint8_t dev_id, uint8_t type, uint16_t value)
{
    uint32_t msg = ((dev_id & 0x1F) << 19)
                 | ((type & 0x03) << 17)
                 | ((value & 0x3FFF) << 3);
    uint8_t crc = crc3(msg);
    msg |= crc & 0x07;
    return msg;
}

//DECODING message
/*
typedef struct {
    uint8_t dev_id;
    uint8_t type;
    uint16_t value;
    uint8_t crc_ok;
} SensorMessage;

SensorMessage decode_message(uint32_t msg) {
    SensorMessage m;
    m.dev_id = (msg >> 19) & 0x1F;
    m.type   = (msg >> 17) & 0x03;
    m.value  = (msg >> 3)  & 0x3FFF;
    uint8_t crc = msg & 0x07;
    uint8_t expected_crc = crc3(msg & 0xFFFFF8);
    m.crc_ok = (crc == expected_crc);
    return m;
}
*/
int send_data_to_LoRA_network(uint32_t value_to_send)
{
	uint8_t buffer[6];
	buffer[0]=0xFF;		//ADDR 0xFFFF = broadcast transmission
	buffer[1]=0xFF;		//ADDR
	buffer[2]= 0x04;	//CH04
	buffer[3] = (value_to_send >> 16) & 0xFF;  // MSB
	buffer[4] = (value_to_send >> 8)  & 0xFF;
	buffer[5] = value_to_send & 0xFF;
	HAL_GPIO_WritePin(G_COMM_GPIO_Port,  G_COMM_Pin,  GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, buffer, 6, HAL_MAX_DELAY);
	/* debug inly
	my_printf("TX:");
	for(int i=0;i<6;i++)
		my_printf("%02X ",buffer[i]);
	my_printf(":\n\r");
	*/
	HAL_GPIO_WritePin(G_COMM_GPIO_Port,  G_COMM_Pin,  GPIO_PIN_RESET);
	return 0;
}

int read_LoRA_433MHZ_module_config_data()  // not in use for 868Mhz LoRA
{

	uint8_t buffer[6];

	HAL_GPIO_WritePin(MODULE_M0_GPIO_Port,  MODULE_M0_Pin,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(MODULE_M1_GPIO_Port,  MODULE_M1_Pin,  GPIO_PIN_SET);
	HAL_Delay(300);
	buffer[0]=0xC1;		//Read config for 433 MHz module
	buffer[1]=0xC1;		//
	buffer[2]=0xC1;

	HAL_UART_Transmit(&huart1, buffer, 3, HAL_MAX_DELAY);
	my_printf("TX:\n\r");
	HAL_IWDG_Refresh(&hiwdg);
	HAL_UART_Receive(&huart1, buffer, 6, 100);
	my_printf("RX:");
	for(int i=0;i<6;i++)
		my_printf("%02X ",buffer[i]);
	my_printf(":\n\r");


	HAL_GPIO_WritePin(MODULE_M0_GPIO_Port,  MODULE_M0_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODULE_M1_GPIO_Port,  MODULE_M1_Pin,  GPIO_PIN_RESET);
	return 0;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t periodic_timer=0;
	uint32_t tick_counter;
	uint32_t time_of_measurement_on_distance_sensor;
	uint8_t select_sensor_number=1;
#ifndef ONE_ULTRASONIC_SENSOR
	uint32_t sum_for_avg=0;
#endif
	uint8_t err_cnt=0;
	uint16_t print_out_cnt =0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_CRC_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  my_printf("LoRA sensor node ID:%d SystemCoreClock=%d [Hz] MCU id:%X\n\r",SENSOR_DEVICE_ID,SystemCoreClock,MCU_Unique_ID());
  setup_HW();
  LED_indicator_init();
  HAL_TIM_Base_Start_IT(&htim14);

  //read_LoRA_433MHZ_module_config_data();
  // Start first UART reception (non-blocking)
	// --- Flush UART2 buffer before enabling interrupts ---

	volatile uint32_t tmp;
	tmp = huart2.Instance->RDR;  // clear any junk
	(void)tmp;
	__HAL_UART_CLEAR_OREFLAG(&huart2);
	HAL_UART_Receive_IT(&huart2, rx_buffer, 4);
	tick_counter = HAL_GetTick();  // increased every 1ms
	time_of_measurement_on_distance_sensor=  HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  periodic_timer++;
	  if((HAL_GetTick()-tick_counter)>1000)
	  {
		  tick_counter = HAL_GetTick();
		  toggle_LED_WRK_indicator();
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if((HAL_GetTick()-time_of_measurement_on_distance_sensor)>500) // 500ms measurement on selected sensor
	  {
		  time_of_measurement_on_distance_sensor = HAL_GetTick();
#ifdef ONE_ULTRASONIC_SENSOR
		  select_sensor_number=2;   //SENSOR2 port
		  avg_distance_mm = DistanceBuffer_GetAverage(select_sensor_number-1);

#else
		  // calculate average of all used distance sensors
		  sum_for_avg=0;
		  for(int j=0;j<NUMBER_OF_DISTANCE_SENSORS_INSERTED_IN_PCB;j++)
		  {
			  sum_for_avg=sum_for_avg+DistanceBuffer_GetAverage(j);//Calculate the average of values stored in the buffer.
		  }
		  avg_distance_mm = sum_for_avg/NUMBER_OF_DISTANCE_SENSORS_INSERTED_IN_PCB;  //average for all sensors
		  // debug only
		  //my_printf("Selected MPX:%d AVG:%u\n\r",select_sensor_number,avg_distance_mm);

		  //select the next distance sensor
		  select_sensor_number++;
		  if(select_sensor_number>NUMBER_OF_DISTANCE_SENSORS_INSERTED_IN_PCB)
			  select_sensor_number=1;


#endif
		  select_distance_sensor(select_sensor_number); // select the available sensor for distance measurement [1,2,3,4]
		  DistanceBuffer_Clear(select_sensor_number-1);

	  }



	  if (distance_sensor_packet_ready)
	  {

		  set_error_LED(0); //clear error
		  distance_sensor_packet_ready=0;
#ifdef ONE_ULTRASONIC_SENSOR
		  select_sensor_number=2;
#endif
		  DistanceBuffer_Add(select_sensor_number-1,distance_mm); // add new measurement into circular buffer

		  if((print_out_cnt++)>10)
		  {
			  my_printf("distance:%u [mm] on sensor NUM:%d \n\r",distance_mm,select_sensor_number); //debug only
			  print_out_cnt =0;
		  }
		  //my_printf("AVG distance:%u [mm] on sensor:%d items:%d\n\r",
		  //			 avg_distance_mm,select_sensor_number,DistanceBuffer_GetCount(select_sensor_number-1)); //debug only
		  err_cnt=0;
	  }
	  else
	  {
		  err_cnt++;
		  if(err_cnt>5000){
			  my_printf("Error: read distance sensor failed.\n\r");
			  set_error_LED(1); //set error indication
		  }
	  }
	  HAL_IWDG_Refresh(&hiwdg);
	  if(global_100ms_counter%SEND_TO_LORA_NET_EVERY_10_SEC==0) //every 10 sec send a message to LoRA concentrator
	  {
		  SHT85_Read();
		  my_printf("TX to LoRA concentrator at: %d [s]\n\r",global_100ms_counter/10);
		  sensors_data_sending_to_LoRA_concentrator();
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_IO1_Pin|MPX_select_2_Pin|MPX_select_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MODULE_M0_Pin|MODULE_M1_Pin|G_COMM_Pin|R_ERROR_Pin
                          |G_WRK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DHT22_Pin MODULE_AUX_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin|MODULE_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_IO1_Pin MPX_select_2_Pin MPX_select_1_Pin */
  GPIO_InitStruct.Pin = GPIO_IO1_Pin|MPX_select_2_Pin|MPX_select_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MODULE_M0_Pin MODULE_M1_Pin G_COMM_Pin R_ERROR_Pin
                           G_WRK_Pin */
  GPIO_InitStruct.Pin = MODULE_M0_Pin|MODULE_M1_Pin|G_COMM_Pin|R_ERROR_Pin
                          |G_WRK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
