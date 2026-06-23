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

// Distance sensor pipeline tunables.
// The sensor outputs ~10 frames/s at 115200 baud.
// With a 10 s TX interval the active sensor collects up to ~100 samples.
// We allocate 100 slots; 95th-percentile index = (count * 95) / 100.
#define DIST_BUF_SIZE           100  // Per-sensor sample buffer (up to ~10 s @ 10 Hz)
#define PROBE_TIMEOUT_MS        2500 // Startup probe: wait this long per port for a valid frame
#define SEND_TO_LORA_NET_EVERY_10_SEC 100  // 100 ms units (10 s)
#define REPROBE_INTERVAL_100MS  1800 // Re-probe sensors every 3 minutes (1800 x 100 ms)
#define NO_FRAME_TIMEOUT_MS     3000 // Light error LED if no valid frame arrives within this time
#define DROP_AFTER_MISSES       3    // Consecutive failed reprobes before a known sensor is dropped
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
uint8_t sht85_ok = 0;  // 1 after a successful I2C read, 0 if sensor absent/failed
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
void reprobe_sensors(void);
void select_distance_sensor(uint8_t sensor_nr_to_select);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// DFRobot SEN0524 TOF sensor: 115200 baud, 16-byte active-output frames.
// Frame layout:
//  [0]      Frame Header    0x57
//  [1]      Function Mark   0x00
//  [2]      reserved
//  [3]      id              module ID
//  [4..7]   system_time     uint32 LE (ms)
//  [8..10]  dis*1000        int24  LE (metres * 1000, i.e. mm)
//  [11]     dis_status      0=valid, 1=invalid
//  [12..13] signal_strength uint16 LE
//  [14]     range_precision uint8 (cm)
//  [15]     sum_check       lowest byte of sum of bytes [0..14]

#define SEN0524_FRAME_HEADER    0x57
#define SEN0524_FUNCTION_MARK   0x00
#define SEN0524_FRAME_LEN       16

// State machine states for frame reception
typedef enum {
    TOF_STATE_WAIT_HEADER = 0,   // waiting for 0x57
    TOF_STATE_WAIT_FUNC,         // waiting for 0x00
    TOF_STATE_COLLECT,           // collecting remaining 14 bytes
} TofRxState_t;

static volatile TofRxState_t tof_rx_state  = TOF_STATE_WAIT_HEADER;
static volatile uint8_t      tof_rx_byte   = 0;   // single-byte DMA/IT target
static          uint8_t      tof_frame_buf[SEN0524_FRAME_LEN];
static volatile uint8_t      tof_frame_idx = 0;   // bytes collected so far

uint16_t distance_mm        = 0;   // Latest decoded distance (mm)
volatile uint8_t distance_sensor_packet_ready = 0;  // Flag: new VALID measurement available
// Set whenever a structurally-valid frame (good header + checksum) is decoded,
// regardless of whether the measurement itself is valid. This is the liveness
// test used by probe_sensors()/reprobe_sensors() and the no-frame watchdog, so a
// sensor that is alive but currently reading invalid (fog, empty trough,
// dead-zone -> distance 0) is NOT mistaken for absent.
volatile uint8_t tof_frame_seen = 0;
static uint32_t last_valid_packet_tick = 0;   // for the watchdog

// Current distance sensor slot (index into active_sensor_idx[]) that the MUX
// is pointing at.  Written authoritatively by sensors_data_sending_to_LoRA_concentrator()
// whenever it switches the MUX; read by the main loop to target DistanceBuffer_Add().
static uint8_t cur_dist_slot = 0;

// helper functions for debugging
void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsnprintf(string, sizeof(string), fmt, argp)) // build string (bounded)
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

	// Fold the three words into a single 32-bit identifier
	return uid0 ^ uid1 ^ uid2;
}

// ---------------------------------------------------------------------------
// Alternative distance-sensor decoders.
//
// The active build uses the DFRobot SEN0524 TOF sensor (decoded in
// decode_SEN0524 / the UART2 RX callback). The two decoders below are kept
// for board variants fitted with an ultrasonic sensor instead; they are not
// called in the SEN0524 data path. Wire one into HAL_UART_RxCpltCallback if
// the corresponding sensor is used.
// ---------------------------------------------------------------------------

// Decode an AJ-SR04M ultrasonic packet (4 bytes: 0xFF, MSB, LSB, checksum).
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

// Decode a DFRobot A01NYUB waterproof ultrasonic packet
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
    uint8_t  count;   // number of valid samples stored (0 .. DIST_BUF_SIZE)
} DistanceBuffer_t;

DistanceBuffer_t distBuf[4];  //buffer for 1,2,3,4 distance sensors

// Sensor auto-detection (set at startup by probe_sensors())
uint8_t sensor_present[4]   = {0, 0, 0, 0};  // 1 if port [0..3] responded
uint8_t sensor_miss[4]      = {0, 0, 0, 0};  // consecutive reprobe misses per port (hysteresis)
uint8_t num_sensors_present = 0;              // how many ports are active
uint8_t active_sensor_idx[4];                 // indices (0-based) of active ports

static uint8_t tx_slot           = 0;  // TX rotation position (shared with reprobe_sensors)
static uint8_t reprobe_requested = 0;  // set to 1 to trigger an out-of-schedule reprobe

/**
 * Add a new distance value (mm) into the flat buffer (fills up to DIST_BUF_SIZE).
 * Oldest samples are simply dropped once the buffer is full - caller should
 * clear the buffer after reading the result.
 */
void DistanceBuffer_Add(uint8_t i, uint16_t value)
{
    if (distBuf[i].count < DIST_BUF_SIZE)
        distBuf[i].buf[distBuf[i].count++] = value;
    // else: buffer full - discard; caller must clear between TX events
}

/**
 * 95th-percentile filter: sort a local copy of the buffer and return the
 * value at the 95 % rank.  Returns 0 if buffer is empty.
 *
 * With N samples the 95th-percentile index = (N * 95) / 100
 * (integer division; for N=100 gives index 95, for N=10 gives index 9).
 */
uint16_t DistanceBuffer_Get95thPercentile(uint8_t i)
{
    uint8_t n = distBuf[i].count;
    if (n == 0)
        return 0;

    uint16_t tmp[DIST_BUF_SIZE];
    for (uint8_t j = 0; j < n; j++)
        tmp[j] = distBuf[i].buf[j];

    // Insertion sort (n <= 100, runs once per TX interval)
    for (uint8_t j = 1; j < n; j++) {
        uint16_t key = tmp[j];
        int8_t   k   = (int8_t)j - 1;
        while (k >= 0 && tmp[k] > key) {
            tmp[k + 1] = tmp[k];
            k--;
        }
        tmp[k + 1] = key;
    }

    uint8_t idx_95 = (uint8_t)(((uint16_t)n * 95u) / 100u);
    if (idx_95 >= n) idx_95 = n - 1;  // clamp (handles n=1)
    return tmp[idx_95];
}

void DistanceBuffer_Clear(uint8_t idx)
{
    distBuf[idx].count = 0;
}

/**
 * Get the number of valid items currently stored in the buffer.
 */
uint8_t DistanceBuffer_GetCount(uint8_t i)
{
    return distBuf[i].count;
}

/**
 * Startup sensor probe: select each MUX port in turn, wait up to
 * PROBE_TIMEOUT_MS for a valid SEN0524 frame to arrive, then record
 * which ports have live sensors.  Populates sensor_present[],
 * active_sensor_idx[], and num_sensors_present.
 */
void probe_sensors(void)
{
    num_sensors_present = 0;
    my_printf("Probing sensor ports...\n\r");

    for (uint8_t port = 1; port <= 4; port++) {
        select_distance_sensor(port);
        HAL_Delay(100);   // let MUX settle before expecting frames
        tof_frame_seen = 0;

        uint32_t t_start = HAL_GetTick();
        while ((HAL_GetTick() - t_start) < PROBE_TIMEOUT_MS) {
            HAL_IWDG_Refresh(&hiwdg);
            if (tof_frame_seen) {   // any structurally-valid frame == sensor alive
                break;
            }
        }

        if (tof_frame_seen) {
            sensor_present[port - 1] = 1;
            sensor_miss[port - 1]    = 0;
            active_sensor_idx[num_sensors_present] = port - 1;
            num_sensors_present++;
            my_printf("  Port %d: FOUND\n\r", port);
        } else {
            sensor_present[port - 1] = 0;
            my_printf("  Port %d: not detected\n\r", port);
        }

        tof_frame_seen = 0;
    }

    my_printf("Probe complete: %d sensor(s) found.\n\r", num_sensors_present);

    // If nothing found, fall back to port 1 so the system still runs
    if (num_sensors_present == 0) {
        my_printf("WARNING: no sensors detected - defaulting to port 1\n\r");
        sensor_present[0]      = 1;
        active_sensor_idx[0]   = 0;
        num_sensors_present    = 1;
    }
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

/**
 * Re-probe sensor ports at runtime.  Detects liveness via a structurally-valid
 * frame (tof_frame_seen), NOT a positive distance, so a sensor that is alive but
 * currently reading invalid (fog, empty trough, dead-zone) is not mistaken for
 * absent.  A known sensor is only dropped after DROP_AFTER_MISSES consecutive
 * failed reprobes (hysteresis), so a single unlucky probe window can't prune it.
 * Never falls back to port 1 on total failure (keeps the old list).  Resets the
 * TX rotation and sample buffers afterwards.
 */
void reprobe_sensors(void)
{
    uint8_t old_present[4];
    for (uint8_t i = 0; i < 4; i++) old_present[i] = sensor_present[i];

    my_printf("Re-probing sensor ports...\n\r");

    uint8_t new_present[4] = {0, 0, 0, 0};

    for (uint8_t port = 1; port <= 4; port++) {
        uint8_t i = port - 1;
        select_distance_sensor(port);
        HAL_Delay(100);
        tof_frame_seen = 0;

        uint32_t t_start = HAL_GetTick();
        while ((HAL_GetTick() - t_start) < PROBE_TIMEOUT_MS) {
            HAL_IWDG_Refresh(&hiwdg);
            if (tof_frame_seen)
                break;
        }

        if (tof_frame_seen) {
            // Sensor responded this round -> present, miss counter cleared.
            new_present[i] = 1;
            sensor_miss[i] = 0;
            if (!old_present[i])
                my_printf("  Port %d: GAINED\n\r", port);
            else
                my_printf("  Port %d: OK\n\r", port);
        } else if (old_present[i]) {
            // Known sensor missed this round. Keep it (sticky) until it has
            // missed DROP_AFTER_MISSES reprobes in a row, then drop it.
            sensor_miss[i]++;
            if (sensor_miss[i] >= DROP_AFTER_MISSES) {
                new_present[i] = 0;
                my_printf("  Port %d: LOST (after %d misses)\n\r", port, sensor_miss[i]);
            } else {
                new_present[i] = 1;
                my_printf("  Port %d: stale (%d/%d misses)\n\r",
                          port, sensor_miss[i], DROP_AFTER_MISSES);
            }
        } else {
            new_present[i] = 0;   // was absent, still absent
        }

        tof_frame_seen = 0;
    }

    // Rebuild the active-sensor list (compacted port indices) from new_present.
    uint8_t new_active[4] = {0, 0, 0, 0};
    uint8_t new_count     = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (new_present[i])
            new_active[new_count++] = i;
    }

    if (new_count > 0) {
        for (uint8_t i = 0; i < 4; i++) sensor_present[i]    = new_present[i];
        for (uint8_t i = 0; i < 4; i++) active_sensor_idx[i] = new_active[i];
        num_sensors_present = new_count;
    } else {
        my_printf("  No sensors found -- keeping old list\n\r");
    }

    // Clear all buffers, reset TX rotation, point MUX at first active sensor
    for (uint8_t i = 0; i < 4; i++) DistanceBuffer_Clear(i);
    distance_sensor_packet_ready = 0;
    tx_slot       = 0;
    cur_dist_slot = 0;
    if (num_sensors_present > 0)
        select_distance_sensor(active_sensor_idx[0] + 1);

    my_printf("Re-probe complete: %d sensor(s).\n\r", num_sensors_present);
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

    // Send measurement command. Use a finite timeout (matching the receive
    // below) so a missing/stuck sensor can't block the main loop indefinitely.
    HAL_I2C_Master_Transmit(&hi2c1, SHT85_ADDR, cmd, 2, 100);

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

        if (temperature < 0)
            int_temperature = ((int8_t)float_to_abs_u8(temperature)) | 0x80;
        else
            int_temperature = ((int8_t)temperature) & 0x7F;
        int_humidity = (int8_t)humidity;

        sht85_ok = 1;
    }
    else
    {
        sht85_ok = 0;
        my_printf("SHT85 read failed - no sensor?\n\r");
    }

}

#define DISABLE_USART1 USART1->CR1&=~USART_CR1_UE;
#define ENABLE_USART1  USART1->CR1|=USART_CR1_UE;

void blink_comm_LED(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        HAL_GPIO_WritePin(G_COMM_GPIO_Port, G_COMM_Pin, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(G_COMM_GPIO_Port, G_COMM_Pin, GPIO_PIN_RESET);
        HAL_Delay(150);
    }
}

void sensors_data_sending_to_LoRA_concentrator()
{
	// TX rotation slot layout:
	//   0          -> temperature
	//   1          -> humidity
	//   2 .. 2+N-1 -> distance sensors (active_sensor_idx[0..N-1])
	//
	// After transmitting a distance slot the MUX is moved to the next distance
	// sensor so it has the full inter-TX interval (~10 s) to accumulate samples.
	// The buffer of the sensor just sent is cleared so fresh samples are taken.

	uint8_t  num_slots   = 2 + num_sensors_present;   // temp + hum + N distance
	uint16_t value       = 0;
	uint8_t  type        = 0;
	uint8_t  skip_tx     = 0;  // set when a slot has no valid data to send

	if (tx_slot == 0) {
		// Temperature
		type = 1;
		if (!sht85_ok) {
			my_printf("Sensor TX: SHT85 absent, skipping temperature slot\n\r");
			skip_tx = 1;
		} else {
			if ((int_temperature & 0x80) > 0)
				my_printf("Sensor TX to LoRA temperature:-%d [C]\n\r", int_temperature & 0x7F);
			else
				my_printf("Sensor TX to LoRA temperature:%d [C]\n\r", int_temperature & 0x7F);
			value = (uint16_t)(int_temperature & 0xFF);
		}

	} else if (tx_slot == 1) {
		// Humidity
		type = 2;
		if (!sht85_ok) {
			my_printf("Sensor TX: SHT85 absent, skipping humidity slot\n\r");
			skip_tx = 1;
		} else {
			my_printf("Sensor TX to LoRA humidity:%d [%%]\n\r", int_humidity);
			value = (uint16_t)(uint8_t)int_humidity;
		}

	} else {
		// Distance sensor (slot 2 .. 2+N-1)
		uint8_t dist_slot = tx_slot - 2;              // 0-based index into active sensors
		uint8_t idx       = active_sensor_idx[dist_slot];  // 0-based port index

		type  = 0;
		uint16_t p95 = DistanceBuffer_Get95thPercentile(idx);
		// An empty window just means no VALID measurements this interval (e.g.
		// the sensor was reading invalid). Detection/removal is handled by the
		// periodic reprobe with hysteresis, so do NOT force an out-of-schedule
		// reprobe here -- doing so churned the active list and cleared every
		// sensor's buffer on each empty window, dragging healthy sensors down too.
		my_printf("Sensor TX to LoRA port %d dist 95pct:%u mm (n=%u samples)\n\r",
		          idx + 1, p95, DistanceBuffer_GetCount(idx));
		value = p95;

		// Blink COMM LED to indicate distance range
		if (value == 0)
			blink_comm_LED(1);
		else if (value < 1000)
			blink_comm_LED(2);
		else
			blink_comm_LED(3);

		// Clear this sensor's buffer - fresh accumulation starts now
		DistanceBuffer_Clear(idx);

		// Switch MUX to the NEXT distance sensor immediately so it gets
		// the maximum collection time before its slot comes up
		uint8_t next_dist_slot = (dist_slot + 1) % num_sensors_present;
		uint8_t next_idx       = active_sensor_idx[next_dist_slot];
		select_distance_sensor(next_idx + 1);
		cur_dist_slot = next_dist_slot;  // keep main loop in sync
	}

	if (!skip_tx) {
		uint32_t data = encode_message(SENSOR_DEVICE_ID, type, value);
		send_data_to_LoRA_network(data);
	}

	// Advance rotation
	tx_slot++;
	if (tx_slot >= num_slots)
		tx_slot = 0;
}



// Decode a complete 16-byte SEN0524 frame.
//   *distance_mm_out <- decoded distance (mm), clamped to >= 0
//   *meas_valid_out  <- 1 if the sensor flags the measurement valid (dis_status==0),
//                       0 if invalid. A structurally-sound frame carrying an invalid
//                       measurement still returns 0 (success): the sensor is alive,
//                       the reading just isn't usable. Liveness vs. usable-sample is
//                       decided by the caller, not by dropping the frame here.
// Returns 0 on a structurally-valid frame (good header + checksum),
//        -1 on bad header/function mark, -2 on checksum error.
int decode_SEN0524(uint8_t *buf, uint16_t *distance_mm_out, uint8_t *meas_valid_out)
{
    if (buf[0] != SEN0524_FRAME_HEADER || buf[1] != SEN0524_FUNCTION_MARK)
    {
        set_error_LED(1);
        return -1;
    }

    // Verify checksum: lowest byte of sum of bytes [0..14]
    uint16_t sum = 0;
    for (int i = 0; i < 15; i++)
        sum += buf[i];
    if ((sum & 0xFF) != buf[15])
        return -2; // checksum mismatch

    // dis_status (buf[11]) is intentionally ignored: the two SEN0524 sensor
    // generations in the field use opposite conventions (one reports 0 on a
    // valid measurement, the other reports 1), so the flag is not portable.
    // Any frame that passes the header + checksum checks is treated as a usable
    // sample; out-of-range / noisy readings are rejected downstream by the 95th
    // percentile filter instead.
    (void)buf[11];
    *meas_valid_out = 1;

    // Distance encoded as int24 LE in bytes [8..10], unit = metres * 1000 (mm)
    uint32_t raw24 = (uint32_t)buf[8] | ((uint32_t)buf[9] << 8) | ((uint32_t)buf[10] << 16);
    // Sign-extend 24-bit to 32-bit
    int32_t dist_mm_i32;
    if (raw24 & 0x800000)
        dist_mm_i32 = (int32_t)(raw24 | 0xFF000000);
    else
        dist_mm_i32 = (int32_t)raw24;

    // raw24 is already metres * 1000, i.e. millimetres
    if (dist_mm_i32 < 0)
        dist_mm_i32 = 0;

    *distance_mm_out = (uint16_t)(dist_mm_i32 > 65535 ? 65535 : dist_mm_i32);
    return 0;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    uint8_t b = tof_rx_byte;

    switch (tof_rx_state)
    {
    case TOF_STATE_WAIT_HEADER:
        if (b == SEN0524_FRAME_HEADER)
        {
            tof_frame_buf[0] = b;
            tof_frame_idx = 1;
            tof_rx_state = TOF_STATE_WAIT_FUNC;
        }
        break;

    case TOF_STATE_WAIT_FUNC:
        if (b == SEN0524_FUNCTION_MARK)
        {
            tof_frame_buf[1] = b;
            tof_frame_idx = 2;
            tof_rx_state = TOF_STATE_COLLECT;
        }
        else if (b == SEN0524_FRAME_HEADER)
        {
            // A new header arrived where the function mark was expected:
            // restart the frame here instead of discarding the byte.
            tof_frame_buf[0] = b;
            tof_frame_idx = 1;
            // stay in WAIT_FUNC
        }
        else
        {
            tof_rx_state = TOF_STATE_WAIT_HEADER;
            tof_frame_idx = 0;
        }
        break;

    case TOF_STATE_COLLECT:
        // A header byte mid-frame most likely means the previous frame was
        // truncated; treat it as the start of a new frame and resync.
        if (b == SEN0524_FRAME_HEADER && tof_frame_idx >= 3)
        {
            tof_frame_buf[0] = b;
            tof_frame_idx = 1;
            tof_rx_state = TOF_STATE_WAIT_FUNC;
        }
        else
        {
            tof_frame_buf[tof_frame_idx++] = b;

            if (tof_frame_idx >= SEN0524_FRAME_LEN)
            {
                // Full frame collected
                uint16_t d          = 0;
                uint8_t  meas_valid = 0;
                if (decode_SEN0524(tof_frame_buf, &d, &meas_valid) == 0)
                {
                    // Structurally-valid frame: the sensor is alive and streaming.
                    tof_frame_seen = 1;
                    last_valid_packet_tick = HAL_GetTick();

                    // meas_valid is always 1 now (the per-generation dis_status
                    // flag is ignored in decode_SEN0524); the guard is kept so
                    // sample collection stays decoupled from liveness detection.
                    if (meas_valid)
                    {
                        distance_mm = d;
                        distance_sensor_packet_ready = 1;
                    }
                }
                // Reset for the next frame
                tof_rx_state = TOF_STATE_WAIT_HEADER;
                tof_frame_idx = 0;
            }
        }
        break;

    default:
        tof_rx_state = TOF_STATE_WAIT_HEADER;
        tof_frame_idx = 0;
        break;
    }

    // Re-arm for next byte
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&tof_rx_byte, 1);
}

uint32_t global_100ms_counter=0;
//timer period elapsed callback function
//Period elapsed callback in non blocking mode

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t divider_counter = 0;
    if(htim == &htim6)
    {	// TIM6 is configured by CubeMX but not started; reserved for future use.

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

// Read back the configuration registers of a 433 MHz LoRa module.
// Kept for boards using the 433 MHz module variant; the default 868 MHz
// module does not use this, so it is not called in the active firmware.
int read_LoRA_433MHZ_module_config_data()
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
	uint32_t tick_counter;
	uint16_t print_out_cnt      = 0;
	uint32_t last_tx_100ms      = 0xFFFFFFFF;
	uint32_t last_reprobe_100ms = 0;
	uint32_t last_frame_ms      = 0;  // tick of the last frame consumed by the main loop
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

  // --- Flush UART2 RX and start byte-by-byte interrupt reception ---
  volatile uint32_t tmp;
  tmp = huart2.Instance->RDR;
  (void)tmp;
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  tof_rx_state  = TOF_STATE_WAIT_HEADER;
  tof_frame_idx = 0;
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&tof_rx_byte, 1);

  // --- Auto-detect which sensor ports have live sensors ---
  probe_sensors();
  last_reprobe_100ms = global_100ms_counter;

  // Point MUX at the first active distance sensor; it will accumulate samples
  // until the first TX event fires.
  if (num_sensors_present > 0)
      select_distance_sensor(active_sensor_idx[cur_dist_slot] + 1);
  tick_counter   = HAL_GetTick();
  last_frame_ms  = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ((HAL_GetTick() - tick_counter) > 1000)
	  {
		  tick_counter = HAL_GetTick();
		  toggle_LED_WRK_indicator();
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Stream liveness: any structurally-valid frame (valid OR invalid
	  // measurement) means the sensor is connected and talking. Keyed off
	  // tof_frame_seen so an all-invalid stream doesn't false-trip the fault.
	  if (tof_frame_seen)
	  {
		  tof_frame_seen = 0;
		  set_error_LED(0);
		  last_frame_ms = HAL_GetTick();
	  }
	  else if (HAL_GetTick() - last_frame_ms > NO_FRAME_TIMEOUT_MS)
	  {
		  // No frames at all for a while: sensor likely disconnected. Flag the
		  // fault and let the reprobe / resync logic recover. Time-based, not
		  // iteration-based, since the main loop spins far faster than frames
		  // arrive (~10 Hz).
		  my_printf("Error: no frame from distance sensor.\n\r");
		  set_error_LED(1);
	  }

	  // Accumulate usable samples into the current distance sensor's buffer.
	  // Only frames with a valid measurement reach here (set in the RX callback).
	  if (distance_sensor_packet_ready)
	  {
		  distance_sensor_packet_ready = 0;

		  // cur_dist_slot tracks which active sensor port the MUX is on.
		  // sensors_data_sending_to_LoRA_concentrator() advances it (and the
		  // MUX) each time a distance slot is transmitted, so here we just
		  // need to know which sensor is currently collecting.
		  uint8_t cur_idx = active_sensor_idx[cur_dist_slot];
		  DistanceBuffer_Add(cur_idx, distance_mm);

		  if ((print_out_cnt++) > 10)
		  {
			  my_printf("distance:%u [mm] port %d (n=%u)\n\r",
			            distance_mm, cur_idx + 1, DistanceBuffer_GetCount(cur_idx));
			  print_out_cnt = 0;
		  }
	  }

	  HAL_IWDG_Refresh(&hiwdg);

	  // UART overrun / resync watchdog
	  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
		  __HAL_UART_CLEAR_OREFLAG(&huart2);
		  tof_rx_state = TOF_STATE_WAIT_HEADER;
		  tof_frame_idx = 0;
	  }

	  // Resync only when a partial frame has been sitting half-received for a
	  // while (a real stall). Don't blanket-reset the idle WAIT_HEADER state:
	  // at low frame rates that risked clobbering a frame that had just begun.
	  if (tof_frame_idx > 0 && HAL_GetTick() - last_valid_packet_tick > 800) {
		  tof_rx_state = TOF_STATE_WAIT_HEADER;
		  tof_frame_idx = 0;
		  last_valid_packet_tick = HAL_GetTick();
	  }

	  // LoRa TX every 10 s
	  if (global_100ms_counter % SEND_TO_LORA_NET_EVERY_10_SEC == 0
	      && global_100ms_counter != last_tx_100ms)
	  {
	      last_tx_100ms = global_100ms_counter;
		  // Read T/H every interval - needed whenever a temp/hum slot fires
		  SHT85_Read();
		  my_printf("TX to LoRA concentrator at: %d [s]\n\r", global_100ms_counter / 10);
		  sensors_data_sending_to_LoRA_concentrator();
		  // cur_dist_slot is updated inside sensors_data_sending_to_LoRA_concentrator()
		  // whenever it switches the MUX, so no sync is needed here.
	  }

	  // Periodic / on-demand sensor re-probe
	  if (reprobe_requested ||
	      (global_100ms_counter - last_reprobe_100ms) >= REPROBE_INTERVAL_100MS)
	  {
	      reprobe_sensors();
	      last_reprobe_100ms = global_100ms_counter;
	      reprobe_requested  = 0;
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
  huart2.Init.BaudRate = 115200;
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
