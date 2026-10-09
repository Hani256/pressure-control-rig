/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>          // snprintf: builds text strings
#include "i2c.h"            // hi2c1, the I2C1 handle CubeMX created
#include "tim.h"            // htim1 (fan PWM) and htim2 (tach capture)
#include "ssd1306.h"        // OLED driver
#include "ssd1306_fonts.h"  // OLED fonts
#include "semphr.h"         // mutexes: two tasks now share the I2C bus
#include <math.h>           // fabsf, for the settled-state check
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// PID fan controller. Tune these.
#define TEMP_SETPOINT_C  23.5f   // target temperature: above this the fan starts
#define PID_KP           20.0f   // P: % of fan speed per degree above the setpoint
#define PID_KI            0.5f   // I: % added per second, per degree above the setpoint
#define PID_KD            5.0f   // D: % per (degree per second) of rising temperature
#define PID_DT            1.0f   // control period in seconds (FanTask runs once per second)
#define FAN_MIN_DUTY      5      // below this the fan stops anyway, so treat it as off

// BMP280 pressure sensor, on the same I2C1 bus as the AHT20 and the OLED.
#define BMP280_ADDR      (0x77 << 1)   // HAL wants the 8-bit form, so shift the 7-bit address left by one
#define BMP280_BASE_N    100           // samples averaged at startup to get the ambient baseline

// Pressure loop and step test
#define PRESS_HZ         20            // pressure sampling rate. The BMP280 gives about 25 Hz at 16x oversampling
#define PRESS_PERIOD_MS  (1000 / PRESS_HZ)
#define STEP_SECONDS     16            // 11 steps x 16 s = 176 s, which fits inside the 180 s log.
                                       // At 20 s the staircase outran the buffer and the last two
                                       // steps, the repeatability check, were lost.
#define LOG_DECIM        2             // log every 2nd sample, so 10 Hz. Plenty for a plant this slow.
#define LOG_N            1800          // 1800 samples at 10 Hz = 180 seconds, 36 KB of RAM

// ---- hybrid controller demo, mode 2 ----
// The sensor is deliberately declared invalid inside this band, the way the real product
// loses its reading outside the sensor's range. We still record the true value, so the
// estimate error during the blind phase becomes a measured number instead of a claim.
// The band must cover the path between the two targets and nothing else. In the previous run it
// reached up to 26 Pa while the high target settled at 24, so the loop parked inside its own blind
// band: it could not close the loop there and could not learn there either. Both edges moved down.
#define BLIND_LO         17.0f         // Pa, lower edge of the artificial blind band
#define BLIND_HI         28.0f         // Pa, upper edge. The band sits 7 Pa clear of each target.
#define BLIND_HYST        1.5f         // Pa, so a noisy signal at the edge cannot chatter

#define SP_LOW           10.0f         // Pa, target below the blind band
#define SP_HIGH          35.0f         // Pa, target above it. Measured staircase: 100 % duty gives
                                       // 56.9 Pa, so 35 Pa lands near 66 % duty and the fan never pins.
                                       // SP_LOW sits at about 28 % duty, deliberately above the fan's
                                       // start-up transient: below roughly 25 % the tach shows the fan
                                       // kicking to break away and then settling, and the model does
                                       // not describe that.
#define SP_HOLD_S        18            // seconds held at each target, long enough to settle after a blind crossing
#define SP_CYCLES         5            // 5 x 2 x 18 s = 180 s = 1800 samples at 10 Hz

#define CTRL_DT          (PRESS_PERIOD_MS / 1000.0f)   // 0.05 s, the pressure loop period
// Measured from the first hybrid run, not from the staircase. The staircase never turned the
// fan fully off, so it hid the real asymmetry: the fan pushes air in actively, but the box can
// only empty through the leak. Rising is fast, falling is slow, by a factor of about 2.7.
#define TAU_UP            2.3f         // s, rising. NOT the 2.06 s the staircase measured, and
                                       // that gap is the point. The staircase steps duty between
                                       // levels with the fan still running. The closed loop visits
                                       // different operating points: on a descent the duty dives to
                                       // about 15 % and the box vents almost freely, which is a much
                                       // faster regime than any staircase step. Fitted against the
                                       // logged runs the estimator wants 2.3 up and 1.3 down, and
                                       // that cuts its blind-range error by 38 % on both runs.
#define TAU_DN            1.3f         // s, falling
#define EST_CORRECT       0.30f        // how hard the estimate is pulled towards the measurement when valid
#define CTRL_KP           0.8f         // percent of duty per pascal of error. Loop gain 0.8 x 0.6
                                       // is about 0.5, well inside the stable region.
#define CTRL_KI           0.35f        // percent per pascal per second. With a plant gain near
                                       // 0.6 Pa/% the integral time is about 4.8 s, comfortably
                                       // slower than the 2.0 s plant lag. At 1.5 the integral time
                                       // was 1.3 s, FASTER than the plant, and the loop answered
                                       // with a limit cycle: duty slamming 100 to 5 and the pressure
                                       // swinging plus and minus 7 Pa around the target.
#define CTRL_I_LIMIT     35.0f         // clamp on the integral term, percent. The box drifted far
                                       // enough from the table that 20 was not enough to reach the
                                       // low setpoint at all, and the loop sat against the clamp.
#define DUTY_RATE_LIMIT   2.0f         // max percent change per control step, 40 %/s. Softer ramp,
                                       // less overshoot, cleaner approach to the target.

// ---- online model adaptation ----
// The table was measured on this box during the staircase run, and the box has drifted since.
// Fitting the logged runs gives roughly  pressure = 1.15 x table(duty) - 21 Pa: the curve has
// shifted, not just shrunk, because the leak grew. Below about 50 % duty the box now produces
// almost nothing. A gain alone cannot describe that, which is why the offset has to be learned
// too. MODEL_SCALE stays at 1.0: the table is used exactly as the staircase measured it.
#define MODEL_SCALE       1.00f        // use the table exactly as the staircase measured it
#define ADAPT_ALPHA       0.12f        // learning rate, fitted against two logged runs. Below this
                                       // it converges too slowly to show inside 180 s, above it the
                                       // extra speed buys nothing.
#define ADAPT_RESID_MAX  12.0f         // Pa. THE guard. A residual bigger than this is not the model
                                       // being wrong, it is the STATE being wrong: the box caught
                                       // mid-decay, moving slowly enough to pass every settled test
                                       // while sitting 25 Pa from where that duty would hold it.
                                       // Measured: samples with residuals of +15 and +18 Pa dragged
                                       // b to +12.7, which made the feedforward ask for zero duty at
                                       // the low setpoint. The fan switched off, the box coasted
                                       // inside the blind band, and the loop could then neither
                                       // control nor learn. Rejecting those samples alone takes the
                                       // final model error from 18.6 Pa back to 2.9 Pa.
#define ADAPT_MIN_DUTY   10.0f         // below this the fan barely moves, the data is useless
#define ADAPT_MAX_DUTY   99.0f         // at the limit the actuator, not the model, sets the pressure
#define ZERO_RATE         0.5f         // Pa/s. Below this the box counts as no longer moving.
#define ZERO_QUIET_S      3             // and it has to stay that way for this many seconds
#define ZERO_MAX_S       25             // give up and zero anyway after this, so a run always starts
#define RATE_WIN         (PRESS_HZ / 2)  // 0.5 s baseline for the derivative, see below
#define ADAPT_STEADY_RATE 1.0f         // Pa/s. Settled means the pressure has STOPPED MOVING.
                                       // The old test asked how far the pressure sat from its own
                                       // running average, which is a different question: a slow
                                       // oscillation sits far from its average most of the time.
                                       // That threw away every sample at the high setpoint, so the
                                       // learner only ever saw duty 46 to 54 and had nothing to
                                       // separate the gain from the offset with.
#define ADAPT_STEADY_D    2.0f         // percent, the duty must also have settled
#define ADAPT_STEADY_TICKS ((3 * PRESS_HZ) / 2)  // and it must STAY settled for 1.5 s (30 ticks at 20 Hz).
                                       // dP/dt passes through zero at the top of every oscillation
                                       // too. Without this dwell the learner treats the peak of a
                                       // limit cycle as a steady state and fits the model to it.
#define K_MIN             0.6f         // tighter rails: the learner should be trimming a
#define K_MAX             1.6f         // model, not replacing it. If it wants to leave this
#define B_LIMIT           8.0f         // range, the data is wrong, not the model.
#define ADAPT_PE_MIN      4.0f         // Pa of spread the regressor must show before we trust
                                       // BOTH parameters. Below this we are looking at one
                                       // operating point, where a gain and an offset cannot be
                                       // told apart, so we only move the offset.
#define ADAPT_SCALE      25.0f         // scales the regressor so both parameters learn at the same speed

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile float temperature_c = 0; // latest temperature in C
volatile float humidity_pct = 0;  // latest humidity in %

volatile uint8_t  fan_duty_pct = 0;  // current fan speed, 0-100 %, set automatically from the temperature
volatile uint32_t fan_rpm = 0;       // measured fan speed
volatile uint32_t tach_pulses = 0;   // pulses counted by the TIM2 interrupt since the last reading

volatile float   pressure_pa = 0;    // absolute pressure right now, in pascal
volatile float   p0_pa       = 0;    // ambient baseline, measured once at startup with the fan off
volatile float   dp_pa       = 0;    // pressure above ambient: this is what the control loop cares about
volatile uint8_t bmp_ok      = 0;    // 1 if the BMP280 answered at startup

// Calibration coefficients. Every BMP280 chip has its own set, burned in at the factory.
static uint16_t dig_T1, dig_P1;                       // these two are unsigned
static int16_t  dig_T2, dig_T3;                       // temperature coefficients
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5;       // pressure coefficients
static int16_t  dig_P6, dig_P7, dig_P8, dig_P9;       // ...continued
static int32_t  t_fine;                               // temperature result carried into the pressure maths

SemaphoreHandle_t i2c_mutex;         // guards I2C1: PressureTask, SensorTask and the OLED all use it

volatile int16_t manual_duty  = -1;  // -1 = the PID decides. 0 to 100 = forced duty, for testing
volatile uint8_t test_start   = 0;   // 1 = run the staircase, 2 = run the hybrid controller demo
static   float   T_slow = 0.0f, T_spread = 0.0f;  // regressor mean and spread, for the PE gate
volatile uint8_t adapt_on     = 1;   // 0 = fixed model, 1 = learn the model while the sensor is visible
volatile float   k_init       = 1.0f; // the model a run STARTS from. Set this in the debugger to
volatile float   b_init       = 0.0f; // hand the controller a deliberately stale model, the way a
                                      // unit shipped with another unit's calibration would be, and
                                      // then watch whether the learner finds its way back.
volatile float   k_unit       = 1.0f; // learned gain correction for this unit
volatile float   b_unit       = 0.0f; // learned offset correction, pascal
volatile uint8_t test_running = 0;   // 1 while the staircase is running
volatile uint16_t log_count   = 0;   // how many samples in log_buf are valid

// The duty steps of the test, in percent. Up, then down, then one hard on and off.
static const uint8_t step_table[] = { 0, 30, 50, 70, 100, 70, 50, 30, 0, 100, 0 };
#define STEP_COUNT (sizeof(step_table) / sizeof(step_table[0]))

// One logged sample, exactly 16 bytes so the python decoder stays simple.
// The staircase test leaves sp and est at zero and valid at 1.
typedef struct {
  uint32_t t_ms;      // milliseconds since the run started
  int16_t  sp_x10;    // setpoint, tenths of a pascal  (mode 2 only)
  int16_t  dp_x10;    // TRUE measured pressure above ambient, tenths of a pascal
  int16_t  est_x10;   // what the controller believed, tenths of a pascal  (mode 2 only)
  uint16_t rpm;       // measured fan speed
  int16_t  temp_x10;  // temperature in tenths of a degree
  int16_t  k_x1000;   // learned gain, 1.000 stored as 1000
  int16_t  b_x10;     // learned offset, tenths of a pascal
  uint8_t  duty;      // commanded duty, 0 to 100
  uint8_t  valid;     // 1 = the controller could see the sensor, 0 = blind
} log_sample_t;       // 20 bytes

log_sample_t log_buf[LOG_N];         // 24 KB in RAM. Dumped from the debugger after the test.
/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void SensorTask(void *argument);    // our own task: reads the AHT20 and draws on the OLED
void FanTask(void *argument);       // our own task: drives the fan PWM and measures its RPM
static uint8_t BMP280_Init(void);        // loads the calibration and starts the pressure sensor
static float   BMP280_ReadPressure(void);// one pressure reading, in pascal
void PressureTask(void *argument);       // 20 Hz pressure loop and the step test
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  // Created before any task starts, so nobody can touch I2C1 before the guard exists.
  i2c_mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Create our task with the native FreeRTOS API.
  // 512 = stack size in words (2 KB); priority 1 = just above the idle task.
  xTaskCreate(SensorTask, "sensor", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
  // Fan task: small stack (256 words = 1 KB), higher priority (2) because it is timing-related.
  xTaskCreate(FanTask, "fan", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
  // Pressure task: highest of the three, because a control loop must not be delayed by the display.
  xTaskCreate(PressureTask, "press", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // blink = "system is alive"
    vTaskDelay(pdMS_TO_TICKS(500));         // sleep 500 ms, CPU free for other tasks
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// Read the 24 calibration bytes, configure the sensor and start it measuring.
// Returns 1 on success, 0 if the chip did not answer.
static uint8_t BMP280_Init(void)
{
  uint8_t id = 0;                                                            // chip id register
  uint8_t cal[24];                                                           // raw calibration bytes
  uint8_t v;                                                                 // one byte to write

  // Register 0xD0 holds a fixed id. A BMP280 answers 0x58. Anything else means wrong chip or wrong address.
  if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xD0, 1, &id, 1, 100) != HAL_OK) return 0;
  if (id != 0x58) return 0;

  // Calibration starts at register 0x88 and is 24 bytes long, little endian, low byte first.
  if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0x88, 1, cal, 24, 100) != HAL_OK) return 0;

  dig_T1 = (uint16_t)(cal[1]  << 8 | cal[0]);    // unsigned
  dig_T2 = (int16_t) (cal[3]  << 8 | cal[2]);    // signed from here on
  dig_T3 = (int16_t) (cal[5]  << 8 | cal[4]);
  dig_P1 = (uint16_t)(cal[7]  << 8 | cal[6]);    // unsigned again
  dig_P2 = (int16_t) (cal[9]  << 8 | cal[8]);
  dig_P3 = (int16_t) (cal[11] << 8 | cal[10]);
  dig_P4 = (int16_t) (cal[13] << 8 | cal[12]);
  dig_P5 = (int16_t) (cal[15] << 8 | cal[14]);
  dig_P6 = (int16_t) (cal[17] << 8 | cal[16]);
  dig_P7 = (int16_t) (cal[19] << 8 | cal[18]);
  dig_P8 = (int16_t) (cal[21] << 8 | cal[20]);
  dig_P9 = (int16_t) (cal[23] << 8 | cal[22]);

  // Register 0xF5 (config): standby 0.5 ms, IIR filter x16. The filter is what buys us the low noise.
  v = (0 << 5) | (4 << 2);
  if (HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, 0xF5, 1, &v, 1, 100) != HAL_OK) return 0;

  // Register 0xF4 (ctrl_meas): temperature oversampling x2, pressure oversampling x16, normal mode.
  // Normal mode means the chip keeps measuring on its own; we just read the newest result when we want it.
  v = (2 << 5) | (5 << 2) | 3;
  if (HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, 0xF4, 1, &v, 1, 100) != HAL_OK) return 0;

  vTaskDelay(pdMS_TO_TICKS(50));   // let the first conversion finish before anyone reads it
  return 1;
}

// Read the raw registers and turn them into pascal, using the formulas from the Bosch datasheet.
// All integer maths, so it works with newlib-nano and costs no floating point in the loop.
static float BMP280_ReadPressure(void)
{
  uint8_t raw[6];                                                   // 3 bytes pressure, 3 bytes temperature
  int32_t adc_P, adc_T;                                             // the 20-bit raw readings
  int64_t var1, var2, p;                                            // 64 bit is required here, do not shrink it

  // 0xF7 to 0xFC in one burst: press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb.
  // Reading them together guarantees pressure and temperature come from the same measurement.
  if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xF7, 1, raw, 6, 100) != HAL_OK) return pressure_pa;

  adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);   // 20 bits, left aligned
  adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);   // same shape

  // Temperature first, because it produces t_fine, which the pressure formula needs.
  var1 = (((adc_T >> 3) - ((int32_t)dig_T1 << 1)) * (int32_t)dig_T2) >> 11;
  var2 = ((((adc_T >> 4) - (int32_t)dig_T1) * ((adc_T >> 4) - (int32_t)dig_T1)) >> 12);
  var2 = (var2 * (int32_t)dig_T3) >> 14;
  t_fine = (int32_t)(var1 + var2);                                  // shared with the pressure maths below

  // Pressure, straight from the datasheet. The result is Q24.8 format, meaning pascal times 256.
  var1 = (int64_t)t_fine - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = ((((int64_t)1 << 47) + var1) * (int64_t)dig_P1) >> 33;
  if (var1 == 0) return pressure_pa;                                // guard against a divide by zero
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)dig_P8 * p) >> 19;
  p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);

  return (float)p / 256.0f;                                         // Q24.8 back to plain pascal
}

void SensorTask(void *argument)
{
  uint8_t cmd[3];            // bytes we send to the sensor
  uint8_t rx[7];             // bytes we get back
  char line[20];             // one line of text for the screen

  vTaskDelay(pdMS_TO_TICKS(40));                             // AHT20 needs 40 ms after power-up

  xSemaphoreTake(i2c_mutex, portMAX_DELAY);                  // claim the bus
  HAL_I2C_Master_Receive(&hi2c1, 0x38 << 1, rx, 1, 100);     // read the AHT20 status byte
  if ((rx[0] & 0x08) == 0)                                   // bit 3 = calibrated? if not...
  {
    cmd[0] = 0xBE; cmd[1] = 0x08; cmd[2] = 0x00;             // ...the "initialize" command
    HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, cmd, 3, 100); // send it
  }
  xSemaphoreGive(i2c_mutex);                                 // release it before sleeping
  vTaskDelay(pdMS_TO_TICKS(10));                             // give the sensor 10 ms, bus free meanwhile

  xSemaphoreTake(i2c_mutex, portMAX_DELAY);                  // the OLED lives on the shared bus too
  ssd1306_Init();                                            // wake the OLED up and configure it
  xSemaphoreGive(i2c_mutex);

  for (;;)
  {
    cmd[0] = 0xAC; cmd[1] = 0x33; cmd[2] = 0x00;             // the "start measurement" command
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);                // claim the bus just for the transfer
    HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, cmd, 3, 100); // send it
    xSemaphoreGive(i2c_mutex);                               // release it: never hold a mutex across a delay

    vTaskDelay(pdMS_TO_TICKS(80));                           // measuring takes ~80 ms; the bus is free meanwhile

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);                // claim it again to collect the result
    HAL_I2C_Master_Receive(&hi2c1, 0x38 << 1, rx, 7, 100);   // read status + 5 data bytes + CRC
    xSemaphoreGive(i2c_mutex);

    if ((rx[0] & 0x80) == 0)                                 // bit 7 = busy; 0 means the data is ready
    {
      uint32_t raw_h = ((uint32_t)rx[1] << 12) | ((uint32_t)rx[2] << 4) | (rx[3] >> 4);   // 20-bit humidity
      uint32_t raw_t = (((uint32_t)rx[3] & 0x0F) << 16) | ((uint32_t)rx[4] << 8) | rx[5]; // 20-bit temperature
      humidity_pct  = raw_h * 100.0f / 1048576.0f;           // datasheet formula (1048576 = 2^20)
      temperature_c = raw_t * 200.0f / 1048576.0f - 50.0f;   // datasheet formula
    }

    int t10 = (int)(temperature_c * 10);                     // 23.4 -> 234, so we can print without floats

    int  dp10 = (int)(dp_pa * 10.0f);                        // 12.3 Pa becomes 123, same trick
    char sign = (dp10 < 0) ? '-' : ' ';                      // keep the sign, the value can go negative
    if (dp10 < 0) dp10 = -dp10;                              // work with the magnitude from here on

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);                        // the whole screen update is one bus session
    ssd1306_Fill(Black);                                             // clear the drawing buffer
    ssd1306_SetCursor(0, 0);                                         // top-left corner
    snprintf(line, sizeof(line), "T %d.%d C", t10 / 10, t10 % 10);   // e.g. "T 23.4 C"
    ssd1306_WriteString(line, Font_11x18, White);                    // draw it into the buffer
    ssd1306_SetCursor(0, 22);                                        // second line
    if (!bmp_ok)
      snprintf(line, sizeof(line), "P --");                          // sensor missing, say so instead of lying
    else if (test_running)
      snprintf(line, sizeof(line), "TEST %u", (unsigned)log_count);  // during the test, show how many samples
    else
      snprintf(line, sizeof(line), "P%c%d.%dPa", sign, dp10 / 10, dp10 % 10);  // e.g. "P 12.3Pa"
    ssd1306_WriteString(line, Font_11x18, White);                    // draw it
    ssd1306_SetCursor(0, 44);                                        // third line
    snprintf(line, sizeof(line), "F %lu %u%%", (unsigned long)fan_rpm, fan_duty_pct); // e.g. "F 1500 40%"
    ssd1306_WriteString(line, Font_11x18, White);                    // draw it
    ssd1306_UpdateScreen();                                          // send the whole buffer to the OLED
    xSemaphoreGive(i2c_mutex);                                       // done with the bus

    vTaskDelay(pdMS_TO_TICKS(1000));                         // one reading per second
  }
}

void FanTask(void *argument)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);        // start the 25 kHz PWM signal on PA8
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);      // start tach capture on PA5, interrupt on every pulse

  float integral = 0.0f;                           // I term: the error summed over time, in %
  float prev_t = 0.0f;                             // temperature one period ago, for the D term
  uint8_t first = 1;                               // no previous temperature on the first run
  uint32_t pid_duty = 0;                           // what the PID last asked for
  uint8_t  slow_tick = 0;                          // counts 100 ms ticks up to one second

  for (;;)
  {
    // Once per second: run the PID and measure the fan speed.
    if (++slow_tick >= 10)
    {
      slow_tick = 0;

      float t = temperature_c;                     // latest temperature from SensorTask
      pid_duty = 0;                                // default: fan off

      if (t != 0.0f)                               // 0.0 means SensorTask has no reading yet
      {
        if (first) { prev_t = t; first = 0; }      // avoid a huge D spike on the first sample

        float error = t - TEMP_SETPOINT_C;         // positive = too warm = needs fan
        float p = PID_KP * error;                  // P: react to how far off we are now

        integral += PID_KI * error * PID_DT;       // I: slowly build up while the error stays
        if (integral < 0.0f)   integral = 0.0f;    // anti-windup: fan can't go below 0 %...
        if (integral > 100.0f) integral = 100.0f;  // ...or above 100 %, so don't let I grow past that

        float d = PID_KD * (t - prev_t) / PID_DT;  // D: react to how fast the temperature is rising
        prev_t = t;                                // remember for next time

        float out = p + integral + d;              // PID output in %
        if (out < 0.0f)   out = 0.0f;              // clamp to what the fan can do
        if (out > 100.0f) out = 100.0f;

        pid_duty = (out < FAN_MIN_DUTY) ? 0 : (uint32_t)out;  // tiny outputs = off
      }

      taskENTER_CRITICAL();                        // block interrupts so the ISR can't change the counter mid-read
      uint32_t pulses = tach_pulses;               // take the count...
      tach_pulses = 0;                             // ...and reset it for the next second
      taskEXIT_CRITICAL();                         // interrupts back on

      fan_rpm = pulses * 30;                       // fan gives 2 pulses per turn: pulses/2 per second x 60 s
    }

    // Every 100 ms: apply the commanded duty. Fast, so a step in the test lands within 100 ms.
    uint32_t duty = (manual_duty >= 0) ? (uint32_t)manual_duty : pid_duty;  // the override wins when set
    if (duty > 100) duty = 100;                    // never go above 100 %
    fan_duty_pct = (uint8_t)duty;                  // publish it, so the OLED and the log can see it

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty * 3840 / 100);  // 3840 ticks = one full PWM period

    vTaskDelay(pdMS_TO_TICKS(100));                // 10 times per second
  }
}

// Steady state pressure for each duty, measured on this rig with the staircase test.
// Pressures are relative to the resting level, which the staircase measured as the duty 0 point.
static const float ff_duty[] = {  0.0f,  30.0f,  50.0f,  70.0f, 100.0f };   // percent
static const float ff_dp[]   = {  0.0f,  10.4f,  21.8f,  38.0f,  56.9f };   // pascal
#define FF_N (sizeof(ff_duty) / sizeof(ff_duty[0]))

// The nominal table, scaled by MODEL_SCALE so it starts out wrong on purpose.
// This is the "factory" curve: the same for every unit, and right for none of them.
static float table_dp(float d)
{
  float y;
  if (d <= ff_duty[0]) y = ff_dp[0];                           // below the table, clamp
  else
  {
    y = ff_dp[FF_N-1];                                         // above the table, clamp
    for (uint8_t i = 1; i < FF_N; i++)
    {
      if (d <= ff_duty[i])                                     // found the bracketing pair
      {
        float f = (d - ff_duty[i-1]) / (ff_duty[i] - ff_duty[i-1]);   // 0 to 1 inside this segment
        y = ff_dp[i-1] + f * (ff_dp[i] - ff_dp[i-1]);                 // straight line between the points
        break;
      }
    }
  }
  return y * MODEL_SCALE;
}

// The nominal inverse: which duty does the factory curve say I need for this pressure?
static float table_duty(float p)
{
  float q = p / MODEL_SCALE;                                   // undo the scaling to search the raw table
  if (q <= ff_dp[0]) return ff_duty[0];
  for (uint8_t i = 1; i < FF_N; i++)
  {
    if (q <= ff_dp[i])
    {
      float f = (q - ff_dp[i-1]) / (ff_dp[i] - ff_dp[i-1]);
      return ff_duty[i-1] + f * (ff_duty[i] - ff_duty[i-1]);
    }
  }
  return ff_duty[FF_N-1];
}

// The model this unit actually uses: the factory curve plus two learned numbers.
//   predicted pressure = k_unit * table(duty) + b_unit
// k_unit and b_unit are the only things learned online. Two parameters converge in a handful
// of cycles and are easy to bound and to reset, unlike a whole surface.
static float ff_dp_from_duty(float d)
{
  return k_unit * table_dp(d) + b_unit;
}

// And its inverse, used as the feedforward term.
static float ff_duty_from_dp(float p)
{
  float k = k_unit;
  if (k < K_MIN) k = K_MIN;                          // k_unit is clamped everywhere it is written,
  return table_duty((p - b_unit) / k);               // but this division is worth guarding anyway
}

// Owns the BMP280: calibrates it, samples it at a fixed rate, and runs the step test.
// Highest priority of the three tasks, because a sampling loop must not be pushed around by the display.
void PressureTask(void *argument)
{
  TickType_t next = xTaskGetTickCount();           // anchor for vTaskDelayUntil, so the period does not drift

  xSemaphoreTake(i2c_mutex, portMAX_DELAY);        // claim the bus for the whole init
  bmp_ok = BMP280_Init();                          // wake the sensor and load its calibration
  xSemaphoreGive(i2c_mutex);

  if (!bmp_ok) vTaskSuspend(NULL);                 // no sensor, nothing to do: stop this task for good

  // Ambient baseline, measured once with the fan off. Everything later is relative to this.
  float sum = 0;                                   // running total
  for (int i = 0; i < BMP280_BASE_N; i++)
  {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);      // one short bus session per sample
    sum += BMP280_ReadPressure();
    xSemaphoreGive(i2c_mutex);
    vTaskDelay(pdMS_TO_TICKS(40));                 // 40 ms matches the sensor's own output rate
  }
  p0_pa = sum / (float)BMP280_BASE_N;              // ambient pressure with the fan off

  float    p_rest = 0;                             // slow average of the pressure while the fan is off
  uint8_t  rest_ready = 0;                         // 0 until p_rest has been seeded
  uint8_t  decim = 0;                              // counts samples between logged ones
  uint32_t t0 = 0;                                 // tick count when the run started
  uint8_t  step_idx = 0;                           // which entry of step_table we are on
  uint32_t step_t0 = 0;                            // when the current step started
  uint8_t  mode = 0;                               // 1 = staircase, 2 = hybrid controller

  // hybrid controller state
  float    est      = 0;                           // what the controller believes the pressure is
  float    integ    = 0;                           // integral term, percent
  float    duty_cmd = 0;                           // the duty we are commanding, after the rate limit
  float    sp       = 0;                           // current setpoint, pascal
  uint8_t  sensor_valid = 1;                       // 0 while inside the artificial blind band
  uint8_t  sp_high  = 0;                           // which of the two targets we are on
  float    fb_applied = 0;                         // the rate limited feedback term, percent of duty
  float    duty_slow = 0;                          // slow average of the duty, for the settled check
  float    dp_slow   = 0;                          // slow average of the pressure, same
  uint16_t steady_ticks = 0;                       // consecutive quiet samples, for the dwell test
  float    p_rate    = 0;                          // filtered dP/dt, Pa per second, on absolute pressure
  float    p_hist[RATE_WIN];                       // half a second of absolute pressure, for it
  uint8_t  p_hi      = 0;                          // write index into that ring
  uint8_t  zeroing   = 0;                          // 1 while we hold the fan off and wait for quiet
  uint16_t zero_quiet = 0, zero_wait = 0;          // consecutive quiet samples, and total samples waited

  for (int i = 0; i < RATE_WIN; i++) p_hist[i] = pressure_pa;   // seed the ring, or the first
                                                   // derivative is a step from 0 to about 100000 Pa
  next = xTaskGetTickCount();                      // re-anchor after the slow baseline loop

  for (;;)
  {
    vTaskDelayUntil(&next, pdMS_TO_TICKS(PRESS_PERIOD_MS));   // fixed 50 ms period, no drift

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);      // short bus session: read and get out
    pressure_pa = BMP280_ReadPressure();           // absolute pressure, pascal
    xSemaphoreGive(i2c_mutex);

    dp_pa = pressure_pa - p0_pa;                   // how far above ambient we are right now

    // dP/dt, over a 0.5 s baseline rather than between neighbouring samples, and on the absolute
    // pressure so that moving p0_pa cannot put a step in it.
    //
    // Why the long baseline. Differencing two neighbouring samples divides the sensor noise by
    // the sample period, so 0.47 Pa at 20 Hz becomes about 13 Pa/s of noise. A single low pass
    // only got that to roughly 4 Pa/s, and a 1.5 Pa/s threshold was then met about 68 % of the
    // time at random. Instantaneous tests survived that; the moment a dwell was required, forty
    // consecutive samples all under threshold became impossible and the learner never ran once.
    // Dividing by 0.5 s instead of 0.05 s cuts the noise tenfold before any filtering.
    float p_old = p_hist[p_hi];
    p_hist[p_hi] = pressure_pa;
    p_hi = (uint8_t)((p_hi + 1) % RATE_WIN);
    p_rate += 0.1f * ((pressure_pa - p_old) * (2.0f) - p_rate);   // 2.0 = 1 / 0.5 s

    // Track the resting pressure whenever the fan is off. A sealed box drifts with temperature,
    // so the zero point moves; this follows it slowly instead of trusting one measurement from boot.
    if (fan_duty_pct == 0)
    {
      if (!rest_ready) { p_rest = pressure_pa; rest_ready = 1; }   // seed it with the first quiet sample
      else p_rest += 0.02f * (pressure_pa - p_rest);               // slow first order average
    }

    // A run is asked for. Do NOT zero yet: the box is usually still emptying from the run before,
    // and p_rest only updates while the fan is off, so it can be many seconds stale. Zeroing on a
    // box that is still moving puts a constant offset on every reading of the whole run. Measured:
    // a 21 Pa offset, which pushed the feedforward 46 points of duty away from the truth, saturated
    // the integrator at its 35 limit, and pinned the fan at 100 % for the entire high setpoint.
    if (test_start && !test_running && !zeroing)
    {
      mode        = test_start;                    // remember what was asked for
      test_start  = 0;                             // consume the request
      zeroing     = 1;                             // fan off, and wait for the box to go quiet
      zero_quiet  = 0;
      zero_wait   = 0;
      manual_duty = 0;
    }

    if (zeroing)
    {
      manual_duty = 0;                             // hold the fan off for the whole zero phase
      zero_wait++;
      if (fabsf(p_rate) < ZERO_RATE) zero_quiet++; else zero_quiet = 0;

      if (zero_quiet < (uint16_t)(ZERO_QUIET_S * PRESS_HZ) &&
          zero_wait  < (uint16_t)(ZERO_MAX_S   * PRESS_HZ))
        continue;                                  // not settled yet, come back in 50 ms

      p0_pa   = pressure_pa;                       // zero on what the sensor reads right now
      p_rest  = pressure_pa;                       // and keep the slow tracker consistent with it
      rest_ready = 1;
      dp_pa   = 0.0f;
      zeroing = 0;
      test_running = 1;
      log_count    = 0;                            // overwrite any earlier run
      step_idx     = 0;
      t0           = xTaskGetTickCount();
      step_t0      = t0;

      if (mode == 1)
      {
        manual_duty = (int16_t)step_table[0];      // staircase drives duty directly
      }
      else
      {
        est = dp_pa; integ = 0; duty_cmd = 0; fb_applied = 0;   // start from where we actually are
        k_unit = k_init;                           // start from whatever model was asked for
        b_unit = b_init;
        if (k_unit < K_MIN) k_unit = K_MIN;         // never let a typo divide the feedforward by zero
        if (k_unit > K_MAX) k_unit = K_MAX;
        if (b_unit < -B_LIMIT) b_unit = -B_LIMIT;
        if (b_unit >  B_LIMIT) b_unit =  B_LIMIT;
        T_slow = 0.0f; T_spread = 0.0f;             // the excitation tracker is file scope and would
                                                    // otherwise carry over, so run two would start
                                                    // warm and run one cold, and they must match
        duty_slow = 0; dp_slow = dp_pa; steady_ticks = 0;
        sp_high = 0; sp = SP_LOW;                  // first target
        sensor_valid = 1;
        manual_duty = 0;                           // the controller writes this every step
      }
    }

    if (test_running && mode == 2)
    {
      // ---- 1. decide whether the controller is allowed to see the sensor ----
      // Hysteresis on both edges, so noise at the boundary cannot flip the mode every sample.
      float lo = sensor_valid ? BLIND_LO : (BLIND_LO - BLIND_HYST);
      float hi = sensor_valid ? BLIND_HI : (BLIND_HI + BLIND_HYST);
      sensor_valid = !(dp_pa > lo && dp_pa < hi);

      // ---- 2. estimator: always runs, on the model ----
      // First order: the plant heads towards the steady state pressure of the current duty.
      float target_ss = ff_dp_from_duty(duty_cmd);             // where this duty would end up
      float tau = (target_ss > est) ? TAU_UP : TAU_DN;         // rising and falling are not identical
      est += (target_ss - est) * (CTRL_DT / tau);              // one step of the model forward

      if (sensor_valid)
        est += EST_CORRECT * (dp_pa - est);                    // pull the estimate towards reality

      // ---- 3. controller ----
      float err = sp - est;                                    // always on the estimate, never on raw truth
      float ff  = ff_duty_from_dp(sp);                         // feedforward: the backbone, works blind too

      // Rate limit the FEEDBACK only, and let the feedforward step freely.
      //
      // The feedforward is not a reaction to anything, it is a number the model already knows the
      // moment the setpoint changes. Rate limiting it throws that knowledge away: measured, the
      // loop crawled out of its parked duty at 40 %/s and took 1.9 s to cross the blind band
      // instead of 0.6 s. And since dead reckoning error grows at about 7 Pa per second blind,
      // the transit time, not the model, ended up deciding the score. That masked the very thing
      // the run is supposed to measure. The rate limit belongs on the feedback, which IS a
      // reaction and can be jumpy; the feedforward should land in one step.
      float fb_raw = CTRL_KP * err + integ;                    // the reacting part
      float dfb = fb_raw - fb_applied;
      if (dfb >  DUTY_RATE_LIMIT) dfb =  DUTY_RATE_LIMIT;      // smooth the reaction
      if (dfb < -DUTY_RATE_LIMIT) dfb = -DUTY_RATE_LIMIT;
      fb_applied += dfb;

      float want_raw = ff + fb_applied;                        // total command before clamping
      float want = want_raw;
      if (want < 0.0f)   want = 0.0f;                          // the fan cannot do less than off
      if (want > 100.0f) want = 100.0f;                        // or more than full

      // Anti-windup, two rules. Integrate only when the error means something (the sensor is
      // visible), and stop pushing further into a limit the actuator is already sitting on.
      uint8_t pushing_into_limit = (want_raw > 100.0f && err > 0.0f) ||
                                   (want_raw <   0.0f && err < 0.0f);
      if (sensor_valid && !pushing_into_limit)
      {
        integ += CTRL_KI * err * CTRL_DT;
        if (integ >  CTRL_I_LIMIT) integ =  CTRL_I_LIMIT;      // hard clamp as a second line of defence
        if (integ < -CTRL_I_LIMIT) integ = -CTRL_I_LIMIT;
      }

      duty_cmd = want;                                         // already smooth: only fb was limited

      manual_duty = (int16_t)duty_cmd;                         // FanTask applies it within 100 ms

      // ---- 4. learn the model, but only while the sensor is telling the truth ----
      duty_slow += 0.02f * (duty_cmd - duty_slow);             // slow average of the duty
      dp_slow   += 0.02f * (dp_pa - dp_slow);                  // kept for the display only

      if ((fabsf(p_rate) < ADAPT_STEADY_RATE) &&
          (fabsf(duty_cmd - duty_slow) < ADAPT_STEADY_D))
      { if (steady_ticks < 60000) steady_ticks++; }            // quiet for one more sample
      else steady_ticks = 0;                                   // any movement resets the clock

      uint8_t steady = (steady_ticks >= ADAPT_STEADY_TICKS);

      // Every condition here exists to stop the model learning from data that means nothing:
      // no learning while blind, while the plant is still moving, while the fan is in its dead
      // zone, or while the actuator is at a limit and the pressure is set by saturation.
      if (adapt_on && sensor_valid && steady &&
          duty_cmd > ADAPT_MIN_DUTY && duty_cmd < ADAPT_MAX_DUTY)
      {
        float T    = table_dp(duty_cmd);                       // what the factory curve predicts
        float pred = k_unit * T + b_unit;                      // what our corrected model predicts
        float resid = dp_pa - pred;                            // how wrong the model is, right now

        // Outlier rejection. Everything above says the plant has settled, and a slow enough
        // transient can satisfy all of it. This asks a different question: is the number
        // even plausible? No correction to a gain and an offset is 15 Pa at one operating
        // point. If the residual is that big, the sample is describing a state, not a model.
        if (fabsf(resid) <= ADAPT_RESID_MAX)                   // plausible: learn from it
        {

          // ---- persistency of excitation ----
          // Fitting a gain AND an offset needs the input to have actually moved. If every accepted
          // sample arrives at the same duty, then k*T + b = measured has infinitely many solutions
          // and the algorithm picks one at random, which is right at that one point and wrong
          // everywhere else. Measured on this rig: ten samples inside a four point duty window
          // pushed k to 0.83 and b to -5.2, which is 13.7 Pa wrong at full duty, and the estimator
          // got worse, not better. So: track how much the regressor is actually moving.
          T_slow   += 0.02f * (T - T_slow);
          T_spread += 0.02f * (fabsf(T - T_slow) - T_spread);
          uint8_t excited = (T_spread > ADAPT_PE_MIN);

          // Two parameters, and they have very different scales: T runs up to about 45 while the
          // offset's own input is always 1. A plain gradient step would move the gain a thousand
          // times faster than the offset, and the gain would end up absorbing an error that is
          // really an offset, until it hit its own limit. Scaling the regressor fixes that: both
          // parameters then learn at a comparable rate.
          float u    = T / ADAPT_SCALE;                          // the scaled input, around 0 to 2
          float den  = u * u + 1.0f;                             // both inputs together, normalised
          float step = ADAPT_ALPHA * resid / den;

          float ff_before = ff_duty_from_dp(sp);                 // what the model asks for before learning

          if (excited)
          {
            k_unit += step * u / ADAPT_SCALE;                    // chain rule back from the scaled input
            b_unit += step;                                      // the offset learns directly
          }
          else
          {
            b_unit += step;                                      // one operating point, one parameter
          }
          if (k_unit < K_MIN) k_unit = K_MIN;                    // bounds, so a bad patch of data
          if (k_unit > K_MAX) k_unit = K_MAX;                    // can never run the model away
          if (b_unit < -B_LIMIT) b_unit = -B_LIMIT;
          if (b_unit >  B_LIMIT) b_unit =  B_LIMIT;

          // Bumpless handover: the model just got better, so the feedforward term moved. Take that
          // same amount out of the integrator, and the total command does not jump. Over time the
          // steady correction migrates out of the integrator and into the model, which is the point:
          // next time we go blind, the model carries it and the integrator is not needed.
          integ -= (ff_duty_from_dp(sp) - ff_before);
        }
      }

      // ---- 5. setpoint schedule ----
      uint32_t now2 = xTaskGetTickCount();
      if (now2 - step_t0 >= pdMS_TO_TICKS(SP_HOLD_S * 1000))
      {
        step_t0 = now2;
        sp_high = !sp_high;                                    // alternate between the two targets
        sp = sp_high ? SP_HIGH : SP_LOW;
        step_idx++;
        if (step_idx >= SP_CYCLES * 2 || log_count >= LOG_N)
        {
          test_running = 0;
          mode = 0;
          manual_duty = -1;                                    // hand the fan back to the PID
        }
      }
    }

    if (test_running)
    {
      uint32_t now = xTaskGetTickCount();          // ticks are milliseconds here, TICK_RATE_HZ is 1000

      // Log every LOG_DECIM-th sample, while there is still room.
      decim++;
      if (decim >= LOG_DECIM && log_count < LOG_N)
      {
        log_sample_t *e = &log_buf[log_count];     // write straight into the array, no copying
        e->t_ms     = now - t0;                    // time since the run started
        e->sp_x10   = (int16_t)(sp * 10.0f);       // setpoint, zero in staircase mode
        e->dp_x10   = (int16_t)(dp_pa * 10.0f);    // the TRUE pressure, always recorded
        e->est_x10  = (int16_t)(est * 10.0f);      // what the controller believed
        e->rpm      = (uint16_t)fan_rpm;           // whatever FanTask last measured
        e->temp_x10 = (int16_t)(temperature_c * 10.0f);
        e->k_x1000  = (int16_t)(k_unit * 1000.0f); // the learned gain at this moment
        e->b_x10    = (int16_t)(b_unit * 10.0f);   // and the learned offset
        e->duty     = fan_duty_pct;                // what the fan was actually commanded
        e->valid    = (mode == 2) ? sensor_valid : 1;   // was the controller blind at this sample?
        log_count++;
        decim = 0;                                 // start counting towards the next logged sample
      }

      // Move to the next step when this one has been held long enough. Staircase only.
      if (mode == 1 && now - step_t0 >= pdMS_TO_TICKS(STEP_SECONDS * 1000))
      {
        step_idx++;
        step_t0 = now;
        if (step_idx >= STEP_COUNT || log_count >= LOG_N)
        {
          test_running = 0;                        // done
          mode         = 0;
          manual_duty  = -1;                       // hand the fan back to the PID
        }
        else
        {
          manual_duty = (int16_t)step_table[step_idx];   // next duty
        }
      }
    }
  }
}

// HAL calls this from the TIM2 interrupt every time the tach pin sees a rising edge.
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)                      // this callback is shared by all timers, so check which one
  {
    tach_pulses++;                                 // just count; keep interrupt code as short as possible
  }
}
/* USER CODE END Application */

