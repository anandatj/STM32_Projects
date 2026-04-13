/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for 3-Phase Motor Control
  * @version        : 1.0.0
  * @date           : 2026-03-30
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
#include "main.h"
#include <math.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_SVPWM // SVPWM Toggle Comment to disable

/* Debug Configuration */
#define DEBUG_UART_ENABLED      1
#define DEBUG_GPIO_ENABLED      1
#define DEBUG_WATCHDOG_ENABLED  1
#define DEBUG_FAULT_DETECTION   1

/* Safety Limits */
#define MAX_DUTY_CYCLE          0.95f
#define MIN_DUTY_CYCLE          0.05f
#define MAX_FREQUENCY_HZ        100.0f
#define MIN_FREQUENCY_HZ        0.1f
#define OVERCURRENT_THRESHOLD   4095  // ADC value
#define OVERTEMP_THRESHOLD      80    // Celsius

/* Mathematical Constants */
#define PI                      3.14159265358979323846f
#define DEG2RAD(deg)            ((deg) * (PI / 180.0f))
#define TWO_PI                  (2.0f * PI)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Debug Macros */
#if DEBUG_UART_ENABLED
    #define DEBUG_PRINT(fmt, ...) printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
    #define ERROR_PRINT(fmt, ...) printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__)
    #define INFO_PRINT(fmt, ...)  printf("[INFO] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)
    #define ERROR_PRINT(fmt, ...)
    #define INFO_PRINT(fmt, ...)
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1; // Timer Handle for TIM1 (PWM Timer)
TIM_HandleTypeDef htim2; // Timer Handle for TIM2

/* USER CODE BEGIN PV */

/* Debug Variables */
volatile uint32_t debug_cycle_count = 0;
volatile uint32_t debug_max_isr_time = 0;
volatile uint32_t debug_last_isr_time = 0;
volatile uint8_t debug_fault_flag = 0;
volatile uint32_t debug_fault_code = 0;

/* Fault Tracking */
typedef enum {
    FAULT_NONE              = 0x0000,
    FAULT_OVERCURRENT       = 0x0001,
    FAULT_OVERTEMP          = 0x0002,
    FAULT_OVERVOLTAGE       = 0x0004,
    FAULT_UNDERVOLTAGE      = 0x0008,
    FAULT_WATCHDOG          = 0x0010,
    FAULT_PWM_SYNC_LOST     = 0x0020,
    FAULT_ENCODER_ERROR     = 0x0040,
    FAULT_COMMUNICATION     = 0x0080
} FaultCode_t;

/* Motor Control State Machine */
typedef enum {
    MOTOR_STATE_IDLE        = 0,
    MOTOR_STATE_INIT        = 1,
    MOTOR_STATE_RUN         = 2,
    MOTOR_STATE_FAULT       = 3,
    MOTOR_STATE_STOP        = 4,
    MOTOR_STATE_EMERGENCY   = 5
} MotorState_t;

/* PWM Mode Selection */
typedef enum {
    PWM_MODE_SINE           = 0,
    PWM_MODE_SVPWM_GEOMETRIC = 1,
    PWM_MODE_SVPWM_MINMAX   = 2
} PWM_Mode_t;

/* Frequency Parameters Structure */
typedef struct {
    float theta;            // Current angle in radians
    float freq;             // Frequency in Hz
    float dt;               // Time step (1/PWM frequency)
    float w;                // Angular frequency in rad/s
    
    float Va;               // Phase A voltage
    float Vb;               // Phase B voltage
    float Vc;               // Phase C voltage
    
    float MinMax;           // Min-Max injection value
    
    float PWM_A;            // Phase A PWM duty
    float PWM_B;            // Phase B PWM duty
    float PWM_C;            // Phase C PWM duty
    
    uint32_t update_count;  // Debug: update counter
} Freq_Param_t;

/* Operation Mode Structure */
typedef struct {
    PWM_Mode_t PWM_MODE;
    MotorState_t motor_state;
    uint8_t emergency_stop;
    uint8_t fault_reset;
} Op_Mode_t;

/* Global Instances */
Freq_Param_t Freq = {
    .dt = 1.0f / (20.0e3f),
    .freq = 0.2f,           // 0.2 Hz adjustable
    .theta = 0.0f,          // 0 Rad initial
    .w = 0.0f,
    .Va = 0.0f,
    .Vb = 0.0f,
    .Vc = 0.0f,
    .MinMax = 0.0f,
    .PWM_A = 0.0f,
    .PWM_B = 0.0f,
    .PWM_C = 0.0f,
    .update_count = 0
};

Op_Mode_t Operation_Mode = {
    .PWM_MODE = PWM_MODE_SVPWM_MINMAX, // Default: Min-Max Injection
    .motor_state = MOTOR_STATE_IDLE,
    .emergency_stop = 0,
    .fault_reset = 0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */

/* Debug Function Prototypes */
void Debug_Init(void);
void Debug_PrintStatus(void);
void Debug_PrintFaults(void);
void Debug_MeasureISRTime(uint32_t start_time);

/* Safety Function Prototypes */
uint8_t Safety_CheckOvercurrent(void);
uint8_t Safety_CheckOvertemp(void);
uint8_t Safety_CheckVoltage(void);
void Safety_HandleFault(FaultCode_t fault);
void Safety_EmergencyStop(void);

/* Motor Control Function Prototypes */
void Motor_Init(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_EmergencyStop(void);
void Motor_ResetFault(void);
void Calculate_Angle(void);
void Update_PWM(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Initialize debug infrastructure
  * @retval None
  */
void Debug_Init(void)
{
    #if DEBUG_UART_ENABLED
    /* UART already initialized by CubeMX */
    INFO_PRINT("=== 3-Phase Motor Control Debug System ===");
    INFO_PRINT("Build: %s %s", __DATE__, __TIME__);
    INFO_PRINT("PWM Frequency: 20 kHz");
    INFO_PRINT("Debug Mode: ENABLED");
    #endif
    
    #if DEBUG_GPIO_ENABLED
    /* GPIO already initialized by CubeMX */
    #endif
    
    #if DEBUG_WATCHDOG_ENABLED
    /* Initialize Independent Watchdog */
    /* IWDG configuration would go here */
    INFO_PRINT("Watchdog: ENABLED");
    #endif
}

/**
  * @brief  Print current system status
  * @retval None
  */
void Debug_PrintStatus(void)
{
    #if DEBUG_UART_ENABLED
    DEBUG_PRINT("--- System Status ---");
    DEBUG_PRINT("Motor State: %d", Operation_Mode.motor_state);
    DEBUG_PRINT("PWM Mode: %d", Operation_Mode.PWM_MODE);
    DEBUG_PRINT("Frequency: %.2f Hz", Freq.freq);
    DEBUG_PRINT("Angle: %.3f rad", Freq.theta);
    DEBUG_PRINT("PWM_A: %.3f, PWM_B: %.3f, PWM_C: %.3f", 
                Freq.PWM_A, Freq.PWM_B, Freq.PWM_C);
    DEBUG_PRINT("Update Count: %lu", Freq.update_count);
    DEBUG_PRINT("Max ISR Time: %lu us", debug_max_isr_time);
    DEBUG_PRINT("Fault Flag: 0x%02lX", debug_fault_code);
    #endif
}

/**
  * @brief  Print fault information
  * @retval None
  */
void Debug_PrintFaults(void)
{
    #if DEBUG_UART_ENABLED
    if (debug_fault_code == FAULT_NONE) {
        INFO_PRINT("No faults detected");
        return;
    }
    
    ERROR_PRINT("=== FAULT DETECTED ===");
    ERROR_PRINT("Fault Code: 0x%04lX", debug_fault_code);
    
    if (debug_fault_code & FAULT_OVERCURRENT)
        ERROR_PRINT("- OVERCURRENT FAULT");
    if (debug_fault_code & FAULT_OVERTEMP)
        ERROR_PRINT("- OVERTEMPERATURE FAULT");
    if (debug_fault_code & FAULT_OVERVOLTAGE)
        ERROR_PRINT("- OVERVOLTAGE FAULT");
    if (debug_fault_code & FAULT_UNDERVOLTAGE)
        ERROR_PRINT("- UNDERVOLTAGE FAULT");
    if (debug_fault_code & FAULT_WATCHDOG)
        ERROR_PRINT("- WATCHDOG FAULT");
    if (debug_fault_code & FAULT_PWM_SYNC_LOST)
        ERROR_PRINT("- PWM SYNC LOST");
    if (debug_fault_code & FAULT_ENCODER_ERROR)
        ERROR_PRINT("- ENCODER ERROR");
    if (debug_fault_code & FAULT_COMMUNICATION)
        ERROR_PRINT("- COMMUNICATION FAULT");
    #endif
}

/**
  * @brief  Measure ISR execution time
  * @param  start_time: DWT->CYCCNT at ISR entry
  * @retval None
  */
void Debug_MeasureISRTime(uint32_t start_time)
{
    #if DEBUG_GPIO_ENABLED
    uint32_t end_time = DWT->CYCCNT;
    uint32_t cycles = end_time - start_time;
    
    /* Convert cycles to microseconds (assuming 100 MHz clock) */
    uint32_t us = cycles / 100;
    
    if (us > debug_max_isr_time) {
        debug_max_isr_time = us;
    }
    debug_last_isr_time = us;
    #endif
}

/**
  * @brief  Check for overcurrent condition
  * @retval 1 if overcurrent detected, 0 otherwise
  */
uint8_t Safety_CheckOvercurrent(void)
{
    /* TODO: Implement ADC reading for current sensing */
    /* For now, return safe */
    return 0;
}

/**
  * @brief  Check for overtemperature condition
  * @retval 1 if overtemperature detected, 0 otherwise
  */
uint8_t Safety_CheckOvertemp(void)
{
    /* TODO: Implement temperature sensor reading */
    /* For now, return safe */
    return 0;
}

/**
  * @brief  Check for voltage anomalies
  * @retval 1 if voltage fault detected, 0 otherwise
  */
uint8_t Safety_CheckVoltage(void)
{
    /* TODO: Implement voltage monitoring */
    /* For now, return safe */
    return 0;
}

/**
  * @brief  Handle fault condition
  * @param  fault: Fault code to handle
  * @retval None
  */
void Safety_HandleFault(FaultCode_t fault)
{
    debug_fault_code |= fault;
    debug_fault_flag = 1;
    
    ERROR_PRINT("Fault detected: 0x%04X", fault);
    
    /* Transition to fault state */
    Operation_Mode.motor_state = MOTOR_STATE_FAULT;
    
    /* Stop PWM outputs */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    
    /* Print fault details */
    Debug_PrintFaults();
}

/**
  * @brief  Emergency stop procedure
  * @retval None
  */
void Safety_EmergencyStop(void)
{
    ERROR_PRINT("EMERGENCY STOP ACTIVATED!");
    
    /* Disable interrupts */
    __disable_irq();
    
    /* Immediately stop all PWM outputs */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    
    /* Set emergency state */
    Operation_Mode.motor_state = MOTOR_STATE_EMERGENCY;
    Operation_Mode.emergency_stop = 1;
    
    /* Toggle GPIO for visual indication */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
    
    /* Halt system */
    while(1) {
        /* System halted - requires power cycle */
    }
}

/**
  * @brief  Initialize motor control system
  * @retval None
  */
void Motor_Init(void)
{
    INFO_PRINT("Initializing motor control...");
    
    /* Reset parameters */
    Freq.theta = 0.0f;
    Freq.update_count = 0;
    debug_fault_code = FAULT_NONE;
    debug_fault_flag = 0;
    
    /* Set to idle state */
    Operation_Mode.motor_state = MOTOR_STATE_IDLE;
    Operation_Mode.emergency_stop = 0;
    Operation_Mode.fault_reset = 0;
    
    INFO_PRINT("Motor control initialized");
}

/**
  * @brief  Start motor operation
  * @retval None
  */
void Motor_Start(void)
{
    if (Operation_Mode.motor_state != MOTOR_STATE_IDLE) {
        WARNING_PRINT("Cannot start: Motor not in IDLE state");
        return;
    }
    
    INFO_PRINT("Starting motor...");
    
    /* Start timers */
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim1);
    
    /* Start PWM outputs */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    
    /* Transition to run state */
    Operation_Mode.motor_state = MOTOR_STATE_RUN;
    
    INFO_PRINT("Motor started");
}

/**
  * @brief  Stop motor operation
  * @retval None
  */
void Motor_Stop(void)
{
    INFO_PRINT("Stopping motor...");
    
    /* Stop timers */
    HAL_TIM_Base_Stop_IT(&htim1);
    HAL_TIM_Base_Stop_IT(&htim2);
    
    /* Stop PWM outputs */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    
    /* Transition to stop state */
    Operation_Mode.motor_state = MOTOR_STATE_STOP;
    
    INFO_PRINT("Motor stopped");
}

/**
  * @brief  Reset fault condition
  * @retval None
  */
void Motor_ResetFault(void)
{
    if (Operation_Mode.motor_state != MOTOR_STATE_FAULT) {
        WARNING_PRINT("No fault to reset");
        return;
    }
    
    INFO_PRINT("Resetting fault...");
    
    /* Clear fault flags */
    debug_fault_code = FAULT_NONE;
    debug_fault_flag = 0;
    Operation_Mode.fault_reset = 0;
    
    /* Transition to idle state */
    Operation_Mode.motor_state = MOTOR_STATE_IDLE;
    
    INFO_PRINT("Fault reset complete");
}

/**
  * @brief  Calculate phase angles and PWM values
  * @retval None
  */
void Calculate_Angle(void)
{
    /* Validate frequency range */
    if (Freq.freq < MIN_FREQUENCY_HZ || Freq.freq > MAX_FREQUENCY_HZ) {
        ERROR_PRINT("Frequency out of range: %.2f Hz", Freq.freq);
        Safety_HandleFault(FAULT_COMMUNICATION);
        return;
    }
    
    /* Calculate angular frequency */
    Freq.w = TWO_PI * Freq.freq;
    
    /* Wrap angle to [0, 2π] */
    Freq.theta = fmodf(Freq.theta, TWO_PI);
    
    /* Calculate phase voltages */
    Freq.Va = cosf(Freq.theta);
    Freq.Vb = cosf(Freq.theta - DEG2RAD(120.0f));
    Freq.Vc = cosf(Freq.theta + DEG2RAD(120.0f));
    
    #ifdef USE_SVPWM
    /* Apply SVPWM modulation */
    switch (Operation_Mode.PWM_MODE) {
        case PWM_MODE_SVPWM_GEOMETRIC:
            /* Geometric SVPWM - Not implemented yet */
            DEBUG_PRINT("Geometric SVPWM not implemented");
            break;
            
        case PWM_MODE_SVPWM_MINMAX:
            /* 3rd Harmonic Injection (MinMax Injection) */
            Freq.MinMax = 0.5f * (fmaxf(Freq.Va, fmaxf(Freq.Vb, Freq.Vc)) + 
                                  fminf(Freq.Va, fminf(Freq.Vb, Freq.Vc)));
            Freq.Va -= Freq.MinMax;
            Freq.Vb -= Freq.MinMax;
            Freq.Vc -= Freq.MinMax;
            break;
            
        case PWM_MODE_SINE:
        default:
            /* Standard sine PWM - no modification */
            break;
    }
    #endif
    
    /* Normalize to [0, 1] range */
    Freq.Va = Freq.Va * 0.5f + 0.5f;
    Freq.Vb = Freq.Vb * 0.5f + 0.5f;
    Freq.Vc = Freq.Vc * 0.5f + 0.5f;
    
    /* Clamp duty cycles to safe limits */
    Freq.Va = fmaxf(MIN_DUTY_CYCLE, fminf(MAX_DUTY_CYCLE, Freq.Va));
    Freq.Vb = fmaxf(MIN_DUTY_CYCLE, fminf(MAX_DUTY_CYCLE, Freq.Vb));
    Freq.Vc = fmaxf(MIN_DUTY_CYCLE, fminf(MAX_DUTY_CYCLE, Freq.Vc));
    
    /* Convert to PWM compare values */
    Freq.PWM_A = Freq.Va * htim1.Init.Period;
    Freq.PWM_B = Freq.Vb * htim1.Init.Period;
    Freq.PWM_C = Freq.Vc * htim1.Init.Period;
    
    /* Increment update counter */
    Freq.update_count++;
}

/**
  * @brief  Update PWM outputs
  * @retval None
  */
void Update_PWM(void)
{
    /* Update timer compare registers */
    TIM1->CCR1 = (uint16_t)Freq.PWM_A;
    TIM1->CCR2 = (uint16_t)Freq.PWM_B;
    TIM1->CCR3 = (uint16_t)Freq.PWM_C;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    
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
    MX_TIM1_Init();
    MX_TIM2_Init();
    
    /* USER CODE BEGIN 2 */
    
    /* Initialize debug system */
    Debug_Init();
    
    /* Initialize motor control */
    Motor_Init();
    
    /* Print initial status */
    Debug_PrintStatus();
    
    /* Start motor */
    Motor_Start();
    
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    
    uint32_t last_status_print = 0;
    const uint32_t STATUS_PRINT_INTERVAL = 1000; // Print status every 1 second
    
    while (1)
    {
        /* USER CODE END WHILE */
        
        /* USER CODE BEGIN 3 */
        
        /* Check for emergency stop */
        if (Operation_Mode.emergency_stop) {
            Safety_EmergencyStop();
        }
        
        /* Check for fault reset request */
        if (Operation_Mode.fault_reset) {
            Motor_ResetFault();
        }
        
        /* Perform safety checks */
        #if DEBUG_FAULT_DETECTION
        if (Safety_CheckOvercurrent()) {
            Safety_HandleFault(FAULT_OVERCURRENT);
        }
        if (Safety_CheckOvertemp()) {
            Safety_HandleFault(FAULT_OVERTEMP);
        }
        if (Safety_CheckVoltage()) {
            Safety_HandleFault(FAULT_OVERVOLTAGE);
        }
        #endif
        
        /* Calculate new PWM values */
        if (Operation_Mode.motor_state == MOTOR_STATE_RUN) {
            Calculate_Angle();
        }
        
        /* Print status periodically */
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_status_print) >= STATUS_PRINT_INTERVAL) {
            Debug_PrintStatus();
            last_status_print = current_time;
        }
        
        /* Feed watchdog */
        #if DEBUG_WATCHDOG_ENABLED
        /* IWDG->KR = 0xAAAA; */
        #endif
        
        /* Small delay to prevent CPU hogging */
        HAL_Delay(1);
    }
    /* USER CODE END 3 */
}

/**
  * @brief  Timer period elapsed callback
  * @param  htim: Timer handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Measure ISR execution time */
    #if DEBUG_GPIO_ENABLED
    uint32_t isr_start_time = DWT->CYCCNT;
    #endif
    
    /* TIM1 interrupt - PWM update */
    if (htim->Instance == TIM1)
    {
        /* Toggle GPIO for timing measurement */
        #if DEBUG_GPIO_ENABLED
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
        #endif
        
        /* Update angle */
        Freq.theta += Freq.w * Freq.dt;
        
        /* Update PWM outputs */
        Update_PWM();
        
        /* Toggle GPIO for timing measurement */
        #if DEBUG_GPIO_ENABLED
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
        #endif
    }
    
    /* TIM2 interrupt - Trigger/sync */
    if (htim->Instance == TIM2)
    {
        /* Toggle GPIO for timing measurement */
        #if DEBUG_GPIO_ENABLED
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
        #endif
    }
    
    /* Measure ISR execution time */
    #if DEBUG_GPIO_ENABLED
    Debug_MeasureISRTime(isr_start_time);
    #endif
}

/**
  * @brief  System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  TIM1 Initialization Function
  * @param  None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
    /* USER CODE BEGIN TIM1_Init 0 */
    
    /* USER CODE END TIM1_Init 0 */
    
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    
    /* USER CODE BEGIN TIM1_Init 1 */
    
    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    htim1.Init.Period = 1250;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 200;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 25;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */
    
    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief  TIM2 Initialization Function
  * @param  None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
    /* USER CODE BEGIN TIM2_Init 0 */
    
    /* USER CODE END TIM2_Init 0 */
    
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    
    /* USER CODE BEGIN TIM2_Init 1 */
    
    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 2499;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
    sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */
    
    /* USER CODE END TIM2_Init 2 */
}

/**
  * @brief  GPIO Initialization Function
  * @param  None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    
    /* USER CODE END MX_GPIO_Init_1 */
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
    
    /*Configure GPIO pin : PE8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    /*Configure GPIO pin : PG8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    
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
    ERROR_PRINT("Error_Handler called!");
    
    /* Disable interrupts */
    __disable_irq();
    
    /* Log error state */
    debug_fault_code |= FAULT_COMMUNICATION;
    
    /* Infinite loop */
    while (1)
    {
        /* Toggle LED to indicate error */
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
        for (volatile uint32_t i = 0; i < 1000000; i++);
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
    ERROR_PRINT("Assert failed: file %s on line %lu", file, line);
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
