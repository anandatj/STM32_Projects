/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 3-Phase PWM Motor Control - Main Program Body
  * @version        : 2.0.0
  * @date           : 2026-04-13
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief  PWM modulation mode selection.
 * @note   MISRA-C 2012 Rule 10.3 - Enumeration underlying type is uint8_t.
 */
typedef enum
{
    PWM_MODE_SINE              = 0U,  /**< Standard sinusoidal PWM                  */
    PWM_MODE_SVPWM_GEOMETRIC  = 1U,  /**< Geometric space-vector PWM (placeholder) */
    PWM_MODE_SVPWM_MINMAX     = 2U   /**< Min-Max (3rd harmonic) injection SVPWM   */
} PwmMode_e;

/**
 * @brief  Motor controller state machine states.
 */
typedef enum
{
    MOTOR_STATE_IDLE      = 0U,
    MOTOR_STATE_RUN       = 1U,
    MOTOR_STATE_FAULT     = 2U,
    MOTOR_STATE_EMERGENCY = 3U
} MotorState_e;

/**
 * @brief  Bitmask fault codes (combinable).
 */
typedef enum
{
    FAULT_NONE          = 0x0000U,
    FAULT_OVERCURRENT   = 0x0001U,
    FAULT_OVERTEMP      = 0x0002U,
    FAULT_OVERVOLTAGE   = 0x0004U,
    FAULT_UNDERVOLTAGE  = 0x0008U,
    FAULT_PWM_SYNC_LOST = 0x0010U
} FaultCode_e;

/* ---------------------------------------------------------------------------
 * Modular structs - all grouped for live-watch / debugger visibility
 * -------------------------------------------------------------------------*/

/**
 * @brief  Three-phase voltage container.
 * @note   Grouped so a debugger can expand a single node to see A/B/C.
 */
typedef struct
{
    float32_t a;   /**< Phase-A normalised voltage [0..1] after modulation */
    float32_t b;   /**< Phase-B normalised voltage [0..1] after modulation */
    float32_t c;   /**< Phase-C normalised voltage [0..1] after modulation */
} PhaseVoltage_t;

/**
 * @brief  Three-phase PWM compare-register values.
 */
typedef struct
{
    uint16_t a;    /**< Phase-A CCR value */
    uint16_t b;    /**< Phase-B CCR value */
    uint16_t c;    /**< Phase-C CCR value */
} PhasePwm_t;

/**
 * @brief  Sine-wave / angle generator parameters.
 */
typedef struct
{
    float32_t theta_rad;     /**< Current electrical angle  [rad]             */
    float32_t freq_hz;       /**< Electrical frequency      [Hz]              */
    float32_t omega_rad_s;   /**< Angular frequency  = 2*pi*freq  [rad/s]    */
    float32_t dt_s;          /**< Time-step per ISR call    [s]               */
} AngleGen_t;

/**
 * @brief  SVPWM-specific intermediate values.
 */
typedef struct
{
    float32_t minmax_offset;  /**< Min-Max injection offset                   */
} SvpwmData_t;

/**
 * @brief  Top-level motor-control data aggregate.
 * @note   Single struct instance visible in debugger as "g_motor".
 */
typedef struct
{
    AngleGen_t      angle;      /**< Angle / frequency generator              */
    PhaseVoltage_t  voltage;    /**< Phase voltages after modulation          */
    PhasePwm_t      pwm;        /**< Final PWM compare values                */
    SvpwmData_t     svpwm;      /**< SVPWM intermediate data                 */
    uint32_t        update_cnt; /**< Total ISR update counter (debug)         */
} MotorCtrl_t;

/**
 * @brief  Operational mode / state container.
 */
typedef struct
{
    MotorState_e  state;        /**< Current state-machine state              */
    PwmMode_e     pwm_mode;     /**< Active PWM modulation algorithm          */
    uint32_t      fault_code;   /**< Latched fault bitmask (FaultCode_e)      */
} OpMode_t;

/**
 * @brief  ISR timing diagnostics (DWT-based).
 */
typedef struct
{
    uint32_t last_cycles;       /**< Cycle count of last ISR execution        */
    uint32_t max_cycles;        /**< Worst-case cycle count since reset       */
} IsrDiag_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** @defgroup AppConfig  Application compile-time configuration
 *  @{ */
#define USE_SVPWM                       /**< Enable SVPWM modulation path     */

/* Mathematical constants  (MISRA Rule 1.2 - no reliance on <float.h>) */
#define MATH_PI             (3.14159265358979323846f)
#define MATH_TWO_PI         (2.0f * MATH_PI)
#define DEG_TO_RAD(deg)     ((deg) * (MATH_PI / 180.0f))

/* PWM / timer parameters */
#define PWM_FREQ_HZ         (20000.0f)  /**< PWM switching frequency [Hz]     */
#define PWM_DT_S            (1.0f / PWM_FREQ_HZ)  /**< PWM time-step [s]     */
#define TIM1_ARR_VALUE       (1250U)    /**< TIM1 auto-reload (period) value  */

/* Safety limits */
#define DUTY_MAX             (0.95f)
#define DUTY_MIN             (0.05f)
#define FREQ_MAX_HZ          (400.0f)
#define FREQ_MIN_HZ          (0.01f)

/** @} */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**
 * @brief  Clamp a float between lo and hi.
 * @note   MISRA-C 2012 Rule 20.7 - macro parameters enclosed in parentheses.
 */
#define CLAMPF(val, lo, hi)  (fminf(fmaxf((val), (lo)), (hi)))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/**
 * @brief  Motor control data - expand in debugger as "g_motor".
 */
static volatile MotorCtrl_t g_motor =
{
    .angle =
    {
        .theta_rad   = 0.0f,
        .freq_hz     = 0.2f,       /* 0.2 Hz default - visible & tuneable    */
        .omega_rad_s = 0.0f,
        .dt_s        = PWM_DT_S
    },
    .voltage = { .a = 0.0f, .b = 0.0f, .c = 0.0f },
    .pwm     = { .a = 0U,   .b = 0U,   .c = 0U   },
    .svpwm   = { .minmax_offset = 0.0f },
    .update_cnt = 0U
};

/**
 * @brief  Operation mode - expand in debugger as "g_opmode".
 */
static volatile OpMode_t g_opmode =
{
    .state      = MOTOR_STATE_IDLE,
    .pwm_mode   = PWM_MODE_SVPWM_MINMAX,
    .fault_code = (uint32_t)FAULT_NONE
};

/**
 * @brief  ISR timing diagnostics - visible as "g_isr_diag".
 */
static volatile IsrDiag_t g_isr_diag =
{
    .last_cycles = 0U,
    .max_cycles  = 0U
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void App_CalculatePhaseVoltages(void);
static void App_ApplyModulation(void);
static void App_NormaliseAndClamp(void);
static void App_ComputePwmCompare(void);
static void App_StartPwmOutputs(void);
static void App_StopPwmOutputs(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===========================================================================
 *  Motor Control - Angle & Voltage Calculation
 * =========================================================================*/

/**
 * @brief  Compute raw phase voltages from the current electrical angle.
 * @note   Uses cosf() (single-precision) to avoid implicit double promotion
 *         (MISRA Rule 10.4).
 */
static void App_CalculatePhaseVoltages(void)
{
    const float32_t theta = g_motor.angle.theta_rad;

    g_motor.voltage.a = cosf(theta);
    g_motor.voltage.b = cosf(theta - DEG_TO_RAD(120.0f));
    g_motor.voltage.c = cosf(theta + DEG_TO_RAD(120.0f));
}

/**
 * @brief  Apply the selected SVPWM modulation to phase voltages.
 * @note   Each case terminates with break (MISRA Rule 16.3).
 */
static void App_ApplyModulation(void)
{
#ifdef USE_SVPWM
    switch (g_opmode.pwm_mode)
    {
        case PWM_MODE_SVPWM_GEOMETRIC:
        {
            /* Placeholder - geometric SVPWM not yet implemented. */
            /* Falls through to sine (no modification) intentionally
             * kept as a separate break to satisfy MISRA Rule 16.3. */
            break;
        }

        case PWM_MODE_SVPWM_MINMAX:
        {
            /* Third-harmonic (min-max) injection */
            const float32_t va = g_motor.voltage.a;
            const float32_t vb = g_motor.voltage.b;
            const float32_t vc = g_motor.voltage.c;

            const float32_t v_max = fmaxf(va, fmaxf(vb, vc));
            const float32_t v_min = fminf(va, fminf(vb, vc));

            g_motor.svpwm.minmax_offset = 0.5f * (v_max + v_min);

            g_motor.voltage.a -= g_motor.svpwm.minmax_offset;
            g_motor.voltage.b -= g_motor.svpwm.minmax_offset;
            g_motor.voltage.c -= g_motor.svpwm.minmax_offset;
            break;
        }

        case PWM_MODE_SINE:
            /* Standard sine - no modification required. */
            break;

        default:
            /* Unknown mode - treat as sine (safe default). */
            break;
    }
#endif /* USE_SVPWM */
}

/**
 * @brief  Normalise voltages from [-1..1] to [0..1] and clamp to safe limits.
 */
static void App_NormaliseAndClamp(void)
{
    /* Map [-1, 1] -> [0, 1] */
    g_motor.voltage.a = (g_motor.voltage.a * 0.5f) + 0.5f;
    g_motor.voltage.b = (g_motor.voltage.b * 0.5f) + 0.5f;
    g_motor.voltage.c = (g_motor.voltage.c * 0.5f) + 0.5f;

    /* Clamp to safe duty-cycle bounds */
    g_motor.voltage.a = CLAMPF(g_motor.voltage.a, DUTY_MIN, DUTY_MAX);
    g_motor.voltage.b = CLAMPF(g_motor.voltage.b, DUTY_MIN, DUTY_MAX);
    g_motor.voltage.c = CLAMPF(g_motor.voltage.c, DUTY_MIN, DUTY_MAX);
}

/**
 * @brief  Convert normalised duty [0..1] to timer compare-register values.
 */
static void App_ComputePwmCompare(void)
{
    const float32_t period = (float32_t)TIM1_ARR_VALUE;

    g_motor.pwm.a = (uint16_t)(g_motor.voltage.a * period);
    g_motor.pwm.b = (uint16_t)(g_motor.voltage.b * period);
    g_motor.pwm.c = (uint16_t)(g_motor.voltage.c * period);
}

/**
 * @brief  Full voltage-calculation pipeline (called from main loop).
 * @pre    g_motor.angle members must be initialised.
 */
static void App_UpdateVoltages(void)
{
    /* Validate frequency before computing (MISRA defensive) */
    if ((g_motor.angle.freq_hz < FREQ_MIN_HZ) ||
        (g_motor.angle.freq_hz > FREQ_MAX_HZ))
    {
        /* Frequency out of range - latch fault, do not update PWM */
        g_opmode.fault_code |= (uint32_t)FAULT_PWM_SYNC_LOST;
        g_opmode.state       = MOTOR_STATE_FAULT;
        return;
    }

    /* Recompute omega in case freq_hz was tuned at runtime */
    g_motor.angle.omega_rad_s = MATH_TWO_PI * g_motor.angle.freq_hz;

    /* Wrap theta to [0, 2*pi) */
    g_motor.angle.theta_rad = fmodf(g_motor.angle.theta_rad, MATH_TWO_PI);

    App_CalculatePhaseVoltages();
    App_ApplyModulation();
    App_NormaliseAndClamp();
    App_ComputePwmCompare();
}

/* ===========================================================================
 *  PWM Output Helpers
 * =========================================================================*/

/**
 * @brief  Start all 3 complementary PWM channels + timer interrupts.
 */
static void App_StartPwmOutputs(void)
{
    (void)HAL_TIM_Base_Start_IT(&htim2);
    (void)HAL_TIM_Base_Start_IT(&htim1);

    (void)HAL_TIM_PWM_Start(&htim1,     TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1,  TIM_CHANNEL_1);

    (void)HAL_TIM_PWM_Start(&htim1,     TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1,  TIM_CHANNEL_2);

    (void)HAL_TIM_PWM_Start(&htim1,     TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1,  TIM_CHANNEL_3);

    g_opmode.state = MOTOR_STATE_RUN;
}

/**
 * @brief  Stop all 3 complementary PWM channels + timer interrupts.
 */
static void App_StopPwmOutputs(void)
{
    (void)HAL_TIM_PWM_Stop(&htim1,      TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1,   TIM_CHANNEL_1);

    (void)HAL_TIM_PWM_Stop(&htim1,      TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim1,   TIM_CHANNEL_2);

    (void)HAL_TIM_PWM_Stop(&htim1,      TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Stop(&htim1,   TIM_CHANNEL_3);

    (void)HAL_TIM_Base_Stop_IT(&htim1);
    (void)HAL_TIM_Base_Stop_IT(&htim2);
}

/* ===========================================================================
 *  ISR Callback
 * =========================================================================*/

/**
 * @brief  Timer period-elapsed ISR callback.
 * @param  htim  Pointer to the timer handle that generated the interrupt.
 * @note   MISRA Rule 8.13 - pointer-to-const not possible here (HAL API).
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        /* --- GPIO profiling pin HIGH --- */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);

        /* Advance electrical angle */
        g_motor.angle.theta_rad += g_motor.angle.omega_rad_s
                                 * g_motor.angle.dt_s;

        /* Write compare registers directly for minimum latency */
        TIM1->CCR1 = (uint32_t)g_motor.pwm.a;
        TIM1->CCR2 = (uint32_t)g_motor.pwm.b;
        TIM1->CCR3 = (uint32_t)g_motor.pwm.c;

        g_motor.update_cnt++;

        /* --- GPIO profiling pin LOW --- */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
    }
    else if (htim->Instance == TIM2)
    {
        /* TIM2 sync / trigger profiling */
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
    }
    else
    {
        /* MISRA Rule 15.7 - all if/else-if chains shall have a terminating else. */
    }
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

  App_StartPwmOutputs();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (g_opmode.state == MOTOR_STATE_RUN)
    {
        App_UpdateVoltages();
    }
    else if (g_opmode.state == MOTOR_STATE_FAULT)
    {
        /* In fault state - stop outputs, wait for external reset.
         * A debugger or UART command can clear g_opmode.fault_code
         * and set g_opmode.state = MOTOR_STATE_IDLE to recover.   */
        App_StopPwmOutputs();
    }
    else
    {
        /* IDLE or EMERGENCY - no PWM update */
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
  * @brief TIM1 Initialization Function
  * @param None
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
  htim1.Init.Period = TIM1_ARR_VALUE;
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
  * @brief TIM2 Initialization Function
  * @param None
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

  /*Configure GPIO pin : PE8 - ISR profiling output */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PG8 - TIM2 sync profiling */
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
  __disable_irq();
  while (1)
  {
      /* Intentional infinite loop on unrecoverable error.
       * MISRA Rule 2.1 - unreachable code acknowledged as design intent. */
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;  /* MISRA Rule 2.7 - unused parameter */
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
