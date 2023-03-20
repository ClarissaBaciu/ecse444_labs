/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define ARM_MATH_CM4
#include "arm_math.h"
#include <stdio.h>
//#include "stm32l4xx_hal_adc_ex.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

/* USER CODE BEGIN PV */
float temp;
float vref;
float vref_temp;
uint8_t sawtooth_data;
uint8_t triangle_data;
int8_t sin_data;

/*
 * The following arrays are for a frequency of 65Hz, and an amplitude of 1V.
 * To adjust the amplitude, just multiply each element by a desired amplitude, A.
 */
double sawtoothArray[260] = {0.0, 0.0038461538461538464, 0.007692307692307693, 0.011538461538461539, 0.015384615384615385, 0.019230769230769232, 0.02307692307692308, 0.026923076923076928, 0.030769230769230778, 0.034615384615384624, 0.03846153846153847, 0.042307692307692324, 0.04615384615384617, 0.05000000000000002, 0.05384615384615386, 0.057692307692307716, 0.06153846153846156, 0.0653846153846154, 0.06923076923076925, 0.0730769230769231, 0.07692307692307694, 0.0807692307692308, 0.08461538461538465, 0.0884615384615385, 0.09230769230769234, 0.09615384615384619, 0.10000000000000003, 0.10384615384615388, 0.10769230769230773, 0.11153846153846159, 0.11538461538461543, 0.11923076923076928, 0.12307692307692313, 0.12692307692307697, 0.1307692307692308, 0.13461538461538464, 0.13846153846153847, 0.1423076923076923, 0.14615384615384613, 0.14999999999999997, 0.1538461538461538, 0.15769230769230763, 0.1615384615384615, 0.16538461538461532, 0.16923076923076916, 0.173076923076923, 0.17692307692307682, 0.18076923076923065, 0.1846153846153845, 0.18846153846153832, 0.19230769230769215, 0.19615384615384598, 0.19999999999999982, 0.20384615384615365, 0.20769230769230748, 0.21153846153846131, 0.21538461538461515, 0.21923076923076898, 0.2230769230769228, 0.22692307692307664, 0.2307692307692305, 0.23461538461538434, 0.23846153846153817, 0.242307692307692, 0.24615384615384583, 0.24999999999999967, 0.2538461538461535, 0.25769230769230733, 0.26153846153846116, 0.265384615384615, 0.26923076923076883, 0.27307692307692266, 0.2769230769230765, 0.2807692307692303, 0.28461538461538416, 0.288461538461538, 0.2923076923076918, 0.29615384615384566, 0.2999999999999995, 0.3038461538461533, 0.30769230769230715, 0.311538461538461, 0.3153846153846148, 0.31923076923076865, 0.3230769230769225, 0.3269230769230763, 0.33076923076923015, 0.33461538461538404, 0.33846153846153787, 0.3423076923076917, 0.34615384615384553, 0.34999999999999937, 0.3538461538461532, 0.35769230769230703, 0.36153846153846086, 0.3653846153846147, 0.36923076923076853, 0.37307692307692236, 0.3769230769230762, 0.38076923076923, 0.38461538461538386, 0.3884615384615377, 0.3923076923076915, 0.39615384615384536, 0.3999999999999992, 0.403846153846153, 0.40769230769230685, 0.4115384615384607, 0.4153846153846145, 0.41923076923076835, 0.4230769230769222, 0.426923076923076, 0.43076923076922985, 0.4346153846153837, 0.4384615384615375, 0.44230769230769135, 0.4461538461538452, 0.449999999999999, 0.45384615384615284, 0.4576923076923067, 0.4615384615384605, 0.46538461538461434, 0.4692307692307682, 0.473076923076922, 0.4769230769230759, 0.4807692307692297, 0.48461538461538356, 0.4884615384615374, 0.4923076923076912, 0.49615384615384506, 0.4999999999999989, 0.5038461538461527, 0.5076923076923066, 0.5115384615384604, 0.5153846153846142, 0.519230769230768, 0.5230769230769219, 0.5269230769230757, 0.5307692307692295, 0.5346153846153834, 0.5384615384615372, 0.542307692307691, 0.5461538461538449, 0.5499999999999987, 0.5538461538461525, 0.5576923076923064, 0.5615384615384602, 0.565384615384614, 0.5692307692307679, 0.5730769230769217, 0.5769230769230755, 0.5807692307692294, 0.5846153846153832, 0.588461538461537, 0.5923076923076909, 0.5961538461538447, 0.5999999999999985, 0.6038461538461524, 0.6076923076923062, 0.61153846153846, 0.6153846153846139, 0.6192307692307677, 0.6230769230769215, 0.6269230769230754, 0.6307692307692292, 0.634615384615383, 0.6384615384615369, 0.6423076923076907, 0.6461538461538445, 0.6499999999999984, 0.6538461538461522, 0.657692307692306, 0.6615384615384599, 0.6653846153846137, 0.6692307692307675, 0.6730769230769214, 0.6769230769230752, 0.680769230769229, 0.684615384615383, 0.6884615384615368, 0.6923076923076906, 0.6961538461538445, 0.6999999999999983, 0.7038461538461521, 0.707692307692306, 0.7115384615384598, 0.7153846153846136, 0.7192307692307675, 0.7230769230769213, 0.7269230769230751, 0.730769230769229, 0.7346153846153828, 0.7384615384615366, 0.7423076923076904, 0.7461538461538443, 0.7499999999999981, 0.753846153846152, 0.7576923076923058, 0.7615384615384596, 0.7653846153846134, 0.7692307692307673, 0.7730769230769211, 0.7769230769230749, 0.7807692307692288, 0.7846153846153826, 0.7884615384615364, 0.7923076923076903, 0.7961538461538441, 0.7999999999999979, 0.8038461538461518, 0.8076923076923056, 0.8115384615384594, 0.8153846153846133, 0.8192307692307671, 0.8230769230769209, 0.8269230769230748, 0.8307692307692286, 0.8346153846153824, 0.8384615384615363, 0.8423076923076901, 0.8461538461538439, 0.8499999999999978, 0.8538461538461516, 0.8576923076923054, 0.8615384615384593, 0.8653846153846131, 0.8692307692307669, 0.8730769230769208, 0.8769230769230746, 0.8807692307692284, 0.8846153846153822, 0.8884615384615361, 0.8923076923076899, 0.8961538461538437, 0.8999999999999976, 0.9038461538461514, 0.9076923076923052, 0.9115384615384591, 0.9153846153846129, 0.9192307692307667, 0.9230769230769206, 0.9269230769230744, 0.9307692307692282, 0.9346153846153821, 0.9384615384615359, 0.9423076923076897, 0.9461538461538436, 0.9499999999999974, 0.9538461538461512, 0.9576923076923051, 0.9615384615384589, 0.9653846153846128, 0.9692307692307667, 0.9730769230769205, 0.9769230769230743, 0.9807692307692282, 0.984615384615382, 0.9884615384615358, 0.9923076923076897, 0.9961538461538435};

double triangleArray[260] = {0.0, 0.007692307692307693, 0.015384615384615385, 0.023076923076923078, 0.03076923076923077, 0.038461538461538464, 0.04615384615384616, 0.053846153846153856, 0.061538461538461556, 0.06923076923076925, 0.07692307692307694, 0.08461538461538465, 0.09230769230769234, 0.10000000000000003, 0.10769230769230773, 0.11538461538461543, 0.12307692307692313, 0.1307692307692308, 0.1384615384615385, 0.1461538461538462, 0.15384615384615388, 0.1615384615384616, 0.1692307692307693, 0.176923076923077, 0.18461538461538468, 0.19230769230769237, 0.20000000000000007, 0.20769230769230776, 0.21538461538461545, 0.22307692307692317, 0.23076923076923087, 0.23846153846153856, 0.24615384615384625, 0.25384615384615394, 0.2615384615384616, 0.2692307692307693, 0.27692307692307694, 0.2846153846153846, 0.29230769230769227, 0.29999999999999993, 0.3076923076923076, 0.31538461538461526, 0.323076923076923, 0.33076923076923065, 0.3384615384615383, 0.346153846153846, 0.35384615384615364, 0.3615384615384613, 0.369230769230769, 0.37692307692307664, 0.3846153846153843, 0.39230769230769197, 0.39999999999999963, 0.4076923076923073, 0.41538461538461496, 0.42307692307692263, 0.4307692307692303, 0.43846153846153796, 0.4461538461538456, 0.4538461538461533, 0.461538461538461, 0.4692307692307687, 0.47692307692307634, 0.484615384615384, 0.49230769230769167, 0.49999999999999933, 0.507692307692307, 0.5153846153846147, 0.5230769230769223, 0.53076923076923, 0.5384615384615377, 0.5461538461538453, 0.553846153846153, 0.5615384615384607, 0.5692307692307683, 0.576923076923076, 0.5846153846153836, 0.5923076923076913, 0.599999999999999, 0.6076923076923066, 0.6153846153846143, 0.623076923076922, 0.6307692307692296, 0.6384615384615373, 0.646153846153845, 0.6538461538461526, 0.6615384615384603, 0.6692307692307681, 0.6769230769230757, 0.6846153846153834, 0.6923076923076911, 0.6999999999999987, 0.7076923076923064, 0.7153846153846141, 0.7230769230769217, 0.7307692307692294, 0.7384615384615371, 0.7461538461538447, 0.7538461538461524, 0.76153846153846, 0.7692307692307677, 0.7769230769230754, 0.784615384615383, 0.7923076923076907, 0.7999999999999984, 0.807692307692306, 0.8153846153846137, 0.8230769230769214, 0.830769230769229, 0.8384615384615367, 0.8461538461538444, 0.853846153846152, 0.8615384615384597, 0.8692307692307674, 0.876923076923075, 0.8846153846153827, 0.8923076923076904, 0.899999999999998, 0.9076923076923057, 0.9153846153846134, 0.923076923076921, 0.9307692307692287, 0.9384615384615363, 0.946153846153844, 0.9538461538461518, 0.9615384615384595, 0.9692307692307671, 0.9769230769230748, 0.9846153846153824, 0.9923076923076901, 0.9999999999999978, 0.9923076923076946, 0.9846153846153869, 0.9769230769230792, 0.9692307692307716, 0.9615384615384639, 0.9538461538461562, 0.9461538461538486, 0.9384615384615409, 0.9307692307692332, 0.9230769230769256, 0.9153846153846179, 0.9076923076923102, 0.9000000000000026, 0.8923076923076949, 0.8846153846153872, 0.8769230769230796, 0.8692307692307719, 0.8615384615384643, 0.8538461538461566, 0.8461538461538489, 0.8384615384615413, 0.8307692307692336, 0.8230769230769259, 0.8153846153846183, 0.8076923076923106, 0.8000000000000029, 0.7923076923076953, 0.7846153846153876, 0.7769230769230799, 0.7692307692307723, 0.7615384615384646, 0.7538461538461569, 0.7461538461538493, 0.7384615384615416, 0.730769230769234, 0.7230769230769263, 0.7153846153846186, 0.707692307692311, 0.7000000000000033, 0.6923076923076956, 0.684615384615388, 0.6769230769230803, 0.6692307692307726, 0.661538461538465, 0.6538461538461573, 0.6461538461538496, 0.638461538461542, 0.6307692307692341, 0.6230769230769264, 0.6153846153846187, 0.6076923076923111, 0.6000000000000034, 0.5923076923076958, 0.5846153846153881, 0.5769230769230804, 0.5692307692307728, 0.5615384615384651, 0.5538461538461574, 0.5461538461538498, 0.5384615384615421, 0.5307692307692344, 0.5230769230769268, 0.5153846153846191, 0.5076923076923114, 0.5000000000000038, 0.4923076923076961, 0.48461538461538844, 0.4769230769230808, 0.4692307692307731, 0.46153846153846545, 0.4538461538461578, 0.4461538461538501, 0.43846153846154245, 0.4307692307692348, 0.4230769230769271, 0.41538461538461946, 0.4076923076923118, 0.40000000000000413, 0.39230769230769647, 0.3846153846153888, 0.37692307692308114, 0.36923076923077347, 0.3615384615384658, 0.35384615384615814, 0.3461538461538505, 0.3384615384615428, 0.33076923076923515, 0.3230769230769275, 0.3153846153846198, 0.30769230769231215, 0.3000000000000045, 0.2923076923076968, 0.28461538461538916, 0.2769230769230815, 0.2692307692307738, 0.26153846153846616, 0.2538461538461585, 0.24615384615385083, 0.23846153846154317, 0.2307692307692355, 0.22307692307692784, 0.21538461538462017, 0.2076923076923125, 0.20000000000000484, 0.19230769230769718, 0.1846153846153895, 0.17692307692308185, 0.16923076923077418, 0.16153846153846652, 0.15384615384615885, 0.14615384615385119, 0.13846153846154352, 0.13076923076923586, 0.12307692307692819, 0.11538461538462053, 0.10769230769231286, 0.1000000000000052, 0.09230769230769753, 0.08461538461538987, 0.0769230769230822, 0.06923076923077431, 0.06153846153846665, 0.053846153846158984, 0.04615384615385132, 0.038461538461543654, 0.03076923076923599, 0.023076923076928324, 0.015384615384620659, 0.007692307692312994};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */
void configure_channels(int i);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define VREFINT ((uint16_t*)(uint32_t)0x1FFF75AA)
#define TS_CAL1 ((uint16_t*)(uint32_t)0x1FFF75A8) //TS ADC raw data acquired at a temperature of 30 °C  (AROUND 3 V)
#define TS_CAL2 ((uint16_t*)(uint32_t)0x1FFF75CA) //TS ADC raw data acquired at a temperature of 130 °C

#define TS_CAL1_TEMP ((float)30.0) //temp used to measure TS_CAL1
#define TS_CAL2_TEMP ((float) 130.0) //temp used to measure TS_CAL2


//conditional compiling

//#define LED_OF
#define LED_TOG





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
  MX_ADC1_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

//  while(HAL_ADCEx_Calibration_Start(&hadc1,0) != HAL_OK);           // calibrate AD convertor


//  HAL_ADCEx_CalibrationStart(); //improve accuarcy of ADC conversion by calibrating driver

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // find vref
  configure_channels(0); 										//switch to voltage channel on ADC MUX
  HAL_ADC_Start(&hadc1); 								   //activate peripheral and start conversion
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  	  //wait for completion
  float raw_voltage = HAL_ADC_GetValue(&hadc1);		  //read sensor's digital value
  HAL_ADC_Stop(&hadc1);
  vref = 3.0f * (*VREFINT)/raw_voltage;

  // initialize variables
  int frequency = 65;
  int numSamples = 4*frequency;
  double timestep = 1/frequency/numSamples*1000; // get the timestep in ms
  int nb = 8; // number of bits in data

  // initialize DAC data
  sawtooth_data = 0;
  triangle_data = 0;
  double sawtooth;
  double triangle;
  double sin;
  sin_data = 0;

  int i = 0;
  float rad = 0;
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//HAL_DAC_setValue(sawtooth[i]);
	sawtooth = sawtoothArray[i];
	triangle = triangleArray[i];
	sawtooth_data = 3.4*sawtooth*pow(2,nb)/vref;
	triangle_data = 3.4*triangle*pow(2,nb)/vref;

	// To test sawtooth signal, simply change the data variable name in the
	// HAL_DAC_SetValue function!
	HAL_Delay(0);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, triangle_data);
	HAL_Delay(0);

	// 10 samples
    i += 26;
	if (i >= numSamples) {
		i = 0;
	}

	  /*
	// arm_sin_f32 test
	sin = 1 + arm_sin_f32(rad);
	sin_data = 0.5*sin*pow(2,nb)/vref;
	// write to DAC
	// - add a delay of 3ms in total to achieve a 65 Hz signal
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_Delay(0); // 1.5ms
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sin_data);
	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
	HAL_Delay(0); // 1.5ms

	rad += 0.01*M_PI;
	if (rad >= 2*M_PI) {
		rad = 0;
	}
*/


  }
  HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);



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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.DFSDMConfig = ADC_DFSDM_MODE_ENABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(myLed_GPIO_Port, myLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : myButton_Pin */
  GPIO_InitStruct.Pin = myButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(myButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : myLed_Pin */
  GPIO_InitStruct.Pin = myLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(myLed_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//configure channels
//0 for Vref and 1 for temp
void configure_channels(int i){

	ADC_ChannelConfTypeDef sConfig = {0};
	if (i == 0){
		sConfig.Channel = ADC_CHANNEL_VREFINT;
	}else{
		sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	}
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

}

//function for printing to console (swb port 0)
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
	  ITM_SendChar(*ptr++);
  }
  return len;
}

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
