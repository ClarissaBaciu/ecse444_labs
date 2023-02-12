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
#include "arm_math.h"
#define ARM_MATH_CM4
#include <stdio.h> // for printf function

//#define LENGTH 5



//definitions for conditional compiling


//=======================================//
// UNCOMMENT ONE TRACE TYPE TO ENABLE//

#define ASSEMBLY
//#define CCODE
//#define LIBRARY


//UNCOMMENT TO SELECT PROCESSING TYPE

#define C_PROCESSING
//#define LIB_PROCESSING

//=======================================//


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	int i = 0;
	for(i = 0; i<len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

// KalmanFilter function declaration
int KalmanFilterAssemly(float* InputArray, float* OutputArray,
		kalman_state* kstate, int Length) {
  // NOTE:
  // InputArray is a pointer, OutputArray too
  // TODO
  // 1) Assume space in memory has already been allocated for the
  //    inputArray, outputArray and state.
  // 2) All we need to do is modify the values in the array of the output array
  // By..
  //  3) creating a for loop to call kalman based on state and measurement
  //  4) inside the for loop, get the current x value of the kstate and store it
  //     in the outputArray.
  // REMEMBER: If kalman subroutine returns -1, update success_variable = -1,
  //           break out of for loop, and return success_variable. Otherwise,
  //		   the success_variable is pre-initialized to 0, and will return 0
  //           if no numerical errors occur during KalmanFiltering.
	int condition = 0;

	for (int i = 0; i < Length; i++) {
		float measurement = InputArray[i]; // post-increment pointer after de-referencing pointer
		// Call Kalman subroutine with current kstate and measurement
		condition = kalman(kstate, measurement); // returns 0 or -1
		if (condition == -1) { // check if numerical error occured
			break; // exit for loop
		}
		OutputArray[i] = kstate->x; // store x in output array
	}

	return condition;
}

int KalmanFilterC(float* InputArray, float* OutputArray,
		kalman_state* kstate, int Length) {
  // Same implementation as in KalmanFilterAssembly, except that all the instructions
  // written in Assembly by the Kalman subroutine will be written as arithmetic operations
  // in this C function.
	int condition = 0;

	for (int i = 0; i < Length; i++) {
		float measurement = InputArray[i]; // post-increment pointer after de-referencing pointer
		// Call Kalman subroutine with current kstate and measurement
		kstate->p = kstate->p + kstate->q;
		kstate->k = kstate->p / (kstate->p + kstate->r);
		kstate->x = kstate->x + kstate->k*(measurement - kstate->x);
		kstate->p = (1-kstate->k)*kstate->p;

		OutputArray[i] = kstate->x; // store x in output array

		// detect numerical errors with isnan (is NaN) function
		if (isnan(kstate->x)) {
			condition = -1;
			break;
		}
	}
	return condition;
}

//performing the kalman filter formuala using the CMSIS Library
int KalmanFilterLib(float* InputArray, float* OutputArray,
		kalman_state* kstate, int Length) {
	// This function is similar to KalmanFilterC, however, we use the built in functions provided by the
	// libarm_cortexM4lf_math.a library to use the CMSIS-CORE hardware.

	int condition = 0;
	// temporary variables
	float tmp1 = 0;
	float tmp2 = 0;

	for (int i = 0; i < Length; i++) {
			float measurement = InputArray[i]; // post-increment pointer after de-referencing pointer
			// (q, r, x, p, k)
			// kstate is base address of quintuple
			int adrs = &(kstate->p);
			arm_add_f32(&(kstate->p),&(kstate->q), &(kstate->p),1);
			arm_add_f32(&(kstate->p),&(kstate->r), &tmp1,1); // tmp1 = (p + r)
			tmp2 = 1/tmp1; // no CMSIS function available?
			// Potential solution but not working:
			// -> arm_mat_inverse_f32(&tmp1, &tmp2)) // take inverse of tmp2 = 1/tmp1
			arm_mult_f32(&(kstate->p),&tmp2,&(kstate->k),1); // k = p*tmp2
			arm_sub_f32(&measurement, &(kstate->x), &tmp1, 1); // tmp1 = measurement - x
			arm_mult_f32(&(kstate->k),&tmp1, &tmp2,1); // tmp2 = k*tmp1
			arm_add_f32(&(kstate->x),&tmp2, &(kstate->x),1); // x = x + k*tmp2
			arm_mult_f32(&(kstate->k),&(kstate->p), &tmp1,1); // tmp1 = k*p
			arm_sub_f32(&(kstate->p), &tmp1, &(kstate->p), 1); // p = p - tmp1

			OutputArray[i] = kstate->x; // add x-state to OutputArray

			// detect numerical errors with isnan (is NaN) function
			if (isnan(kstate->x)) {
				condition = -1;
				break;
			}
	}
	return condition;
}

//wrapper function to run all KalmanFilter functions
int KalmanFilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length){
	//conditional compiling to choose between methods
	int condition = 0;
	#ifdef ASSEMBLY
	condition = KalmanFilterAssemly(InputArray, OutputArray, kstate, Length);
	#endif
	#ifdef CCODE
	condition = KalmanFilterC(InputArray, OutputArray, kstate, Length);
	#endif
	#ifdef LIBRARY
	condition = KalmanFilterLib(InputArray, OutputArray, kstate, Length);
	#endif
	return condition;
}



//function to calculate standard deviation (using population)
float calculateStandardDeviation(float* array, int length, float average ){
	float sum = 0.0;
	for (int i = 0; i < length; i++) {
		float diff = array[i] - average;  //√(Σ(x_i - x̄)^2 / (n - 1))
		float pow = diff*diff;
		sum += pow;
	}
	float stand_dev = sqrt(sum/length);
	return stand_dev;
}


//helper function to calculate convolution
float calculateCorrelation(float* inputArray,float* outputArray,float average_input, float average_output,
		int length, float stand_dev_input,float stand_dev_output){

	float convolution_sum = 0.0;
	for (int i = 0; i < length; i++){
		int input_mult = (inputArray[i]- average_input)/stand_dev_input; //multiplicand for input
		int output_mult = (outputArray[i]- average_output)/stand_dev_output;	//multiplicand for output
		convolution_sum += input_mult*output_mult;
	}

	float value = convolution_sum/length;			//assign correlation
	return value;

}


//VERIFY USING LIBRARY FUNCTIONS
//helper function to calculate the convolution
float* calculateConvolution(float* inputArray,float* outputArray, int length, float* convolvedArray){
	for (int i = 0; i<length; i++){
		convolvedArray[i] = 0.0;
		for (int j = 0; j<length; j++){
			if(i-j >= 0 && i-j < length){
				convolvedArray[i] = inputArray[j] * outputArray[i-j];
			}
		}
	}
	return convolvedArray;
}

//function to process data  in C (results are passed as pointers)
int processC(float* inputArray, float* outputArray, int length, float* differenceArray, float* average,
		float* stand_dev, float* correlation, float* convolution){

	//difference array
	float sum_diff = 0.0;
	float sum_input = 0.0;
	float sum_output = 0.0;

	//loop1 : finds the difference and sums the values for subsequent mean calculation
	for (int i = 0; i < length; i++)
	{
		differenceArray[i] = inputArray[i] - outputArray[i]; 		//difference array
		if (isnan(differenceArray[i])) {
			return -1;
		}
		sum_diff += differenceArray[i];		//summing the values for mean
		sum_input += inputArray[i];
		sum_output += outputArray[i];

	}


	//average of differences
	*average = sum_diff/length;				//mean

	float average_input = sum_input/length;	 //find averages for input and output
	float average_output = sum_output/length;

	//standard deviation

	*stand_dev = calculateStandardDeviation(differenceArray,length, *average);						//standard deviation for difference array

	if (isnan(*stand_dev)) { 	//check standard deviation validity
		return -1;
	}

	//calculating convolution

	float stand_dev_input = calculateStandardDeviation(inputArray,length, average_input);
	float stand_dev_output = calculateStandardDeviation(outputArray, length, average_output);


	*correlation = calculateCorrelation(inputArray,outputArray,average_input,average_output,length,stand_dev_input,stand_dev_output); ///correlation

	if (isnan(*correlation)) { 	//check validity
		return -1;
	}

	//calculate convolution
	convolution = calculateConvolution(inputArray, outputArray, length, convolution);

	if (isnan(*convolution)) { 	//check validity
		return -1;
	}

	return 0;
}



// function to process data using CMSIS-DSP library
int processLib(float* inputArray, float* outputArray, int length, float* differenceArray, float* average,
		float* stand_dev, float* correlation, float* convolution){

	//difference array
	arm_sub_f32(inputArray, outputArray, differenceArray, length);

	//standard deviation
	arm_std_f32(differenceArray, length, stand_dev);

	//difference mean
	arm_mean_f32(differenceArray, length, average);
	//correlation
	arm_correlate_f32(inputArray, length, outputArray, length, correlation);
	//convolution
	arm_conv_f32(inputArray, length, outputArray, length, convolution);
	return 0;

}


//wrapper function for data processing
int dataProcess(float* inputArray, float* outputArray, int length, float* differenceArray, float* average_diff,
		float* stand_dev, float* correlation, float* convolution){
	//conditional compiling to select dataprocessing function
	int condition = 0;
	#ifdef C_PROCESSING
	condition = processC(inputArray, outputArray, length, differenceArray, average_diff, stand_dev, correlation, convolution);
	#endif

	#ifdef LIB_PROCESSING
	condition = processLib(inputArray, outputArray, length, differenceArray, average_diff, stand_dev, correlation, convolution);
	#endif

	return condition;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  float inputArray[100] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706,
	  					10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493,
	  					9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356,
	  					9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039,
	  					10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479,
	  					10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721,
	  					10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814,
	  					10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024,
	  					9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599,
	  					10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259,
	  					10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732,
	  					10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876,
	  					10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898,
	  					10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583,
	  					9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608,
	  					10.3465431058, 9.98446410493, 9.79376005657, 10.202518901,
	  					9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804,
	  					10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909,
	  					9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596,
	  					10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178,
	  					9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226,
	  					10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928,
	  					9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087,
	  					9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486,
	  					10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991,
	  					9.5799256668};

	  float outputArray[100] = {0};



//
//	  int length = 5;
//
//	  float inputArray[5] = {0};
//	  for (int i = 0; i < length; i++) {
//		  inputArray[i] = i;
//	  }
//
//	  float outputArray[5] = {0};
//
//
//	 //===================//
//	 // PART 2 TEST CASES //
//	 //===================//
//
//
//	  //division by 0
//
//	  kalman_state kstate2 = {.q = -0.1, .r = 0.1, .x = 5, .p = 0, .k = 0};
//	  int condition_zero = KalmanFilter(inputArray, outputArray, &kstate2, 5);
//
//	  if (condition_zero < 0){
//		  ERROR_ZERO = 1;
//	  	}
//
//	  // overflow
//
//	  kalman_state kstate3 = {.q = 3.40282e+38, .r = 3.40282e+38, .x = 5, .p = 0.1, .k = 0};
//	  int condition_over = KalmanFilter(inputArray, outputArray, &kstate3, 5);
//
//	  if (condition_over < 0){
//		  ERROR_OF = 1;
//
//	  }


	  int length = 100;
	  // regular
	  kalman_state kstate1 = {.q = 0.1, .r = 0.1, .x = 5.0, .p = 0.1, .k = 0};

	  ITM_Port32(31) = 1;

	  int condition_reg = KalmanFilter(inputArray, outputArray, &kstate1, length);
	  //put your code in here for monitoring execution time
	  ITM_Port32(31) = 2;


	  ITM_Port32(31) = 3;

	  //filter results

	  float differenceArray[100] = {0};
	  float average_diff = 0.0;
	  float stand_dev = 0.0;
	  float correlation = 0.0;
	  float convolution[100] = {0};

		//standard deviation


	  int x = dataProcess(inputArray, outputArray, length, differenceArray, &average_diff, &stand_dev, &correlation, convolution);

	  int breakpoint = 0;


	  if (x < 0){
//		  ERROR_PROCESS = 1;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

	int FAILURE = 1;
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
