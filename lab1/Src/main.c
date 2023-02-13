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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//=======================================//
// UNCOMMENT ONE TRACE TYPE TO ENABLE//

#define ASSEMBLY
#define CCODE
#define LIBRARY

//UNCOMMENT TO SELECT PROCESSING TYPE

#define C_PROCESSING
#define LIB_PROCESSING
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
float measurements[] = {
	50.33991394,  108.5995496 ,  101.16621108,  140.70847454,
	146.40066024,  132.8273938 ,  117.27498907,   75.2086868 ,
	118.82706757,   92.74245656,  101.95712986,  123.09030155,
	104.85462546,  116.28788509,   70.12425473,  102.38420298,
	126.97629399,  111.47346777,  125.21704925,   55.45409253,
	103.75122731,  131.09913697,   92.98028696,   53.97050003,
	111.64433253,   94.95783781,   74.1165049 ,  114.53478434,
	95.28763094,   56.62871342,   74.92242113,   93.72414977,
	81.89504741,   68.10727397,   78.76178757,  126.76286485,
	73.81774593,  114.14231436,   96.78224287,  146.01661775,
	90.85753369,   58.19254513,   81.8775203 ,   86.55621816,
	159.28477916,  128.82408917,   37.36850068,   79.60208335,
	95.58830723,   96.13448031,   95.95847415,   60.09172601,
	104.55115958,  175.1981823 ,   77.63277847,  102.68543955,
	80.88215887,   83.37748179,   96.76792743,   50.86298559,
	68.37143063,   68.14302619,   97.43570786,   83.25268855,
	121.4768682 ,   15.07841306,  120.99586221,   68.11175888,
	140.94606191,  120.27018556,  115.72114147,   75.23043975,
	108.0717018 ,   98.67420698,   98.76008278,   59.3090059 ,
	54.98238836,  139.16147473,   46.17309359,  167.08611345,
	120.19963849,  122.86357714,  128.30234219,  100.84901454,
	51.53079758,  116.74902876,   55.22273731,   94.4630115 ,
	165.63436337,  127.29334879,  166.91526609,   68.5635133 ,
	53.75012943,   75.66792119,  124.53808967,  103.19985117,
	105.09142146,  121.82069096,  109.28155397,   55.04592986,
	92.60351122,  128.88342355,  132.78646008,   90.14690438,
	115.22067932,   55.4789164 ,  127.76839555,   96.98558262,
	92.98261401,   97.76433603,   79.75818046,   88.29135342,
	73.10722998,  100.21579392,  104.45691583,  102.74864141,
	76.29617343,  140.29345766,  114.77753494,  126.65671407,
	67.7500999 ,   47.28451066,   78.42593527,  126.54264774,
	118.40660982,   42.66228816,   79.99471976,   98.37840175,
	102.13726935,  126.84734276,   38.74286353,  127.46366402,
	124.01187573,  107.14090014,   74.2162594 ,  121.55339385,
	138.59442189,   66.61150411,  101.76391333,  115.54385013,
	97.91381104,   76.57214692,  135.52201258,   71.40203349,
	87.28527641,  105.57274455,   91.78521805,   93.65259717,
	133.96881597,  133.8885431 ,   63.43838792,   69.652153  ,
	18.12145956,   80.14140995,  116.57563395,  124.08541401,
	149.43668308,  146.13096252,  114.20488238,  130.18517589,
	59.27667008,  137.54649273,  112.09297677,  107.16653953,
	171.50515972,   92.96758789,   96.68873333,  111.59242699,
	87.10693323,   91.62730553,   93.99632815,   73.64018755,
	102.77258667,  108.4959407 ,  106.36690063,   93.78581215,
	71.50530476,  110.30313339,  134.14330486,  136.17698345,
	139.53559802,   95.13016329,   93.35015957,   29.42981732,
	115.38383913,   69.12252614,  140.46278303,  115.49616401,
	69.43076551,   96.56442545,   64.32630936,   89.16478182,
	111.67165469,  126.34885278,  115.58262772,  109.15959316,
	116.79994294,   72.16197262,  118.65963755,   83.31693764,
	96.37758654,  121.48379355,  136.44363021,   53.37797885,
	58.8588467 ,   91.39762116,  134.54793758,  119.04379755,
	108.10014932,   47.63280174,  143.57113113,   74.65865345,
	94.54807515,  119.68705638,  111.14899977,  128.10356091,
	141.08526413,   98.26280144,  101.01531255,   82.11157604,
	46.28856728,  159.5162167 ,  134.05977476,  120.37284223,
	99.08277576,  142.19145422,   46.47572037,  104.6258349 ,
	112.63355821,   82.19437898,   98.87074664,   90.12952759,
	120.51844528,  111.13794462,   91.40909484,   92.4463013 ,
	142.16733576,  105.62217552,  135.76121796,  119.11323203,
	87.03531748,   87.65325948,  109.76938293,   78.09702766,
	72.4537781 ,  135.57998671,   85.16129301,   75.84522702,
	136.99938139,   93.46980317,   86.72052262,  113.06283024,
	124.63464627,  127.4158884 ,  103.41500169,  114.07859344,
	50.92864561,  112.92551419,   76.63756487,   61.95119806,
	102.68699292,  136.43615514,   43.25756596,  160.33833193,
	150.88798373,   72.35852031,   81.60235377,   40.49067494,
	92.319623  ,   94.30322061,   71.04355207,   77.63247182,
	92.71622627,   73.04653944,  153.03882177,  124.33379991,
	62.46995011,  136.23829226,  127.40907316,  151.17414145,
	77.64066548,  125.27507938,   96.41272909,  111.20837641,
	111.17993459,  107.86323312,  111.47227204,   96.58420056,
	59.73369005,  143.15097591,   89.80028714,  138.33888915,
	50.26662223,   54.42024947,   59.45413716,   88.26592863,
	112.91576922,  112.91157154,   76.42418688,  119.89975957,
	71.96430642,  107.70109829,   77.89711097,   90.88429512,
	106.71572811,   74.51853804,  194.79412395,   96.13498956,
	144.4920922 ,  104.8804323 ,  108.50114902,  139.02999656,
	43.70392459,  151.74691651,  120.49547221,   64.98656303,
	100.01737993,   63.95714497,  130.82140306,  136.98501964,
	90.22697863,   63.8818474 ,   78.59757331,   91.56656451,
	88.50071279,  147.92517749,  107.33568415,  139.45963998,
	172.66018587,   93.2212795 ,   91.89541414,  110.27053914,
	121.06251917,  126.0909583 ,  121.23095953,  154.40838995,
	139.65178749,  134.13960161,   80.89774231,  120.67232348,
	99.62662886,   89.26143131,  139.39611477,   69.03878295,
	80.93336988,  104.29978887,  110.77413293,   85.24256256,
	112.57218175,  116.33998064,  163.51210435,  131.78120938,
	81.55127788,  104.01239192,  121.92252152,   78.42493578,
	51.80051986,  140.00866052,   52.68993803,  132.60949323,
	86.1389788 ,  105.87863942,  109.92229837,   92.4240023 ,
	80.40004686,   70.07138598,  121.16482538,  134.84308621,
	118.12612093,   89.70313605,  133.93508627,  142.49974221,
	141.35486589,  126.38015369,  108.78783076,   76.68649149,
	118.88993339,  146.05087096,  149.68188567,  100.66470984,
	93.61082793,  120.25107567,   88.34088328,  119.250345  ,
	139.74587777,  116.31367518,   92.86027116,  109.48790495,
	73.86078286,   78.94231369,   78.43331354,  122.84567006,
	90.84816411,  129.07109406,  112.86471563,  142.94876081,
	112.63771385,  145.48778307,  121.43391372,  132.98279284,
	116.42606501,  112.24525451,  134.45073069,   74.3915669 ,
	69.20036276,  135.85129792,   87.6918277 ,  100.71859116,
	80.19507646,   94.66799672,   87.285061  ,  101.80310393,
	73.16876109,  147.99061943,  114.29680879,  103.03286671,
	126.72668508,  120.47387313,  136.32107649,   95.7891348 ,
	111.31459887,  114.8392038 ,   85.39325432,   92.4694419 ,
	69.77767097,  114.0158314 ,   50.32934829,  155.59498875,
	152.6727945 ,  127.06194389,   97.00701484,  142.26519781,
	116.39365392,   55.30163842,  104.19732615,  128.15936481,
	69.51860555,   86.84549541,   51.01418022,  100.3434208 ,
	136.92120112,   77.12867894,  111.52821092,  105.13283183,
	78.66282615,  103.30759979,  105.86073806,  120.17580133,
	113.04216978,   86.20270816,   80.15003338,  145.71193952,
	59.41497529,  138.7767136 ,   61.11503031,  129.02830154,
	100.6570287 ,  108.52094272,    9.91190043,  147.0999762 ,
	88.16841971,   55.65278411,  121.02306906,  127.26634713,
	99.49924621,   64.02899628,  119.00619219,   84.84936847,
	116.10317846,   86.23059182,   88.99310542,   36.17684344,
	91.35594641,  104.024059  ,  157.666907  ,   70.40908017,
	106.61143769,   66.21117839,   58.56743095,  101.97192396,
	127.52100956,   59.73699208,   71.52941119,  112.03865618,
	83.53648738,   65.50782791,   48.07370231,   84.52790843,
	82.81447597,   74.11153904,  117.785601  ,  120.883613  ,
	74.53676317,  133.12892466,   85.17145399,  101.53949102,
	31.87856809,   76.33800683,  120.31673498,  104.53545507,
	111.3396811 ,  111.163327  ,  116.7377102 ,   52.13263908,
	55.54505825,  116.11062769,  161.10431684,  106.09154033,
	89.70204096,   72.68909276,  167.8509128 ,  102.71375243,
	153.20415678,   51.78250816,  113.23686999,   99.16938029,
	135.36017662,  131.85114751,  158.72301781,   63.24011385,
	90.16912476,  102.34370798,  111.2643665 ,   94.7398689 ,
	116.04269973,   94.91017287,  122.72859751,  118.31198276,
	88.26004969,   64.36577789,   83.96050827,  129.26429749,
	152.89674814,  125.78902589,  113.08561336,  146.01617026,
	149.1557688 ,   89.29083448,   87.26070034,  150.22155392,
	116.48450863,  104.72326167,  126.26007869,  105.75559164,
	108.19879047,  142.79878853,   18.36070406,  111.38319707,
	104.72606487,   93.2174297 ,   97.61477733,   85.33835069,
	131.26174769,  100.77984221,  124.87689557,   83.00033805,
	90.11916799,  139.49731009,  104.72754411,  112.69950018,
	89.98644624,   90.98533894,  152.22060528,   73.58758914,
	138.74395001,  104.58615098,  140.02567835,  107.57992432,
	94.67297114,   49.68535431,  121.40336592,  115.42058054,
	94.24000241,  105.24530387,   79.9229339 ,   64.86251427,
	88.10817758,  106.34823013,  121.41153363,  102.69309448,
	156.0934353 ,   73.36123305,   88.73283304,  101.67473587,
	104.80169274,   63.3794546 ,  127.00605127,   69.33009804,
	71.50756997,  135.87457155,   88.05884224,  111.64574823,
	67.6022459 ,   80.79454064,  116.98129605,  127.20048323,
	118.25918849,  140.85278978,  128.05544839,  106.09875603
};

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

// calls kalman assembly subroutine
int KalmanFilterAssemly(float* InputArray, float* OutputArray,
		kalman_state* kstate, int Length) {
	int condition = 0;
	//loop through input measurement array
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

// Same implementation as in KalmanFilterAssembly, except that all the instructions
// written in Assembly by the Kalman subroutine will be written as arithmetic operations
// in this C function.
int KalmanFilterC(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	int condition = 0;
	for (int i = 0; i < Length; i++) {
		float measurement = InputArray[i]; // post-increment pointer after de-referencing pointer
		kstate->p = kstate->p + kstate->q;
		kstate->k = kstate->p / (kstate->p + kstate->r);
		kstate->x = kstate->x + kstate->k*(measurement - kstate->x);
		kstate->p = (1-kstate->k)*kstate->p;

		OutputArray[i] = kstate->x; // store x in output array
		if (isnan(kstate->x)) {// detect numerical errors with isnan (is NaN) function
			condition = -1;
			break;
		}
	}
	return condition;
}

// Helper function for processDataC
void calculateDifferenceArray(float* InputArray, float* OutputArray, float* differenceArray, int Length) {
	for(int i = 0; i < Length; i++) {
		// element-wise subtraction
		differenceArray[i] = InputArray[i] - OutputArray[i];
	}
	return; // void
}

// Helper function for calcuateStandardDeviation
float calculateAverage(float* Array, int Length) {
	float sum = 0;
	for(int i = 0; i < Length; i++) {
		sum += Array[i];
	}
	return sum/Length;
}

// Helper function for processDataC
//calculate standard deviation of an array
float calculateStandardDeviation(float* Array, int Length)
{
	float average = calculateAverage(Array, Length);

	float sum = 0;
	for(int i = 0; i < Length; i++) {
		sum += pow(Array[i] - average, 2); // sum of standard dev
	}

	return sqrt(sum/Length); // standard deviation
}

// Helper function for processDataC
//calculates cross correlation of two vectors
void calculateCorrelation(float* InputArray, float* OutputArray, float* correlationArray, int Length) {


	float reversedOutputArray[Length]; //create reversed array
	for (int i = 0; i<Length; i++){
		reversedOutputArray[i] = OutputArray[Length - 1 - i];
	}
//	for (int i = 0; i<Length*2-1; i++){
//	  		correlationArray[i] = 0.0;
//	  		for (int j = 0; j<Length; j++){
//	  			if(i-j >= 0 && i-j < Length){
//	  				correlationArray[i] = InputArray[j] * reversedOutputArray[i-j];
//	  			}
//	  		}
//	  	}
//	return;


	for (int i = 0; i<Length*2-1; i++){
		correlationArray[i] = 0.0;
		for (int j = 0; j<Length; j++){
			if(i-j >= 0 && i-j < Length){
				correlationArray[i] += InputArray[j] * reversedOutputArray[i-j];
			}
		}
	}
	return;

}



// Helper function for processDataC
//calculate convolution of two vectors
void convolve(float* InputArray, float* OutputArray, float* convolutionArray, int Length)
{
  for (int i = 0; i<Length*2-1; i++){
  		convolutionArray[i] = 0.0;
  		for (int j = 0; j<Length; j++){
  			if(i-j >= 0 && i-j < Length){
  				convolutionArray[i] += InputArray[j] * OutputArray[i-j];
  			}
  		}
  	}
  return;
}

// Function that processes the filtered data using the CMSIS-DSP library
void processDataLib(float* InputArray, float* OutputArray, int Length, float* differenceArray, float* convolutionArray, float* correlationArray, properties* stats) {
	arm_sub_f32(InputArray, OutputArray, differenceArray, Length); // calculate differenceArray
	arm_mean_f32 (differenceArray, Length, &(stats->average)); // calculate the average
	arm_std_f32 (differenceArray, Length, &(stats->standardDev)); // calculate standardDev
	arm_conv_f32 (InputArray, Length, OutputArray, Length, convolutionArray); // calculate the convolution
	arm_correlate_f32 (InputArray, Length, OutputArray, Length, correlationArray); // calculate correlation
	return; // exit function
}

// Function that processes filtered data using C arithmetic
void processDataC(float* InputArray, float* OutputArray, int Length, float* differenceArray, float* convolutionArray, float* correlationArray, properties* stats) {

	// calculate differenceArray
	calculateDifferenceArray(InputArray, OutputArray, differenceArray, Length);
	// calculate S.D. from differenceArray
	float average = calculateAverage(differenceArray, Length);
	float standardDev = calculateStandardDeviation(differenceArray, Length);

	// calculate correlation
	calculateCorrelation(InputArray, OutputArray, correlationArray, Length);

	// calculate convolution
	convolve(InputArray, OutputArray, convolutionArray, Length);

	stats->average = average;
	stats->standardDev = standardDev;
}

// This function is similar to KalmanFilterC, however, we use the built in functions provided by the
// libarm_cortexM4lf_math.a library to use the CMSIS-CORE hardware.
int KalmanFilterLib(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	int condition = 0;
	float tmp1 = 0;	// temporary variables
	float tmp2 = 0;
	for (int i = 0; i < Length; i++) {
			float measurement = InputArray[i]; // post-increment pointer after de-referencing pointer
			// (q, r, x, p, k)
			// kstate is base address of quintuple
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

	  // declare structs to be used
	  properties statsC;
	  properties statsLib;

	  // initialize variables
	  int Length = 100;
	  float OutputArrayA[100] = {0};
	  float OutputArrayC[100] = {0};
	  float OutputArrayLib[100] = {0};

	  // create output array and kstate used by assembly filter
	  kalman_state kstateA = {.q = 0.1, .r = 0.1, .x = 5.0, .p = 0.1, .k = 0};

	  // create output array and kstate used by C filter
	  kalman_state kstateC = {.q = 0.1, .r = 0.1, .x = 5.0, .p = 0.1, .k = 0};

	  // create output array and kstate used by CMSIS-DSP filter
	  kalman_state kstateLib = {.q = 0.1, .r = 0.1, .x = 5.0, .p = 0.1, .k = 0};

	  // call subsequent filtering functions

	  #ifdef ASSEMBLY
	  ITM_Port32(31) = 1; //timing SWB
	  KalmanFilterAssemly(measurements, OutputArrayA, &kstateA, Length); //calling assembly kalman filter
	  ITM_Port32(31) = 2;
	  #endif
	  #ifdef CCODE
	  ITM_Port32(31) = 3;
	  KalmanFilterC(measurements, OutputArrayC, &kstateC, Length); //calling C filter function
	  ITM_Port32(31) = 4;
	  #endif
	  #ifdef LIBRARY
	  ITM_Port32(31) = 5;
	  KalmanFilterLib(measurements, OutputArrayLib, &kstateLib, Length); //calling library function
	  ITM_Port32(31) = 6;
	  #endif


	  // process the data accordingly (only for C and CMSIS-DSP library functions)
	  // initialize other variables
	  float differenceArrayC[100] = {0};
	  float differenceArrayLib[100] = {0};

	  float correlationArrayC[199] = {0};
	  float correlationArrayLib[199] = {0};

	  float convolutionArrayC[199] = {0};
	  float convolutionArrayLib[199] = {0};


	  #ifdef C_PROCESSING
	  ITM_Port32(31) = 7;
	  processDataC(measurements, OutputArrayC, Length, differenceArrayC, convolutionArrayC, correlationArrayC, &statsC);
	  ITM_Port32(31) = 8;
	  #endif
	  #ifdef LIB_PROCESSING
	  ITM_Port32(31) = 9;
	  processDataLib(measurements, OutputArrayLib, Length, differenceArrayLib, convolutionArrayLib, correlationArrayLib, &statsLib);
	  ITM_Port32(31) = 10;
	  #endif

	  break;


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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
