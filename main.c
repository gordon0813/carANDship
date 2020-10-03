
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);



/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int get_min_from_three(int a,int b,int c){
			if(a>b){
				if(a>c){
					return 0;
				}else{
					return 2;
				}
			}else{
				if(b>c){
					return 1;
				}else{
					return 2;
				}
			}
		}
	int return_slope(int* s,int length){
		//char buf[10] = { 0 };
		int a=get_min_from_three(s[length-1],s[length-2],s[length-3]);
		int b=get_min_from_three(s[0],s[1],s[2]);
		int c=s[b]-s[a];
//		buf[6] = '\n';
//		buf[5] = '\r';
//		buf[4] = c % 10 + '0';
//		buf[3] = c / 10 % 10 + '0';
//		buf[2] = c / 100 % 10 + '0';
//		buf[1] = c / 1000 % 10 + '0';
//		buf[0] = c / 10000 % 10 + '0';
//		HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 0xFFFF);

		return (s[a+length-3]-s[b])/(length-3-b+a);
	}
	float slope(int d1,int d2){
		return (d1-d2)/5000.0;
	}
	int d_h(){//l2
			int t2=0;
			int t1=0;
			int tmp=1;
			int tmp2=1;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
					HAL_Delay(10);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
					t1=0;
					t2=0;
					while (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)) {
						t1++;
						if (t1 > 20000) {
							break;
							t1 = 0;
						}
					}



					while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)) {
						t2++;
						if (t2 > 30000) {
							break;
						}
					}
					return t2;
		}
	int d_l2(){//l2
		int t2=0;
		int t1=0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
				HAL_Delay(10);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				t1=0;
				t2=0;
				while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)) {
					t1++;
					if (t1 > 20000) {
						break;
						t1 = 0;
					}
				}
				while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)) {
					t2++;
					if (t2 > 30000) {
						break;
					}
				}
				return t2;
	}
	int d_r2(){//r2
			int t2=0;
			int t1=0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
					HAL_Delay(10);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
					t1=0;
					t2=0;
					while (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)) {
						t1++;
						if (t1 > 20000) {
							break;
							t1 = 0;
						}
					}
					while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)) {
						t2++;
						if (t2 > 30000) {
							break;
						}
					}
					return t2;
		}
	int d_r1(){//r1
				int t2=0;
				int t1=0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
						HAL_Delay(10);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
						t1=0;
						t2=0;
						while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
							t1++;
							if (t1 > 20000) {
								break;
								t1 = 0;
							}
						}
						while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
							t2++;
							if (t2 > 30000) {
								break;
							}
						}
						return t2;
			}
	int d_l1(){//l1
				int t2=0;
				int t1=0;
				int tmp=0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
						HAL_Delay(10);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
						t1=0;
						t2=0;
						while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {
							t1++;
							if (t1 > 20000) {
								break;
								t1 = 0;
							}
						}
						while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {
							t2++;
							if (t2 > 30000) {
								break;
							}
						}

						return t2*4/5;
			}

	void rebuildright(int* slope2){
		int t2;
		int t2comp1 =distanceright();
		int t2comp2 = distanceright();
					if (t2comp1 < t2comp2) {
						t2 = t2comp1;
					}
					for (int i = 0; i < 10; i++) {
						slope2[i] = t2;
					}


	}
void printnumber(int num) {
	char buf[10] = { 0 };
	int t2stay=num;
	if (t2stay < 0) {
		t2stay = -t2stay;
		buf[7] = '\n';
		buf[6] = '\r';
		buf[5] = t2stay % 10 + '0';
		buf[4] = t2stay / 10 % 10 + '0';
		buf[3] = t2stay / 100 % 10 + '0';
		buf[2] = t2stay / 1000 % 10 + '0';
		buf[1] = t2stay / 10000 % 10 + '0';
		buf[0] = '-';
		HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 0xFFFF);
	} else {

		buf[6] = '\n';
		buf[5] = '\r';
		buf[4] = t2stay % 10 + '0';
		buf[3] = t2stay / 10 % 10 + '0';
		buf[2] = t2stay / 100 % 10 + '0';
		buf[1] = t2stay / 1000 % 10 + '0';
		buf[0] = t2stay / 10000 % 10 + '0';
		HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 0xFFFF);
	}
}
	/*void rightspeed(int k){
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = k;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //right
	}
	void leftspeed(int k){
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = k;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	}*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
          v v c  */ int main(void) {
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  /* USER CODE END 2 */
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;

	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	void setspeed(int l,int r){
		//l=0;r=0;
		if(l<0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			l=20+l;
		}else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		}
		sConfigOC.Pulse = l;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //right
		if(r<0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
			r=20+r;
		}else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		}
		sConfigOC.Pulse = r;
		HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);



	}
	setspeed(0,0);
	while (1) {
		if (!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
			break;
		}
		HAL_Delay(20);
		HAL_UART_Transmit(&huart2, (uint8_t*) "Hellow Wtest\r\n", 14, 0xFFFF);
	}
	void goback(){
		setspeed(0,0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);// left
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	}
	int filter(int d,int* rec){
		int i;
		for (i=0;i<4;i++){
			rec[i]=rec[i+1];
		}
		rec[4]=d;
		return rec[4];
	}
	int abs(int a){
		return (a>0)?a:-a;
	}
	void turnright(){
		while(1){


		int c=0;
		setspeed(-20,6);
		int dr1=d_r1();
		HAL_Delay(20);
		int dr2=d_r2();
		HAL_Delay(20);
		int dl1=d_l1();
		HAL_Delay(20);
		int dl2=d_l2();
		HAL_Delay(20);
		int dh=d_h();
		HAL_Delay(20);
		if(dh<12000){
			c+=1;
		}
		if(dr1>30000 ||dr2>30000){
			c+=1;
		}
		if(dl2<10000 |dl1<10000){
			c+=1;
		}
		if(c==3){
			return ;
		}else{
			c=0;
		}
		HAL_UART_Transmit(&huart2, (uint8_t*) "right\r\n", 7, 0xFFFF);
		}
	}

    int dr1;
    int dl1;
    int dr2;
	int dl2;
	int i;
	int dr1t;
    int dl1t;
    int dr2t;
	int dl2t;

	int dh;
	int dht;
	float sr;
	float sl;
	int ncase=0;

    int dhtmp1=0;
    int dhfilter[5];
    int backtmp=0;
    int add=0;
    int L1500c=0;
    int state=0;
    int hspeed=0;
    int nowspeedr=0;
    int nowspeedl=0;
    int count1;
    int count2;



	char buf[10] = { 0 };

	HAL_UART_Transmit(&huart2, (uint8_t*) "Hellow World\r\n", 14, 0xFFFF);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
   dr1=d_r1();
   dr2=d_r2();

   dl2=d_l2();
   dl1=d_l1();


   setspeed(10,10);
   count1=0;
   count2=0;
   ncase=2;
   for(i=0;i<6;i++){
	   dr1=d_r1();
	   dr2=d_r2();
	   dl2=d_l2();
	   dl1=d_l1();
	   dh=filter(d_h(),dhfilter);
   }
   ncase=-3;
   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	while (1) {

		   dr1=d_r1();
		   dr2=d_r2();

		   dl2=d_l2();
		   dl1=d_l1();

		   dh=d_h();



           /*if(dh>19000 && dr1-dr2>1300&&dr1<12000&&dr2<10000&&dr1-dr2<2400){
        	   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        	   ncase=1;
        	   setspeed(18,10);
        	   HAL_Delay(3000);
           }else*/

		   if(count1>30){
			   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		   }


           if(ncase==-3){
        	   setspeed(9,14);
        	   if((dr2>6000||dl2>6000)&&(dr1<4000||dl1<4000)){
        		   ncase=-1;
        		   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            	   setspeed(9,14);
            	   HAL_Delay(1200);
            	   setspeed(-15,-15);
            	   HAL_Delay(750);

        	   }
        	   HAL_Delay(100);
           }
           else if(ncase==-1){
        	   	  /*
			   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			   if(dr1+dl1>13000||dr2+dl2>13000){
				   setspeed(12,10);
			   }else if(dr1-dr2>1500 ||dl2-dl1>1500){
				   setspeed(10,0);
			   }else if(dr2-dr1>1500 ||dl1-dl2>1500){
				   setspeed(0,10);
			   }else if (dl1>dr1&&dl2>dr2){
				   setspeed(10,0);
			   }
			   else{
				   setspeed(10,10);
			   }*/

        	   if(dh<10000){
        		   setspeed(7,6);
        	   }else if(dr1-dr2>1000&&(dr1>200&&dr1<10000)&&(dr2>200&&dr2<10000)){
        		   setspeed(10,0);
        	   }else if(dl1-dl2>1000&&(dl1>200&&dl1<10000)&&(dl2>200&&dl2<10000)){
        		   setspeed(0,10);
        	   }else if(dl1<dr1){
        		   setspeed(10,6);
        	   }
        	   else{
        		   setspeed(13,10);
        	   }
			   if((dr2>7000||dr1>7000)&&(dr1>7000||dl1>7000)){
				   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
				   ncase=0;
				   setspeed(15,-18);
				   HAL_Delay(3000);
				   setspeed(10,10);
				   HAL_Delay(800);
				   count1=0;
			   }else if(dh<7000&&dr1>5000&&dr2>5000){
				   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
				   ncase=0;
				   setspeed(15,-18);
				   HAL_Delay(3000);
				   setspeed(10,10);
				   HAL_Delay(800);
				   count1=0;
			   }
			   HAL_Delay(100);

		} else if (ncase == 1) {
			if (dl1 - dl2 > 600 && (dl1 > 200 && dl1 < 10000)
					&& (dl2 > 200 && dl2 < 10000)) {
				setspeed(0, 10);
			} else if (dr1 - dr2 > 600 && (dr1 > 200 && dr1 < 10000)
					&& (dr2 > 200 && dr2 < 10000)) {
				setspeed(8, 0);
			} else {
				setspeed(8, 12);
			}
			HAL_Delay(100);
		} else if (dr1 > 16000 && dh < 20000&&(dr2<7500||dr2>19000)&&count1>65){
			   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
        	   ncase=2;
        	   setspeed(-14,-19);
        	   HAL_Delay(2500);
        	   setspeed(19,-19);
        	   HAL_Delay(3000);
        	   setspeed(10,13);
        	   HAL_Delay(3000);
        	   ncase=1;
           }else{
        	   if(dh>200&&dh<1500){
        		   setspeed(-17,0);
        		   HAL_Delay(300);
        		   count1+=3;
        		   backtmp=1;
        	   }

                else if((dh>200&&dh<3000) || dr2-dr1>2400){
        		   ncase=12;
        		   setspeed(-17,19);
        		   HAL_Delay(100);
        	   }else if(dr2-dr1>1000 &&dr1>200&&dr2>200){
        		   ncase=13;
        		   setspeed(-15,19);
        		   HAL_Delay(100);
        	   }else{
        		   ncase=14;
        		   setspeed(6,19);
        		   HAL_Delay(100);
        	   }
        	   count1+=1;
           }

		printnumber(ncase);
		printnumber(count1);
		printnumber(dh);
		printnumber(dr1);
		printnumber(dr2);
		printnumber(dl1);
		printnumber(dl2);


		//printnumber(dr1);

		//printnumber(dh);
		//printnumber(dl2);
		/*
        printnumber((int)((sr-sl)*1000));//(int)((sr-sl)*1000)
        printnumber((int)(sl*1000));
        printnumber((int)(sr*1000));
        */


	}

}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_10|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC2 PC5 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure GPIO pins : PC1 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 LD2_Pin PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|LD2_Pin|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB11 PB12
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	while (1) {
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
