/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef tim2;
ADC_HandleTypeDef adc3;
DAC_HandleTypeDef dac;
UART_HandleTypeDef uart2;
DMA_HandleTypeDef dma_adc3;

struct netif gnetif;
static void Netif_Config(void);

GPIO_TypeDef *INPUT_PORT[16] = {GPIOG,GPIOG,GPIOG,GPIOG,GPIOG,GPIOG,GPIOD,GPIOD,GPIOD,GPIOD,GPIOD,GPIOD,GPIOD,GPIOD,GPIOC,GPIOC};
uint16_t INPUT_Pin[16] ={GPIO_PIN_14,GPIO_PIN_13,GPIO_PIN_12,GPIO_PIN_11,GPIO_PIN_10,GPIO_PIN_9,GPIO_PIN_7,GPIO_PIN_6,GPIO_PIN_5,GPIO_PIN_4,GPIO_PIN_3,GPIO_PIN_2,GPIO_PIN_1,GPIO_PIN_0,GPIO_PIN_12,GPIO_PIN_11};

GPIO_TypeDef *OUTPUT_PORT[16] = {GPIOB,GPIOB,GPIOE,GPIOE,GPIOE,GPIOE,GPIOE,GPIOE,GPIOE,GPIOE,GPIOE,GPIOG,GPIOG,GPIOF,GPIOF,GPIOF};
uint16_t OUTPUT_Pin[16] ={GPIO_PIN_11,GPIO_PIN_10,GPIO_PIN_15,GPIO_PIN_14,GPIO_PIN_13,GPIO_PIN_12,GPIO_PIN_11,GPIO_PIN_10,GPIO_PIN_9,GPIO_PIN_8,GPIO_PIN_7,GPIO_PIN_1,GPIO_PIN_0,GPIO_PIN_15,GPIO_PIN_14,GPIO_PIN_13};


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void Dma_init(void);
void Timer_Config(void);
static void Adc_init(void);
static void MX_USART2_UART_Init(void);
void Send_string(char* s);
void Set_LED(unsigned char ,unsigned char );
void Update_coils(void);
void Update_discrete_inputs(void);
void Update_registers(void);
void Update_inputs_registers(void);
static void Dac_init(void);

/* Global variables definitions */
uint8_t *Outputs;
uint8_t *Inputs;
uint16_t *Ainputs;
uint16_t *Registers;
uint8_t *Errors;
uint8_t Modbus_changed;
uint8_t Timer_irq;
uint8_t Counter;
uint16_t Adc_buffor[MAX_input_registers*4];
uint8_t Inputs_counter[MAX_discrate_inputs];
uint8_t Ainputs_counter[MAX_input_registers];

int main(void)
{
		
		Inputs = (uint8_t*) malloc((MAX_discrate_inputs/8) * sizeof(*Inputs));
		Outputs = (uint8_t*) malloc((MAX_coils/8) * sizeof(*Outputs));
		Ainputs = (uint16_t*) malloc(MAX_input_registers * sizeof(*Ainputs));
		Registers = (uint16_t*) malloc(MAX_registers * sizeof(*Registers));
		Errors = (uint8_t*) malloc(1 * sizeof(*Errors));
		for(int i = 0; i < MAX_registers; i++)Registers[i]= 0;
		for(int i = 0; i < MAX_input_registers; i++)Ainputs[i] = 0;
		for(int i = 0; i < (MAX_coils/8); i++)Outputs[i] = 0;
		for(int i = 0; i < (MAX_discrate_inputs/8); i++)Inputs[i] = 0;
		
		Errors[0] = 0;
		Counter = 0;
		
		HAL_Init();
		SystemClock_Config();
		
		MX_GPIO_Init();
		MX_SPI2_Init();
		MX_USART2_UART_Init();
		
		Set_LED(ALL_LED,LED_OFF);

  /* Initilaize the LwIP stack */
    lwip_init();

    /* Configure the Network interface */
    Netif_Config();

    /* tcp echo server Init */
    tcp_echoserver_init();

    /* Notify user about the network interface config */
    User_notification(&gnetif);

		Timer_Config();	
		Dma_init();
		Adc_init();
		Set_LED(LED_GREEN,LED_ON);
		while (1)
		{
			/* Read a received packet from the Ethernet buffers and send it  to the lwIP for handling */
			ethernetif_input(&gnetif);

			/* Triggers actual ethernet transmission if transmission buffers are not empty */
			ethernet_transmit();

			/* Handle timeouts */
			sys_check_timeouts();
			if(Modbus_changed)
			{
				switch(Modbus_changed)
				{
					case MODBUS_CHANGED_WRITE_COILS:
						Update_coils();
						break;
					case MODBUS_CHANGED_WRITE_REGISTERS:
						Update_registers();
						break;
				}
				Modbus_changed = 0;
			}
			if(Timer_irq)
			{
				Counter += 1;
				if(Counter > 99) Counter = 0;
				Timer_irq=0;
				Update_discrete_inputs();
				Update_inputs_registers();
			}
			
			if (__HAL_UART_GET_FLAG(&uart2, UART_FLAG_RXNE) == SET)
			{
			 uint8_t value;
			 HAL_UART_Receive(&uart2, &value, 1, 100);
			 Registers[6] = value;
			}
		}
}



void Timer_Config(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();

	
	tim2.Instance = TIM2;
  tim2.Init.Period = 10000 - 1;
  tim2.Init.Prescaler = 800 - 1;
  tim2.Init.ClockDivision = 0;
  tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim2.Init.RepetitionCounter = 0;
  tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&tim2);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_TIM_Base_Start_IT(&tim2);
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  Timer_irq = 1;
}

void SystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Initializes the CPU, AHB and APB busses clocks
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Initializes the CPU, AHB and APB busses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
}

static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}



static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED4_Pin|LED5_Pin|ENC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin LED5_Pin ENC_CS_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LED5_Pin|ENC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_INT_Pin */
  GPIO_InitStruct.Pin = ENC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_INT_GPIO_Port, &GPIO_InitStruct);

  /* Configurate GPIOB*/
  GPIO_InitStruct.Pin = output0_Pin|output1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configurate GPIOE*/
  GPIO_InitStruct.Pin = output10_Pin | output9_Pin | output8_Pin | output7_Pin | output6_Pin | output5_Pin | output4_Pin | output3_Pin | output2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* Configurate GPIOG*/
  GPIO_InitStruct.Pin = output11_Pin | output12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* Configurate GPIOF*/
  GPIO_InitStruct.Pin = output15_Pin | output14_Pin | output13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/* Configurate ADC3 Pins */
	GPIO_InitStruct.Pin = Ainput0_Pin | Ainput1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ainput0_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = Ainput2_Pin | Ainput3_Pin |Ainput4_Pin | Ainput5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ainput5_GPIO_Port, &GPIO_InitStruct);

	/* Configurate Usart2 Pins */
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		 
	GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void Adc_init(void)
{
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC3EN; // enable clock on ADC3
	RCC->CFGR|=RCC_CFGR_ADCPRE_DIV4;// ADC max clock. 14MHz
	ADC3->SQR1 |= ADC_SQR1_L_0 | ADC_SQR1_L_2;//6 conversions	
	ADC3->SQR3 = (4<<ADC_SQR3_SQ1_Pos)|(5<<ADC_SQR3_SQ2_Pos)|(10<<ADC_SQR3_SQ3_Pos)|(11<<ADC_SQR3_SQ4_Pos)|(12<<ADC_SQR3_SQ5_Pos)|(13<<ADC_SQR3_SQ6_Pos); //4,5,10,11,12,13 added channel 
	ADC3->SMPR2 |= ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2; //Sample time for channel 4
	ADC3->SMPR2 |= ADC_SMPR2_SMP5_1 | ADC_SMPR2_SMP5_2; //Sample time for channel 5
	ADC3->SMPR1 |= ADC_SMPR1_SMP10_1 | ADC_SMPR1_SMP10_2; //Sample time for channel 10
	ADC3->SMPR1 |= ADC_SMPR1_SMP11_1 | ADC_SMPR1_SMP11_2; //Sample time for channel 11
	ADC3->SMPR1 |= ADC_SMPR1_SMP12_1 | ADC_SMPR1_SMP12_2; //Sample time for channel 12
	ADC3->SMPR1 |= ADC_SMPR1_SMP13_1 | ADC_SMPR1_SMP13_2; //Sample time for channel 13
	ADC3->CR1 |= ADC_CR1_SCAN;    // enable scan mode ( continious measure )
	ADC3->CR2 |= (7<<ADC_CR2_EXTSEL_Pos)| ADC_CR2_EXTTRIG | ADC_CR2_ADON | ADC_CR2_CONT| ADC_CR2_DMA; //Configurate external trigger (EXTSEL/EXTTRIG), enable measure (ADON), continuous mode (CONT) and using DMA(DMA)
	ADC3->CR2 |= ADC_CR2_SWSTART; //Start measure

}

static void Dma_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA2EN;//enable clock on DMA2 
	DMA2_Channel5->CNDTR= 24; // how many measures ( 6 channel * 4 value in buffor to estimate real value)
	DMA2_Channel5->CPAR = 0x40013C4C; //ADC3->DR addres;
	DMA2_Channel5->CMAR = (uint32_t)&(Adc_buffor[0]); // buffor addres 
	DMA2_Channel5->CCR |= DMA_CCR_PL_0;  // channel priority 
	DMA2_Channel5->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC | DMA_CCR_MINC; // size of adc->dr and buffor, Circular mode ( CIRC) and incrementing memory addres(MINC) 
	DMA2_Channel5->CCR |= DMA_CCR_EN; //stard DMA
}

static void Dac_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	DAC->CR |= (7<<DAC_CR_TSEL2_Pos) | (7<<DAC_CR_TSEL1_Pos) | DAC_CR_EN2 | DAC_CR_EN1;
	
	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2;
	
}


static void MX_USART2_UART_Init(void)
{
	uart2.Instance = USART2;
	uart2.Init.BaudRate = 115200;
	uart2.Init.WordLength = UART_WORDLENGTH_8B;
	uart2.Init.StopBits = UART_STOPBITS_1;
	uart2.Init.Parity = UART_PARITY_NONE;
	uart2.Init.Mode = UART_MODE_TX_RX;
	uart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart2.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&uart2);
	
}

static void Netif_Config(void)
{
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;

  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

  /* add the network interface */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /*  Registers the default network interface */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernetif_update_config);
}

/**
  * @brief  EXTI line detection callbacks
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//  /* Get the IT status register value */
//  if(BSP_IO_ITGetStatus(MII_INT_PIN))
  if (GPIO_Pin == ENC_INT_PIN)
  {
    ethernetif_set_link(&gnetif);
  }
  //BSP_IO_ITClear();
}



void Error_Handler(void)
{

}


void Update_coils(void)
{
	int state = 0; 
	for ( int i = 0; i < MAX_coils; i++)
	{
		if((!(i%8))&& (i > 0))state+=1;
		if (Outputs[state] & (1<<(i-(state*8)))) HAL_GPIO_WritePin(OUTPUT_PORT[i], OUTPUT_Pin[i], GPIO_PIN_SET);
		else HAL_GPIO_WritePin(OUTPUT_PORT[i], OUTPUT_Pin[i], GPIO_PIN_RESET);
	}
	
}

void Update_discrete_inputs(void)
{
	int state = 0; 
	for ( int i = 0; i < MAX_discrate_inputs; i++)
	{
		if((!(i%8))&& (i > 0))state+=1;
		if(Inputs_counter[i] == 0);
		else if(!(Counter % Inputs_counter[i]))
		{
			if(HAL_GPIO_ReadPin(INPUT_PORT[i], INPUT_Pin[i]) == GPIO_PIN_RESET)Inputs[state] |= 1<<(i-state*8);
			else Inputs[state] &= ~(1<<(i-state*8));
		}
	}
}

void Update_registers(void)
{
	
	for(int i = 0 ; i<MAX_discrate_inputs; i++)
	{
		int k = 0;
		if((i%4)&&(i!=0))k+=1;
		Inputs_counter[i] = (Registers[k]& (0x0F<<(4*(i-4*k))));	
	}
	for(int i = 0 ; i<MAX_input_registers; i++)
	{
		int k = 0;
		if((i%4)&&(i!=0))k+=1;
		Ainputs_counter[i] = (Registers[4+k] & (0x0F<<(4*(i-4*k))));//(Registers[4+k]&(0b1111<<(i - 4*k)));	
	}
	DAC->DHR12R1 = Registers[8];
	DAC->DHR12R2 = Registers[9];
	static char buffor[16] ;
	if(buffor[0] != '\0')
	{
	Send_string(buffor);
	buffor[0] = '\0';
	}
	
}
void Update_inputs_registers(void)
{
	{
		for(int i = 0; i<MAX_input_registers; i++)
		{
		 if(Ainputs_counter[i] == 0);
		 else if(!(Counter % Ainputs_counter[i]))
		 {
			 uint16_t temp = 0;
			 for (int j = 0; j <4 ;j++) temp +=  Adc_buffor[i+6*j];	
			 Ainputs[i] =temp/4;
		 }
		}
	}
}

void Send_string(char* s)
{
 HAL_UART_Transmit(&uart2, (uint8_t*)s, strlen(s), 1000);
}
void Set_LED(unsigned char Which_LED,unsigned char OnOff)
{
	switch(Which_LED)
	{
	case LED_BLUE:
		if(OnOff == 1) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		break;
	case LED_YELLOW:
		if(OnOff == 1) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		break;
	case LED_RED:
		if(OnOff == 1) HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		break;
	case LED_GREEN:
		if(OnOff == 1) HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		break;
	case LED_RED2:
		if(OnOff == 1) HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		break;
	case ALL_LED:
		if(OnOff == 1)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		}
	default:
		break;
	}
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
