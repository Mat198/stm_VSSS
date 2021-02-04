/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Arquivo           : main.c
  *
  * Descrição         : Código do microcontrolador ARM para os robôs VSSS da
  * 					Equipe Red Dragons UFSCar. Desenvolvido por Matheus
  * 					Sousa Soares em 2020.
  *
  * 					Para contato: matheussousasf@gmail.com ou
  * 								  matheussousa@df.ufscar.br
  * 								  (31) 975340460
  *
  ******************************************************************************
  ******************************************************************************
  *
  *	Coisas para resolver:
  *
  *
  * Implementar o controle do robo
  *
  *
  *
  ******************************************************************************
  ******************************************************************************
  * Código desenvolvido na CUBE IDE 1.3.0
  *
  * Lista de funções desempenhadas:
  * Comunicação UART por interrupção com módulo Xbee OK
  * Leitura e interpretação da mensagem recebida OK
  * Geração de PWM para motores OK
  * Leitura dos Encoders OK
  * Controle da velocidade dos motores com PWM gerado X
  *
  * ****************************************************************************
  * Como utilizar:
  *
  * Editar nas partes com "USER CODE" o resto do código é gerado automaticamente
  * pela parte de configuração do microcontrolador na IDE (arquivo .ioc).
  *
  * Organize e comente as suas funções e códigos adequadamente para facilitar
  * para os membros futuros da equipe. Utilize o Git para atualizar o código.
  *
  * ***************************************************************************
  * Partes do código
  * USER CODE Header: Comentários de introdução
  * USER CODE Includes: Bibliotecas do usuário. Utilizando math.h
  * USER CODE PV: Declaração de variáveis
  * USER CODE 0: Vazio
  * USER CODE 1: Vazio
  * USER CODE SysInit: Inicialização do sistema
  * USER CODE 2: Vazio
  * USER CODE WHILE: Loop de programa
  * USER CODE 3: Vazio
  * USER CODE 4: Funções de interrupção, timers e comunicação
  *
  * Preencher aqui conforme o código for atualizado, por favor.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Variáveis gerais
static int true = 1;
static int false = 0;
uint16_t delay = 500;
//double velocidadesM1[150]= {};
//double velocidadesM2[150]= {};
int cont_vel = 0;
uint8_t buffer_vel[6] = {};

//Variáveis de comunicação serial
uint8_t size_msg = 10;
uint8_t tx_buffer []={'L','i','g','a','d','o',' ',' ',' ','\n'}; // Msg inicial enviada pelo robô
uint8_t rx_buffer[10]; //Buffer que recebe as mensagens

// Variáveis de controle dos motores
uint16_t duty_cycle = 0;
int speedM1 = 0;
int speedM2 = 0;
int pwmM1 = 0;
int pwmM2 = 0;
int sentidoM1 = 0;
int sentidoM2 = 0;
int Tloop = 0.01;

double w_rodaM1[2] = {};
double w_rodaM2[2] = {};
double w_targetM1 = 20;
double w_targetM2 = 20;

double acaoM1[2] = {0,0};
double acaoM2[2] = {0,0};
double erroM1[3] = {0,0,0};
double erroM2[3] = {0,0,0};

int pulse_cnt = 3200; // Contagem de pulsos do clock. É a resolução do PWM
int cont_controle = 0;
int npulsos = 12;
int cppM1 = 0;
int cppM2 = 0;
int flag_controle = 0;

// Valores do controlador
double Kp = 1.5;
double Kd = 0.1;
double Ki = 0.5;
double Kpwm = 2350*8.165/21.0;


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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit_IT(&huart2, tx_buffer, size_msg); // envia a msg inicial
  HAL_UART_Receive_IT(&huart2, rx_buffer, size_msg);// Faz com que o robô ligue o receptor de msg
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Inicia o Timer 2 do PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2); // Inicia o Timer 2 no modo de interrupção.
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Inicia o encoder M1
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //Inicia o módulo de leitura do encoder M2
  HAL_GPIO_WritePin(Led_i_GPIO_Port, Led_i_Pin, SET);// Desliga o led da placa de desenvolvimento
  HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, RESET);
  HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, RESET);
  HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, RESET);
  HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, RESET);

  /*
  HAL_Delay(1000);
  HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, SET);
  HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, RESET);
  HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, SET);
  HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, RESET);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 3200);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 3200);
  */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	  while (1)
	  {

		  if(flag_controle == true){
			  flag_controle = false;

			  HAL_GPIO_TogglePin(Led_g_GPIO_Port,Led_g_Pin);
			  //HAL_GPIO_TogglePin(Led_y_GPIO_Port,Led_y_Pin);

			  // Checagem do sentido de rotação das rodas e calculo da velocidade

			  if (TIM3->CNT >= cppM1){
				  sentidoM1 = 1;
				  w_rodaM1[0] = (2*M_PI*(TIM3->CNT-cppM1))/(npulsos);
			  }

			  if (TIM3->CNT < cppM1) {
				  sentidoM1 = 0;
				  w_rodaM1[0] = (2*M_PI*(cppM1-TIM3->CNT))/(npulsos);
			  }

			  if (TIM4->CNT >= cppM2){
				  sentidoM2 = 1;
				  w_rodaM2[0] = (2*M_PI*(TIM4->CNT-cppM2))/(npulsos);
			  }

			  if (TIM4->CNT < cppM2){
				  sentidoM2 = 0;
				  w_rodaM2[0] = (2*M_PI*(cppM2-TIM4->CNT))/(npulsos);
			  }

			  // Filtro para o estouro do encoder
			  if (w_rodaM1[0] > 35) w_rodaM1[0] = w_rodaM1[1];
			  if (w_rodaM2[0] > 35) w_rodaM2[0] = w_rodaM2[1];

			  w_rodaM1[1] = w_rodaM1[0];
			  w_rodaM2[1] = w_rodaM2[0];

			  erroM1[0] = w_targetM1 - w_rodaM1[0];
			  erroM2[0] = w_targetM2 - w_rodaM2[0];

			  acaoM1[0] = acaoM1[1] + (Kp + Kd/Tloop + Ki*Tloop)*erroM1[0] + (-Kp - 2*Kd/Tloop)*erroM1[1] + (Kd/Tloop)*erroM1[2];
			  acaoM2[0] = acaoM2[1] + (Kp + Kd/Tloop + Ki*Tloop)*erroM1[0] + (-Kp - 2*Kd/Tloop)*erroM2[1] + (Kd/Tloop)*erroM2[2];

			  pwmM1 = ceil(acaoM1[0]*Kpwm);
			  pwmM2 = ceil(acaoM2[0]*Kpwm);

			  if (pwmM1 > 3200)	pwmM1 = 3200;
			  if (pwmM2 > 3200) pwmM2 = 3200;

			  /*
			  // Ajusta o valor do duty cycle do PWM atualizando a velocidade do robô
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmM1);
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwmM2);
			  */

			  // Atualizando variáveis para o próximo loop de controle
			  cppM1 = TIM3->CNT;
			  cppM2 = TIM4->CNT;

			  erroM1[2] = erroM1[1];
			  erroM2[2] = erroM1[1];

			  erroM1[1] = erroM1[0];
			  erroM2[1] = erroM1[0];

			  acaoM1[1] = acaoM1[0];
			  acaoM2[1] = acaoM1[0];
/*
			  // Atualizando variáveis para o próximo loop de controle
			  cppM1 = TIM3->CNT;
			  cppM2 = TIM4->CNT;

			  velocidadesM1[cont_vel] = w_rodaM1[0];
			  velocidadesM2[cont_vel] = w_rodaM2[0];
			  cont_vel ++;

			  HAL_GPIO_TogglePin(Led_g_GPIO_Port,Led_g_Pin);

			  if (cont_vel == 150){
				  HAL_TIM_Base_Stop_IT(&htim2);
				  HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, RESET);
				  HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, RESET);
				  HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, RESET);
				  HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, RESET);
				  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0000);
				  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0000);

				  uint8_t motor1[] = {'M','O','T','O','R','_','1',' ','\n','\n'};
				  uint8_t motor2[] = {'M','O','T','O','R','_','2',' ','\n','\n'};

				  HAL_UART_Transmit_IT(&huart2, motor1, 10);

				  for(int j=0;j<150;j++){
					  int aux = 0;
					  int aux1 = 0;
					  aux1 = floor(velocidadesM1[j]*100);

					  for(int i = 0; i<5; i++ ){

						  if(i == 4){
							  aux = aux1;
						  }
						  else{
							  aux = aux1/pow(10,4-i);
							  aux1 = aux1- aux*pow(10,4-i);
						  }

						  buffer_vel[i] = aux +'0';

					  }
					  buffer_vel[5] = '\n';
					  HAL_UART_Transmit_IT(&huart2, buffer_vel, 6);
				  }

				  HAL_Delay(1000);

				  HAL_UART_Transmit_IT(&huart2, motor2, 10);

				  for(int j=0;j<150;j++){
					  int aux = 0;
					  int aux1 = 0;
					  aux1 = floor(velocidadesM2[j]*100);

					  for(int i = 0; i<5; i++ ){

						  if(i == 4){
							  aux = aux1;
						  }
						  else{
							  aux = aux1/pow(10,4-i);
							  aux1 = aux1- aux*pow(10,4-i);
						  }

						  buffer_vel[i] = aux +'0';

					  }
					  buffer_vel[5] = '\n';
					  HAL_UART_Transmit_IT(&huart2, buffer_vel, 6);
					  }
			  }*/
	  }

	  /*
	   if(retorno == true){

	  		for(int i = 0; i<10; i++ ){ // Clona e envia a msg recebida
	  			tx_buffer[i] = rx_buffer[i];
	  		}
	  		HAL_UART_Transmit_IT(&huart2, tx_buffer, 10);

		}
	    */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Led_i_Pin|Led_g_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led_y_Pin|Led_r_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M2B_Pin|M2A_Pin|M1A_Pin|M1B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Led_i_Pin Led_g_Pin */
  GPIO_InitStruct.Pin = Led_i_Pin|Led_g_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_y_Pin Led_r_Pin */
  GPIO_InitStruct.Pin = Led_y_Pin|Led_r_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M2B_Pin M2A_Pin M1A_Pin M1B_Pin */
  GPIO_InitStruct.Pin = M2B_Pin|M2A_Pin|M1A_Pin|M1B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ // Função chaamada quando o buffer de recepção está CHEIO

	HAL_GPIO_TogglePin(Led_r_GPIO_Port,Led_r_Pin);

	//####################################################
	if(rx_buffer[0] == 'G'){// Função que inicia o PWM
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		}

	//####################################################
	// Interpreta a msg e converte em velocidade
	speedM1 = 1000*(rx_buffer[1]-'0') + 100*(rx_buffer[2]-'0') + 10*(rx_buffer[3]-'0') + (rx_buffer[4]-'0');
	// Interpreta a msg e converte em velocidade
	speedM2 = 1000*(rx_buffer[6]-'0') + 100*(rx_buffer[7]-'0') + 10*(rx_buffer[8]-'0') + (rx_buffer[9]-'0');

	w_targetM1 = speedM1/100;
	w_targetM2 = speedM2/100;


	if (speedM1 > 3200){
		speedM1 = 3200;
	}
	if(speedM2 > 3200){
		speedM2 = 3200;
	}

		// Ajusta o valor do duty cycle do PWM atualizando a velocidade do robô
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speedM1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speedM2);


	//####################################################
	//Parte que lida com o motor 1
	if(rx_buffer[0] == 'F'){// Função vira para a direita

		HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, SET);
		HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, RESET);
		//HAL_GPIO_TogglePin(Led_g_GPIO_Port,Led_g_Pin);
	}
	if(rx_buffer[0] == 'T'){// Função vira para a esquerda

		HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, RESET);
		HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, SET);
		//HAL_GPIO_TogglePin(Led_y_GPIO_Port,Led_y_Pin);

	}
	if(rx_buffer[0] == 'S'){// Função que par aos motores
		//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		//HAL_GPIO_TogglePin(Led_r_GPIO_Port,Led_r_Pin);

		//HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, RESET);
		//HAL_GPIO_WritePin(M1A_GPIO_Port, M1B_Pin, RESET);
		speedM2 = 0;
		}

	//####################################################
	//Parte que lida com o motor 2
	if(rx_buffer[5] == 'F'){// Função vira para a direita

		HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, SET);
		HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, RESET);
		//HAL_GPIO_TogglePin(Led_g_GPIO_Port,Led_g_Pin);
	}
	if(rx_buffer[5] == 'T'){// Função vira para a esquerda

		HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, RESET);
		HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, SET);
		//HAL_GPIO_TogglePin(Led_y_GPIO_Port,Led_y_Pin);

	}
	if(rx_buffer[5] == 'S'){// Função que par aos motores
		//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		//HAL_GPIO_TogglePin(Led_r_GPIO_Port,Led_r_Pin);
		HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, RESET);
		HAL_GPIO_WritePin(M2A_GPIO_Port, M2B_Pin, RESET);
		speedM2 = 0;
		}

	//####################################################

	else{
		__NOP();// Adicionado para fins de Debug portanto não faz nada
	}

	//####################################################

	HAL_UART_Receive_IT(&huart2, rx_buffer, size_msg);// Liga a recepção para a próxima msg


	HAL_GPIO_TogglePin(Led_r_GPIO_Port,Led_r_Pin);

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){// Função executada depois que a msg é totalmente enviada
	__NOP();

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_TogglePin(Led_i_GPIO_Port, Led_i_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	cont_controle++;
	if (cont_controle == 200){
		cont_controle = 0;
		flag_controle = true;
		HAL_GPIO_TogglePin(Led_y_GPIO_Port,Led_y_Pin);
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
