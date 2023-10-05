/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
enum SIMO_Register_Addresses {
							 CID = 0x14, SBB1_A = 0x2B, SBB1_B = 0x2C,
						 	 SBB2_A = 0x2D, SBB2_B = 0x2E, SBB_TOP = 0x2F,
							 LDO0_A = 0x38, LDO0_B = 0x39, LDO1_A = 0x3A,
							 LDO1_B = 0x3B
							 };

enum Charger_Register_Addresses {
								CHG_B = 0x21, CHG_C = 0x22, CHG_D = 0x23,
								CHG_E = 0x24, CHG_G = 0x26, CHG_I = 0x28,
								CHG_STAT_B = 0x03
								};

enum Global_Register_Addresses {
								GLBL = 0x10,
							   };
enum Charger_Status_Values {
							CHG_OK = 0, CHG_DISCON = 1, CHG_ERR = 2, CHG_FULL = 3
						   };
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t SlaveAddress = 0x48 << 1; //0x90
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
	uint8_t Check_Chip_ID(uint8_t *Buffer);
	uint8_t Local_I2C_Transmit(uint8_t *Buffer, uint8_t Bytes);
	uint8_t Local_I2C_Receive(uint8_t *Buffer, uint8_t Bytes);
	uint8_t I2C_Scanner(uint8_t *Buffer);
	uint8_t SIMO_SetSBB(uint8_t SBBx_A_Address, uint8_t *Buffer ,uint8_t voltage);
	uint8_t SIMO_PMIC_Init(uint8_t *Buffer);
	uint8_t PMIC_Write_Reg(uint8_t Reg_Address, uint8_t data, uint8_t *Buffer);
	uint8_t PMIC_Read_Reg(uint8_t Reg_Address, uint8_t *Buffer);
	void Charger_Init(uint8_t *Buffer);
	void Global_Config_Init(uint8_t *Buffer);
	uint8_t Check_Charger_Status(uint8_t *Buffer);
	void Suspend_Charger(uint8_t *Buffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	/*Charger_Init Function*/
	void Charger_Init(uint8_t *Buffer)
	{
		//setting charger VCHGIN-MIN = 4.7V, ICHGIN-LIM = 475mA, i_PQ = 20%, CHG_EN = enable
		PMIC_Write_Reg(CHG_B, 0xF3, Buffer);
		//setting CHG_PQ(V_PQ) = 2.3V, I_TERM = 10% of IFAST_CHG(300mA), topoff timer 0 min
		PMIC_Write_Reg(CHG_C, 0x10, Buffer);
		//setting die junction temp at 60degree celsius, system regulation voltage(VSYS-REG) = 4.5V
		PMIC_Write_Reg(CHG_D, 0x10, Buffer);
		//setting CHG_CC(I_FAST-CHG) = 300mA, fast charge safety timer is 5hours
		PMIC_Write_Reg(CHG_E, 0x9E, Buffer);
		//setting CHG_CV(V_FAST-CHG) = 4.25V, USBS = not suspended
		PMIC_Write_Reg(CHG_G, 0x68, Buffer);
		//setting battery discharge current full sacle current value = 8.2mA, AMUX = disabled
		PMIC_Write_Reg(CHG_I, 0x00, Buffer);
	}
	/*End*/

	/*Check_Charger_Status Function*
	 @retval 0 - Charger OK and charging
	 	 	 1 - Charger OFf or Disconnected
	 	 	 2 - Charger Connected and enabled but not charging
	 	 	 3 - Charger done state*/
	uint8_t Check_Charger_Status(uint8_t *Buffer)
	{
		uint8_t status, ret;
		status = PMIC_Read_Reg(CHG_STAT_B, Buffer);
		if((status & 0xF0) == 0x80)
			ret = CHG_FULL;
		else if((status & 0xF0) != 0x00)
		{
			if((status & 0x0F) == 0x0E)
				ret  = CHG_OK; //charger debounced and charging OK
			else
				ret = CHG_ERR; //either charger in ULVO || OVP || i/p is being debounced || Charging Not-OK
		}
		else
			ret = CHG_DISCON; //Charger Disconnected or Charger OFF

		return ret;
	}
	/*End*/

	/*Suspend Charger Function*/
	void Suspend_Charger(uint8_t *Buffer)
	{
		//set USBS bit to suspend mode to disconnect charger
		PMIC_Write_Reg(CHG_G, 0x6A, Buffer);
	}
	/*End*/

	/*Global_Config_Init Function*/
	void Global_Config_Init(uint8_t *Buffer)
	{
		//Software Cold reset
		PMIC_Write_Reg(GLBL, 0x01, Buffer);
		//Main Bias force Enable by Software
		PMIC_Write_Reg(GLBL, 0x10, Buffer);
	}
	/*End*/


	/*PMIC_Write_Reg Function*/
	uint8_t PMIC_Write_Reg(uint8_t Reg_Address, uint8_t data, uint8_t *Buffer)
	{
		uint8_t ret;
		Buffer[0] = Reg_Address;
		Buffer[1] = data;
		ret = Local_I2C_Transmit(Buffer, 2);
		return ret;
	}
	/*End*/

	/*PMIC_Read_Reg Function*/
	uint8_t PMIC_Read_Reg(uint8_t Reg_Address, uint8_t *Buffer)
	{
		uint8_t ret;
		Buffer[0] = Reg_Address;
		ret = Local_I2C_Receive(Buffer, 1);
		return ret;
	}
	/*End*/


	/*SIMO_PMIC_Init Function*/
	uint8_t SIMO_PMIC_Init(uint8_t *Buffer)
	{
		uint8_t received;
		/*Initializing SBB Drive strength slower*/
		received = PMIC_Write_Reg(SBB_TOP, 0x02, Buffer);
		return received;
	}
	/*End*/
#if 0
	/*I2C_Scanner Function*/
	uint8_t I2C_Scanner(uint8_t *Buffer)
	{
		uint8_t i,received,ret;
	for(i = 1; i < 200; i++)
	  {
		  received = HAL_I2C_Master_Receive(&hi2c1, i, Buffer, 1, 100);
		  if(received != HAL_OK)
		  {
			  ret = 1;
		  }
		  else
		  {
			  ret = 0;
			  break;
		  }
	  }
	  return ret;

	}
#endif
	/*Local_I2C_Transmit Function
	  @retval - 1 - I2C trasmission error
	  	  	  	transmit_val - returns transmitted value*/
	uint8_t Local_I2C_Transmit(uint8_t *Buffer, uint8_t Bytes)
	{
		uint8_t transmit,ret;
		transmit = HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, Buffer, Bytes, HAL_MAX_DELAY);
		if(transmit != HAL_OK)
		{
			ret = 1;
		}
		else
		{
			ret = Local_I2C_Receive(Buffer, 1);

		}
		return ret;
	}
	/*END*/

	/*Local_I2C_Receive Function
	  @retval 1 - I2C transmission error
	  	  	  Receive_val[Buffer] - return the recieved value*/
	uint8_t Local_I2C_Receive(uint8_t *Buffer, uint8_t Bytes)
	{
		uint8_t received,ret;
		ret = HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, Buffer, Bytes, HAL_MAX_DELAY);
		received = HAL_I2C_Master_Receive(&hi2c1, SlaveAddress, Buffer, Bytes, HAL_MAX_DELAY);
		if(received != HAL_OK)
			ret = 1;
		else
		{
			ret = Buffer[0];
		}
		return ret;
	}
	/*END*/

	/*Check_Chip_ID Function
	  @retval 0 - Correct Chip ID
	  	  	  1 - Incorrect Chip ID*/
	uint8_t Check_Chip_ID(uint8_t *Buffer)
	{
		uint8_t ret, Id;
		Id = PMIC_Read_Reg(CID, Buffer);
		Id &= 0x0F;
		if(Id == 0x06)
			ret = 0; //Chip ID is correct
		else
			ret = 1; // Chip ID is Incorrect

		return ret;
	}
	/*End*/

	/*SIMO_SetSBB Function*/
	uint8_t SIMO_SetSBB(uint8_t SBBx_A_Address, uint8_t *Buffer ,uint8_t voltage)
	{
		uint8_t val;
			//SBB_A_Address + 1 = SBB_B_Address
			//Buck Mode, 333mA current, active discharge enabled, enabled irrespective of FPS
			val = PMIC_Write_Reg(SBBx_A_Address + 1, 0x7F, Buffer);
			if(val != 0x7F)
				return 1; //i2c transmission error
			//SBB_A_Address
			//voltage to set at SBBx
			val = PMIC_Write_Reg(SBBx_A_Address, voltage, Buffer);
			if(val != voltage)
				return 1; //i2c transmission error
			else
			{
				//ReadBack_SIMO(val);
				return 0; //OK
			}
	}

	uint8_t SIMO_SetLDO(uint8_t LDOx_A_Address, uint8_t *Buffer ,uint8_t voltage)
		{
			uint8_t val;
				//LDO_A_Address + 1 = LDO_B_Address
				//Buck Mode, 333mA current, active discharge enabled, enabled irrespective of FPS
				val = PMIC_Write_Reg(LDOx_A_Address + 1, 0x0E, Buffer);
				if(val != 0x0E)
					return 1; //i2c transmission error
				//LDO_A_Address
				//voltage to set at LDOxx
				val = PMIC_Write_Reg(LDOx_A_Address, voltage, Buffer);
				if(val != voltage)
					return 1; //i2c transmission error
				else
				{
					//ReadBack_SIMO(val);
					return 0; //OK
				}
		}
	/*End*/
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
  MX_I2C1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  uint8_t Buffer[20] = {0};

  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_SET); //set nEN pin to HIGH


  if(Check_Chip_ID(Buffer)) //check the chip id of max77654 pmic which should be 0xX6
	 Error_Handler();
  else
  {
  Global_Config_Init(Buffer); //software cold reset and main bias enable by software

  SIMO_PMIC_Init(Buffer);

  Charger_Init(Buffer); //configured charger for fast charging

  SIMO_SetSBB(SBB1_A, Buffer, 0x08);
  SIMO_SetSBB(SBB2_A, Buffer, 0x32);

  SIMO_SetLDO(LDO0_A, Buffer, 0x28);
  SIMO_SetLDO(LDO1_A, Buffer, 0x44);

  HAL_TIM_Base_Start_IT(&htim14);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
	  uint8_t Buffer[30] = {0};
	  if(htim == &htim14)
	  {
		  switch(Check_Charger_Status(Buffer))
		  {
		  	  case CHG_OK:
		  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12); //set LED1 HIGH
		  		  break;
		  	  case CHG_DISCON:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //set LED1 LOW
				  break;
		  	  case CHG_ERR:
		  		  Error_Handler();
		  		  break;
		  	  case CHG_FULL:
		  		  Suspend_Charger(Buffer);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //set LED1 HIGH
				  break;
		  	  default:
		  		  break;
		  }
	  }
	 // Check_TIM_Delay();
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
