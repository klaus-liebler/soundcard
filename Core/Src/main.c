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
#include "tusb.h"
#include "waveforms.h"
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
I2C_HandleTypeDef hi2c1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
	BLINK_NOT_MOUNTED = 250,
	BLINK_MOUNTED = 1000,
	BLINK_SUSPENDED = 2500,
};
static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;


#define AUDIO_SAMPLE_RATE   48000

// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; 	// +1 for master channel 0
uint16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // +1 for master channel 0
uint32_t sampFreq;
uint8_t clkValid;

// Range states
audio_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX+1];// Volume range state
audio_control_range_4_n_t(1) sampleFreqRng;// Sample frequency range state

// Audio test data
uint16_t i2s_dummy_buffer[CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO][49*2];   // Ensure half word aligned

static inline void board_vbus_sense_init(void) {
	// Blackpill doesn't use VBUS sense (B device) explicitly disable it
	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
	USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
	USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
}

void led_blinking_task(void)
{
	static uint32_t start_ms = 0;
	static bool led_state = false;

	// Blink every interval ms
	if ( HAL_GetTick() - start_ms < blink_interval_ms) return; // not enough time
	start_ms += blink_interval_ms;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, led_state);
	led_state = 1 - led_state; // toggle
}




//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{
	// Yet to be filled - e.g. put meas data into TX FIFOs etc.
	// asm("nop");
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
	(void) rhport;
	(void) pBuff;

	// We do not support any set range requests here, only current value requests
	TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t ep = TU_U16_LOW(p_request->wIndex);

	(void) channelNum; (void) ctrlSel; (void) ep;

	return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
	(void) rhport;
	(void) pBuff;

	// We do not support any set range requests here, only current value requests
	TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t itf = TU_U16_LOW(p_request->wIndex);

	(void) channelNum; (void) ctrlSel; (void) itf;

	return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
	(void) rhport;

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t itf = TU_U16_LOW(p_request->wIndex);
	uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

	(void) itf;

	// We do not support any set range requests here, only current value requests
	TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

	// If request is for our feature unit
	if ( entityID == 2 )
	{
		switch ( ctrlSel )
		{
		case AUDIO_FU_CTRL_MUTE:
			// Request uses format layout 1
			TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

			mute[channelNum] = ((audio_control_cur_1_t*) pBuff)->bCur;

			TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
			return true;

		case AUDIO_FU_CTRL_VOLUME:
			// Request uses format layout 2
			TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

			volume[channelNum] = (uint16_t) ((audio_control_cur_2_t*) pBuff)->bCur;

			TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
			return true;

			// Unknown/Unsupported control
		default:
			TU_BREAKPOINT();
			return false;
		}
	}
	return false;    // Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void) rhport;

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t ep = TU_U16_LOW(p_request->wIndex);

	(void) channelNum; (void) ctrlSel; (void) ep;

	//	return tud_control_xfer(rhport, p_request, &tmp, 1);

	return false; 	// Yet not implemented
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void) rhport;

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t itf = TU_U16_LOW(p_request->wIndex);

	(void) channelNum; (void) ctrlSel; (void) itf;

	return false; 	// Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void) rhport;

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	// uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
	uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

	// Input terminal (Microphone input)
	if (entityID == 1)
	{
		switch ( ctrlSel )
		{
		case AUDIO_TE_CTRL_CONNECTOR:
		{
			// The terminal connector control only has a get request with only the CUR attribute.
			audio_desc_channel_cluster_t ret;

			// Those are dummy values for now
			ret.bNrChannels = 1;
			ret.bmChannelConfig = (audio_channel_config_t) 0;
			ret.iChannelNames = 0;

			TU_LOG2("    Get terminal connector\r\n");

			return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));
		}
		break;

		// Unknown/Unsupported control selector
		default:
			TU_BREAKPOINT();
			return false;
		}
	}

	// Feature unit
	if (entityID == 2)
	{
		switch ( ctrlSel )
		{
		case AUDIO_FU_CTRL_MUTE:
			// Audio control mute cur parameter block consists of only one byte - we thus can send it right away
			// There does not exist a range parameter block for mute
			TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
			return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);

		case AUDIO_FU_CTRL_VOLUME:
			switch ( p_request->bRequest )
			{
			case AUDIO_CS_REQ_CUR:
				TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
				return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));

			case AUDIO_CS_REQ_RANGE:
				TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);

				// Copy values - only for testing - better is version below
				audio_control_range_2_n_t(1)
				ret;

				ret.wNumSubRanges = 1;
				ret.subrange[0].bMin = -90;           // -90 dB
				ret.subrange[0].bMax = 90;		// +90 dB
				ret.subrange[0].bRes = 1; 		// 1 dB steps

				return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));

				// Unknown/Unsupported control
			default:
				TU_BREAKPOINT();
				return false;
			}
			break;

			// Unknown/Unsupported control
			default:
				TU_BREAKPOINT();
				return false;
		}
	}

	// Clock Source unit
	if ( entityID == 4 )
	{
		switch ( ctrlSel )
		{
		case AUDIO_CS_CTRL_SAM_FREQ:
			// channelNum is always zero in this case
			switch ( p_request->bRequest )
			{
			case AUDIO_CS_REQ_CUR:
				TU_LOG2("    Get Sample Freq.\r\n");
				return tud_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));

			case AUDIO_CS_REQ_RANGE:
				TU_LOG2("    Get Sample Freq. range\r\n");
				return tud_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));

				// Unknown/Unsupported control
			default:
				TU_BREAKPOINT();
				return false;
			}
			break;

			case AUDIO_CS_CTRL_CLK_VALID:
				// Only cur attribute exists for this request
				TU_LOG2("    Get Sample Freq. valid\r\n");
				return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

				// Unknown/Unsupported control
			default:
				TU_BREAKPOINT();
				return false;
		}
	}

	TU_LOG2("  Unsupported entity: %d\r\n", entityID);
	return false; 	// Yet not implemented
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
	(void) rhport;
	(void) itf;
	(void) ep_in;
	(void) cur_alt_setting;

	for (uint8_t fifo=0; fifo < CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO; fifo++)
	{
		tud_audio_write_support_ff(fifo, i2s_dummy_buffer[fifo], CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_CHANNEL_PER_FIFO_TX * ((AUDIO_SAMPLE_RATE/1000)));
	}

	return true;
}

uint8_t sineTableIndex[4]={0,0,0,0};

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
	(void) rhport;
	(void) n_bytes_copied;
	(void) itf;
	(void) ep_in;
	(void) cur_alt_setting;


	/* USER CODE BEGIN 1 */

	// Generate dummy data
	 uint16_t dataVal;
	  for (uint16_t cnt = 0; cnt < CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO; cnt++)
	  {
	    uint16_t * p_buff = i2s_dummy_buffer[cnt];              // 2 bytes per sample
	    dataVal = 1;
	    for (uint16_t cnt2 = 0; cnt2 < AUDIO_SAMPLE_RATE/1000; cnt2++)
	    {
	      for (uint8_t cnt3 = 0; cnt3 < CFG_TUD_AUDIO_FUNC_1_CHANNEL_PER_FIFO_TX; cnt3++)
	      {
	        *p_buff++ = dataVal;
	      }
	      dataVal++;
	    }
	  }
	return;
	for(int i=0;i<48;i++){
		i2s_dummy_buffer[0][2*i]   = 100+i;
		i2s_dummy_buffer[0][2*i+1] = 200+i;
		i2s_dummy_buffer[1][2*i]   = 300+i;
		i2s_dummy_buffer[1][2*i+1] = 400+i;

	}

	return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void) rhport;
	(void) p_request;

	return true;
}





/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

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
	MX_USB_OTG_FS_PCD_Init();
	/* USER CODE BEGIN 2 */




	// init device stack
	board_vbus_sense_init();
	TU_ASSERT(tusb_init());

	// Init values
	sampFreq = AUDIO_SAMPLE_RATE;
	clkValid = 1;

	sampleFreqRng.wNumSubRanges = 1;
	sampleFreqRng.subrange[0].bMin = AUDIO_SAMPLE_RATE;
	sampleFreqRng.subrange[0].bMax = AUDIO_SAMPLE_RATE;
	sampleFreqRng.subrange[0].bRes = 0;







	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		tud_task(); // tinyusb device task
		led_blinking_task();
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */


	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void)
{

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
	__disable_irq();
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
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
