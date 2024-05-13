#include <cstdio>
#include <cstring>
#include <cctype>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "firfilter.hh"
#include "mpu6050.hh"
#include "tusb.h"
#include "stm32f4xx_hal.h"
#include "log.h"
#include "cmsis_os.h"


enum class BLINK {
	NOT_MOUNTED = 250, MOUNTED = 1000, SUSPENDED = 2500,
};
static BLINK blink_interval_ms = BLINK::NOT_MOUNTED;

constexpr uint32_t AUDIO_SAMPLE_RATE { 8000 };

// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; 	// +1 for master channel 0
uint16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // +1 for master channel 0
uint32_t sampFreq;
uint8_t clkValid;

// Range states
//audio_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // Volume range state
audio_control_range_4_n_t(1) sampleFreqRng; // Sample frequency range state


uint16_t audio_buffer[CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO][9 * 2]; // Ensure half word aligned


constexpr size_t SENSOR_DATA_SIZE{32};

int16_t sensor_data_buffer[2][SENSOR_DATA_SIZE];
int16_t filter_data_buffer[2][SENSOR_DATA_SIZE];
int16_t lastValidSensorData{0};
int16_t lastValidFilterData{0};


size_t data_doublebuffer_index{0};
size_t data_offset{0};


//Realtime
extern osMutexId_t mtxBufferHandle;

static void board_vbus_sense_init(void) {
	// Blackpill doesn't use VBUS sense (B device) explicitly disable it
	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
	USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
	USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
}

void led_blinking_task(void) {
	static uint32_t start_ms = 0;
	static GPIO_PinState led_state = GPIO_PIN_RESET;

	// Blink every interval ms
	if (HAL_GetTick() - start_ms < (uint32_t) blink_interval_ms)
		return; // not enough time
	start_ms += (uint32_t) blink_interval_ms;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, led_state);
	led_state = led_state == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
	blink_interval_ms = BLINK::MOUNTED;
	log_info("tud_mount_cb");
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
	blink_interval_ms = BLINK::NOT_MOUNTED;
	log_info("tud_umount_cb");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
	(void) remote_wakeup_en;
	blink_interval_ms = BLINK::SUSPENDED;
	log_info("tud_suspend_cb");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
	blink_interval_ms = tud_mounted() ? BLINK::MOUNTED : BLINK::NOT_MOUNTED;
	log_info("tud_resume_cb");
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
	(void) rhport;
	(void) pBuff;

	// We do not support any set range requests here, only current value requests
	TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t ep = TU_U16_LOW(p_request->wIndex);

	(void) channelNum;
	(void) ctrlSel;
	(void) ep;

	return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
	(void) rhport;
	(void) pBuff;

	// We do not support any set range requests here, only current value requests
	TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t itf = TU_U16_LOW(p_request->wIndex);

	(void) channelNum;
	(void) ctrlSel;
	(void) itf;

	return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
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
	if (entityID == 2) {
		switch (ctrlSel) {
		case AUDIO_FU_CTRL_MUTE:
			// Request uses format layout 1
			TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

			mute[channelNum] = ((audio_control_cur_1_t*) pBuff)->bCur;

			log_info("Set Mute: %d of channel: %u", mute[channelNum], channelNum);
			return true;

		case AUDIO_FU_CTRL_VOLUME:
			// Request uses format layout 2
			TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

			volume[channelNum] = (uint16_t) ((audio_control_cur_2_t*) pBuff)->bCur;

			log_info("Set Volume: %d dB of channel: %u", volume[channelNum], channelNum);
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
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
	(void) rhport;

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t ep = TU_U16_LOW(p_request->wIndex);

	(void) channelNum;
	(void) ctrlSel;
	(void) ep;

	//	return tud_control_xfer(rhport, p_request, &tmp, 1);

	return false; 	// Yet not implemented
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
	(void) rhport;

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t itf = TU_U16_LOW(p_request->wIndex);

	(void) channelNum;
	(void) ctrlSel;
	(void) itf;

	return false; 	// Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
	(void) rhport;

	// Page 91 in UAC2 specification
	uint8_t channelNum = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	// uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
	uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

	// Input terminal (Microphone input)
	if (entityID == 1) {
		switch (ctrlSel) {
		case AUDIO_TE_CTRL_CONNECTOR: {
			// The terminal connector control only has a get request with only the CUR attribute.
			audio_desc_channel_cluster_t ret;

			// Those are dummy values for now
			ret.bNrChannels = 1;
			ret.bmChannelConfig = (audio_channel_config_t) 0;
			ret.iChannelNames = 0;

			log_info("Get terminal connector");

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
	if (entityID == 2) {
		switch (ctrlSel) {
		case AUDIO_FU_CTRL_MUTE:
			// Audio control mute cur parameter block consists of only one byte - we thus can send it right away
			// There does not exist a range parameter block for mute
			log_info("Get Mute of channel: %u", channelNum);
			return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);

		case AUDIO_FU_CTRL_VOLUME:
			switch (p_request->bRequest) {
			case AUDIO_CS_REQ_CUR:
				log_info("Get Volume of channel: %u", channelNum);
				return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));

			case AUDIO_CS_REQ_RANGE:
				log_info("Get Volume range of channel: %u", channelNum);

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
	if (entityID == 4) {
		switch (ctrlSel) {
		case AUDIO_CS_CTRL_SAM_FREQ:
			// channelNum is always zero in this case
			switch (p_request->bRequest) {
			case AUDIO_CS_REQ_CUR:
				log_info("Get Sample Freq.");
				return tud_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));

			case AUDIO_CS_REQ_RANGE:
				log_info("Get Sample Freq. range");
				return tud_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));

				// Unknown/Unsupported control
			default:
				TU_BREAKPOINT();
				return false;
			}
			break;

		case AUDIO_CS_CTRL_CLK_VALID:
			// Only cur attribute exists for this request
			log_info("Get Sample Freq. valid");
			return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

			// Unknown/Unsupported control
		default:
			TU_BREAKPOINT();
			return false;
		}
	}

	log_warn("Unsupported entity: %d", entityID);
	return false; 	// Yet not implemented
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting) {
	(void) rhport;
	(void) itf;
	(void) ep_in;
	(void) cur_alt_setting;



	for (uint8_t fifo = 0; fifo < CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO; fifo++) {
		tud_audio_write_support_ff(
				fifo,
				audio_buffer[fifo],
				CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_CHANNEL_PER_FIFO_TX * (AUDIO_SAMPLE_RATE / 1000)
				);
	}

	return true;
}

int16_t ch[4]={0, 1000, 2000, 3000};

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in,
		uint8_t cur_alt_setting) {
	(void) rhport;
	(void) n_bytes_copied;
	(void) itf;
	(void) ep_in;
	(void) cur_alt_setting;

	//der I2C lockt sich den mutex, sobald ein neuer sensorwert eintrifft. Er kann dann "entspannt" sowohl den sensorbuffer als auch den filterbuffer aktualisieren
	//er inkrementiert den offset und achtet darauf, nicht über die grenzen des buffers hinaus zu schreiben


	//Mutex, hole die Daten, mit denen wir hier arbeiten und schalte den data_doublebuffer_index um und setze den data_offset auf 0 zurück
	size_t my_data_offset{0};
	size_t my_data_doublebuffer_index{0};

	assert(osMutexAcquire(mtxBufferHandle, 100)==osOK);
	my_data_offset=data_offset;
	data_offset=0;
	my_data_doublebuffer_index=data_doublebuffer_index;
	data_doublebuffer_index=data_doublebuffer_index==1?0:1;
	assert(osMutexRelease(mtxBufferHandle) == osOK);

	//wir halten es in dieser ersten Version wirklich sehr einfach, weil die Abtastfrequenz maximal 1kHz ist und damit der Abfragefrequenz via USB entspricht.
	//Entweder gibt es keinen oder einen neuen Sensorwert.
	//Sobald es einen neuen Wert gibt, merken wir uns den
	//den zuletzt gemerkten neuen Wert schreiben wir in den kompletten buffer
	//Das Resampling können wir uns sparen!
	if(my_data_offset>0){
		//Falls es überhaupt neue Sensor-Daten gab: Daten sichern
		lastValidSensorData=sensor_data_buffer[my_data_doublebuffer_index][my_data_offset-1];
		lastValidFilterData=filter_data_buffer[my_data_doublebuffer_index][my_data_offset-1];
		//printf("%5d, %5d\r\n", lastValidSensorData, lastValidFilterData);

	}
/*
	for(int i=0;i<(AUDIO_SAMPLE_RATE / 1000);i++){
		audio_buffer[0][2*i]   = ch[0]++;
		audio_buffer[0][2*i+1] = ch[1]++;
		audio_buffer[1][2*i]   = ch[2]++;
		audio_buffer[1][2*i+1] = ch[3]++;
	}
	return true;
*/

	for (uint32_t i = 0; i < (AUDIO_SAMPLE_RATE / 1000); i++) {
		audio_buffer[0][2 * i] 		= lastValidSensorData;
		audio_buffer[0][2 * i + 1]  = lastValidFilterData;
		audio_buffer[1][2 * i] 		= lastValidSensorData;
		audio_buffer[1][2 * i + 1] 	= lastValidFilterData;
	}
	return true;


}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
	(void) rhport;
	(void) p_request;

	return true;
}

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

#include "fdacoefs.h"
#include "cmsis_os.h"

FIR::Filter<BL> *filter;
MPU6050::MPU6050 *mpu6050;

extern "C" void taskI2C(void *attributes) {
	//query i2c bus

	log_info("Query I2C bus");
	int n = 0;
	for (int i = 0; i < 256; i += 2) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 200) != HAL_OK)
			continue;
		log_info("Device found at 8bit address %d", i);
		n++;
	}
	log_info("Found %d devices", n);

	filter = new FIR::Filter<BL>(B);
	mpu6050 = new MPU6050::MPU6050(&hi2c1);
	assert(mpu6050->Setup());

	//char txBuf[32];
	while (true) {
		mpu6050->UpdateAccel();
		int16_t Ax = mpu6050->GetAx();
		int16_t Ax_filtered = filter->Update(Ax);
		assert(osMutexAcquire(mtxBufferHandle, 100)==osOK);
		if(data_offset<SENSOR_DATA_SIZE){
			sensor_data_buffer[data_doublebuffer_index][data_offset]=Ax;
			filter_data_buffer[data_doublebuffer_index][data_offset]=Ax_filtered;
			data_offset++;
		}
		assert(osMutexRelease(mtxBufferHandle) == osOK);



		//printf("Ax=%.2f, Ay=%.2f, Az=%.2f, Gx=%.2f, Gy=%.2f, Gz=%.2f \r\n", Ax, Ay, Az, Gx, Gy, Gz);
		//printf("%.4f, %.4f\r\n", Ax, Ax_filtered);
		//sprintf(txBuf, "%.4f, %.4f\r\n", Ax, Ax_filtered);
		//CDC_Transmit_FS((uint8_t*) txBuf, strlen(txBuf));
		osDelay(200);
	}

}

extern "C" void taskUsb(void *attributes) {

	log_info("Init USB");
	board_vbus_sense_init();
	assert(tusb_init());

	// Init values
	sampFreq = AUDIO_SAMPLE_RATE;
	clkValid = 1;

	sampleFreqRng.wNumSubRanges = 1;
	sampleFreqRng.subrange[0].bMin = AUDIO_SAMPLE_RATE;
	sampleFreqRng.subrange[0].bMax = AUDIO_SAMPLE_RATE;
	sampleFreqRng.subrange[0].bRes = 0;

	while (1) {
		tud_task(); // tinyusb device task
		led_blinking_task();
		osDelay(1);
	}
}

