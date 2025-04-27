

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "USB.h"
#include "USB_CDC.h"
#include <stdio.h>
#include <FreeRtos.h>
#include <task.h>
#include <queue.h>
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"
#include "ssd1306_port.h"
#include "ssd1306.h"
#include "cy_rtc.h"
#include "rtc_commands.h"
/*******************************************************************************
 * Macros
 ********************************************************************************/
#define USB_CONFIG_DELAY          (10U)       /* In milliseconds */
#define ADC_CHANNEL               (0U)        /* SAR ADC channel 0 */
#define	COMMAND                   "start"

#define SAMPLE_COUNT     		  (50U)       /* Number of samples to send */
#define SAMPLE_INTERVAL           (10U)       /* Sampling interval in milliseconds */

#define BUFFER_SIZE               (1024U)     /* Buffer size */
#define SAMPLE_INTERVAL_MS         10
#define NUM_CHANNELS               4          // Number of ADC channels

#define QUEUE_LENGTH               100        /* Number of items the queue can hold */
#define QUEUE_ITEM_SIZE BUFFER_SIZE

int sample_index = 0;
float adc_samples[SAMPLE_COUNT];

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void usb_add_cdc(void);

/*********************************************************************
 *       Information that are used during enumeration
 **********************************************************************/
static const USB_DEVICE_INFO usb_deviceInfo = {
		0x058B,                       /* VendorId    */
		0x027D,                       /* ProductId    */
		"Infineon Technologies",      /* VendorName   */
		"CDC Code Example",           /* ProductName  */
		"12345678"                    /* SerialNumber */
};

typedef struct {
	float voltages[NUM_CHANNELS];  // Array to store voltage for each channel
	int sample_index;              // Index of the sample
} adc_sample_t;

typedef struct {
	float voltages[NUM_CHANNELS];  // Array to store voltage for each channel
} oled_data_t;

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
static USB_CDC_HANDLE usb_cdcHandle;

static bool isSampling = false; 	// Flag to indicate if sampling should start

static bool start_command_received= false;
TaskHandle_t usb_task_handle;
TaskHandle_t adc_sampling_task_handle;
QueueHandle_t oled_queue;  			// Queue to send data to OLED task

// Declare the queue handle
QueueHandle_t adc_to_usb_queue;

//USB task for communication of ADC and  OLED
void usb_task(void *pvParameters)
{
	char received_buffer[BUFFER_SIZE];
	char command_buffer[64];  // Buffer to store incoming commands
	uint8_t commandIndex = 0;
	char data_from_queue[BUFFER_SIZE]; // Buffer to hold data from queue

	printf("usb task running \r\n");

	/* Start the USB stack */
	USBD_Start();

	/* Turning the LED on to indicate device is active */
	cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

	/* Wait for USB configuration */
	while ((USBD_GetState() & USB_STAT_CONFIGURED) != USB_STAT_CONFIGURED)
	{
		vTaskDelay(USB_CONFIG_DELAY);
	}

	for (;;) {
		// Try receiving data from USB
		int num_bytes_received = USBD_CDC_Receive(usb_cdcHandle, received_buffer, sizeof(received_buffer), 0);

		if (num_bytes_received > 0)
		{
			printf("Received %d bytes from USB\r\n", num_bytes_received);

			for (int i = 0; i < num_bytes_received; i++)
			{
				char read_data = received_buffer[i];

				if (read_data == '\n' || read_data == '\r')
				{
					command_buffer[commandIndex] = '\0';  // Null-terminate the command

					if (strcmp(command_buffer, "start") == 0)
					{
						start_command_received = true;  // Set flag to start sampling and sending data
						isSampling = true;
						printf("Command received: Start sending data\r\n");

					} 	else if (strcmp(command_buffer, "read") == 0)
						{

							printf("Command received: Display RTC time\r\n");
							char rtc_time_buffer[64];

							read_rtc_time(rtc_time_buffer, sizeof(rtc_time_buffer));  // Read RTC time
							vTaskDelay(pdMS_TO_TICKS(10));

							USBD_CDC_Write(usb_cdcHandle, rtc_time_buffer, strlen(rtc_time_buffer), 0);  // Send RTC time over USB
							USBD_CDC_WaitForTX(usb_cdcHandle, 0);  // Wait for transmission to complete

						// vTaskDelay(pdMS_TO_TICKS(10));;
					} else if (strncmp(command_buffer, "set",3) == 0)
					{
						printf("Command received: Set RTC time\r\n");
						char response_buffer[64];

						set_rtc_time(command_buffer, response_buffer, sizeof(response_buffer));  // Set RTC time
						vTaskDelay(pdMS_TO_TICKS(100));

						USBD_CDC_Write(usb_cdcHandle, response_buffer, strlen(response_buffer), 0);  // Send response over USB
						USBD_CDC_WaitForTX(usb_cdcHandle, 0);  // Wait for transmission to complete

						vTaskDelay(pdMS_TO_TICKS(1000));
					}
					commandIndex = 0;
				} else {
					if (commandIndex < sizeof(command_buffer) - 1) {

						command_buffer[commandIndex++] = read_data;
					}
				}
			}
		}

		// If the start command has been received, send data from the queue
		if (start_command_received)
		{
			while (xQueueReceive(adc_to_usb_queue, data_from_queue, pdMS_TO_TICKS(100)) == pdPASS)
			{
				USBD_CDC_Write(usb_cdcHandle, data_from_queue, BUFFER_SIZE, 0);
				USBD_CDC_WaitForTX(usb_cdcHandle, 0);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
void adc_sampling_task(void *pvParameters)
{
	uint16_t adc_result;
	float voltage[NUM_CHANNELS];
	char local_buffer[BUFFER_SIZE];  // Ensure buffer is large enough
	oled_data_t oled_data;
	printf("adc sampling task running \r\n");

	while (1) {
		if (isSampling)
		{
			for (int i = 0; i < SAMPLE_COUNT; i++)
			{
				// Clear the buffer before formatting
				memset(local_buffer, 0, sizeof(local_buffer));
				// Clear the buffer before formatting
				// memset(oled_data.voltageso, 0, sizeof(oled_data.voltageso));

				int len = snprintf(local_buffer, sizeof(local_buffer), "\r\n Sample %d: ", i + 1);
				if (len < 0 || len >= sizeof(local_buffer))
				{
					printf("Buffer overflow at sample start!\r\n");
					break;
				}

				for (int ch = 0; ch < NUM_CHANNELS; ch++)
				{

					adc_result = Cy_SAR_GetResult16(SAR, ch);
					voltage[ch] = Cy_SAR_CountsTo_Volts(SAR, ch, adc_result);
					oled_data.voltages[ch] = voltage[ch];

					// Add data to buffer, check for overflow
					len += snprintf(local_buffer + len, sizeof(local_buffer) - len, "ch%d: %.3fV ", ch + 1, voltage[ch]);

					if (len >= sizeof(local_buffer))
					{
						printf("Buffer overflow detected!\r\n");
						break;
					}
				}
				len += snprintf(local_buffer + len, sizeof(local_buffer) - len, "\r\n");

				// Send the data to USB queue
				if (xQueueSend(adc_to_usb_queue, local_buffer, pdMS_TO_TICKS(100)) == pdPASS)
				{
					printf("Sample %d sent to queue\r\n", i + 1);
				} else {
					printf("Failed to send sample %d to queue\r\n", i + 1);
				}
				// Only send the last sample to the OLED queue
				if (i == SAMPLE_COUNT - 1)
				{
					if (xQueueSend(oled_queue, &oled_data, pdMS_TO_TICKS(100)) == pdPASS)
					{
						printf("50th sample voltages sent to OLED task\r\n");
						for (int ch = 0; ch < NUM_CHANNELS; ch++)
						{
							printf("ch%d: %.3fV \r\n", ch , oled_data.voltages[ch]);
						}
						printf("\r\n");
					} else {
						printf("Failed to send 50th sample voltages to OLED task \r\n");
					}
				}
     			// Delay before taking the next sample
				vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
			}
			isSampling = false;  // Reset the sampling flag after completion
		}
	}
}
void display_task(void *pvParameters) {
	oled_data_t oled_data;

	while (1) {
		// Wait to receive voltage data from the ADC task
		if (xQueueReceive(oled_queue, &oled_data, portMAX_DELAY) == pdPASS)
		{
			char display_buffer[20];
			snprintf(display_buffer, sizeof(display_buffer),
					"ch1: %.3fV", oled_data.voltages[0]);
			printf("oled buffer : %s \r\n", display_buffer);
			ssd1306_SetCursor(10,2);
			ssd1306_UpdateScreen();
			vTaskDelay(pdMS_TO_TICKS(100));
			ssd1306_WriteString(display_buffer, Font_7x10, White);
			ssd1306_UpdateScreenp1();
			vTaskDelay(pdMS_TO_TICKS(100));

			char display_buffer1[20];
			snprintf(display_buffer1, sizeof(display_buffer1),
					"ch2: %.3fV", oled_data.voltages[1]);
			ssd1306_SetCursor(10,15);
			ssd1306_UpdateScreen();
			vTaskDelay(pdMS_TO_TICKS(100));
			ssd1306_WriteString(display_buffer1, Font_7x10, White);
			ssd1306_UpdateScreenp2();
			vTaskDelay(pdMS_TO_TICKS(100));

			char display_buffer2[20];
			snprintf(display_buffer2, sizeof(display_buffer2),
					"ch3: %.3fV", oled_data.voltages[2]);
			ssd1306_SetCursor(10,28);
			ssd1306_UpdateScreen();
			vTaskDelay(pdMS_TO_TICKS(100));
			ssd1306_WriteString(display_buffer2, Font_7x10, White);
			ssd1306_UpdateScreenp3();
			vTaskDelay(pdMS_TO_TICKS(100));

			char display_buffer3[20];
			snprintf(display_buffer3, sizeof(display_buffer3),
					"ch4: %.3fV", oled_data.voltages[3]);
			ssd1306_SetCursor(10,40);
			ssd1306_UpdateScreen();
			vTaskDelay(pdMS_TO_TICKS(100));
			ssd1306_WriteString(display_buffer3, Font_7x10, White);
			ssd1306_UpdateScreenp4();
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}

int main(void)
{
	cy_rslt_t result;
	cy_stc_scb_spi_context_t spiContext;

	/* Initialize the device and board peripherals */
	result = cybsp_init() ;

	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	/* Enable global interrupts */
	__enable_irq();

	/* Initialize retarget-io to use the debug UART port */
	result=cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}
	/* Create the queue */
	adc_to_usb_queue = xQueueCreate(QUEUE_LENGTH,BUFFER_SIZE );
	oled_queue = xQueueCreate(QUEUE_LENGTH, sizeof(oled_data_t));

	if (adc_to_usb_queue == NULL)
	{
		printf("Failed to create the queue!\r\n");
		CY_ASSERT(0);
	}


	/* Initialize the User LED */
	cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);


	//ADC pin
	Cy_GPIO_Pin_Init(ioss_0_port_10_pin_0_PORT, ioss_0_port_10_pin_0_PIN, &ioss_0_port_10_pin_0_config);
	Cy_GPIO_Pin_Init(ioss_0_port_10_pin_1_PORT, ioss_0_port_10_pin_1_PIN, &ioss_0_port_10_pin_1_config);
	Cy_GPIO_Pin_Init(ioss_0_port_10_pin_2_PORT, ioss_0_port_10_pin_2_PIN, &ioss_0_port_10_pin_2_config);
	Cy_GPIO_Pin_Init(ioss_0_port_10_pin_3_PORT, ioss_0_port_10_pin_3_PIN, &ioss_0_port_10_pin_3_config);
	Cy_GPIO_Pin_Init(ioss_0_port_6_pin_2_PORT, ioss_0_port_6_pin_2_PIN, &ioss_0_port_6_pin_2_config); //RS
	Cy_GPIO_Pin_Init(ioss_0_port_8_pin_0_PORT, ioss_0_port_8_pin_0_PIN, &ioss_0_port_8_pin_0_config); //DC

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	printf("\x1b[2J\x1b[;H");
	printf("****************** "
			"emUSB Device: send data to usb "
			"****************** \r\n\n");
	//Init ADC
	Cy_SysAnalog_Init(&pass_0_aref_0_config);

	// Initialize AREF
	Cy_SysAnalog_Enable();

	cy_en_sar_status_t status;
	status = Cy_SAR_Init(SAR, &pass_0_sar_0_config);
	if (CY_SAR_SUCCESS == status)
	{
		/* Turn on the SAR hardware. */
		Cy_SAR_Enable(SAR);
	}

	Cy_GPIO_Write(ioss_0_port_9_pin_3_PORT, ioss_0_port_9_pin_3_PIN,0);

	result = Cy_SCB_SPI_Init(SCB2, &scb_2_config, &spiContext);
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}
	Cy_SCB_SPI_Enable(SCB2);

	ssd1306_Init();

	Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS);

	/* Initializes the USB stack */
	USBD_Init();

	/* Endpoint Initialization for CDC class */
	usb_add_cdc();

	/* Set device info used in enumeration */
	USBD_SetDeviceInfo(&usb_deviceInfo);

	Cy_RTC_Init(&srss_0_rtc_0_config);

	// Create FreeRTOS tasks
	xTaskCreate(usb_task, "usb_task", 2048, NULL, 6, NULL);
	xTaskCreate(adc_sampling_task, "adc_sampling_task", 1024, NULL, 1, NULL);
	xTaskCreate(display_task, "OLED Task", 1024, NULL, 1, NULL);

	//start scheduler
	vTaskStartScheduler();
}

/*********************************************************************
 * Function Name: USBD_CDC_Echo_Init
 **********************************************************************
 * Summary:
 *  Add communication device class to USB stack
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 **********************************************************************/

void usb_add_cdc(void) {

	static U8             OutBuffer[USB_FS_BULK_MAX_PACKET_SIZE];
	USB_CDC_INIT_DATA     InitData;
	USB_ADD_EP_INFO       EPBulkIn;
	USB_ADD_EP_INFO       EPBulkOut;
	USB_ADD_EP_INFO       EPIntIn;

	memset(&InitData, 0, sizeof(InitData));
	EPBulkIn.Flags          = 0;                             /* Flags not used */
	EPBulkIn.InDir          = USB_DIR_IN;                    /* IN direction (Device to Host) */
	EPBulkIn.Interval       = 0;                             /* Interval not used for Bulk endpoints */
	EPBulkIn.MaxPacketSize  = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
	EPBulkIn.TransferType   = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
	InitData.EPIn  = USBD_AddEPEx(&EPBulkIn, NULL, 0);

	EPBulkOut.Flags         = 0;                             /* Flags not used */
	EPBulkOut.InDir         = USB_DIR_OUT;                   /* OUT direction (Host to Device) */
	EPBulkOut.Interval      = 0;                             /* Interval not used for Bulk endpoints */
	EPBulkOut.MaxPacketSize = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
	EPBulkOut.TransferType  = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
	InitData.EPOut = USBD_AddEPEx(&EPBulkOut, OutBuffer, sizeof(OutBuffer));

	EPIntIn.Flags           = 0;                             /* Flags not used */
	EPIntIn.InDir           = USB_DIR_IN;                    /* IN direction (Device to Host) */
	EPIntIn.Interval        = 64;                            /* Interval of 8 ms (64 * 125us) */
	EPIntIn.MaxPacketSize   = USB_FS_INT_MAX_PACKET_SIZE ;   /* Maximum packet size (64 for Interrupt) */
	EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;         /* Endpoint type - Interrupt */
	InitData.EPInt = USBD_AddEPEx(&EPIntIn, NULL, 0);

	usb_cdcHandle = USBD_CDC_Add(&InitData);
}

/* [] END OF FILE */
