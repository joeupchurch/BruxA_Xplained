/*	BruxA_revA1 - based on code from BruxA_revA and build from new GCC project
	for ATSAMD21E18A board. */

#include <asf.h>
#include "console_serial.h"
#include "ble_manager.h"
#include "conf_at25dfx.h"

struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;
struct adc_module adc_instance;
struct tc_module tc_instance;

volatile at_ble_status_t ble_status;
volatile bool timer_flag = false;
volatile bool connected_flag = false;
volatile bool adc_read_done = false;
volatile bool streaming = false;

/* Services handlers */
at_ble_handle_t bruxa_service_handler;
at_ble_characteristic_t bruxa_service_characs[10];

#define BRUXA_SERVICE_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xc0,0xba,0x5a,0xf0

#define PRESSURE_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xd8,0xba,0x5a,0xf0
#define TIMESTAMP_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xe1,0xba,0x5a,0xf0
#define RECORD_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xf1,0xba,0x5a,0xf0
#define SYNC_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xf2,0xba,0x5a,0xf0
#define STOP_SYNC_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xf3,0xba,0x5a,0xf0
#define PRESSURE_READ_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xf4,0xba,0x5a,0xf0
#define BATTERY_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xf5,0xba,0x5a,0xf0
#define BATTERY_READ_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xf6,0xba,0x5a,0xf0
#define STREAMING_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xf7,0xba,0x5a,0xf0
#define STREAM_DATA_CHAR_UUID 0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xa6,0x87,\
0xe5,0x11,0x36,0x39,0xf8,0xba,0x5a,0xf0

#define AT25DFX_BUFFER_SIZE		(182)
#define TIMESTAMP_DATA_SIZE		(4)
#define AT25DFX_SIZE			(1000000)
#define ADC_SAMPLES				1
#define CONF_TC_MODULE			TC3

uint8_t adc_sensor_buffer[ADC_SAMPLES];
volatile uint16_t last_page = 0;
volatile uint16_t current_page = 0;
uint8_t current_idx = 0;
uint32_t timestamp_buffer[1];
uint32_t temp_timestamp_buffer[1] = {1536342757};
uint8_t recording_value[1];
uint8_t sync_stop_value[1];
uint8_t sync_value[1];
uint32_t memory_address = 0; // Flash start address
uint8_t write_buffer_counter = 0;
uint8_t battery_percent = 100;

///* Buffer used to read in this example */
static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
/* Buffer used to write ADC data to */
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE];


void stream_data(void){

}

void send_battery_data(void)
{
	uint16_t current_battery;
	adc_set_positive_input(&adc_instance,BATTERY_PIN);
	adc_flush(&adc_instance);
	adc_start_conversion(&adc_instance);
	
	do{
		/* Wait for conversion to be done and read out result */
	} while(adc_read(&adc_instance, &current_battery) == STATUS_BUSY);
	
	battery_percent = (current_battery*220)/51-600; // Battery in percent where 127=3.7V=0 percent and 162 = 2.1/3.3=4.2V=100
	DBG_LOG("Read Battery :: %d",current_battery);
	
	DBG_LOG("Changed Battery :: %d%%", battery_percent);
	ble_status = at_ble_characteristic_value_set(bruxa_service_characs[6].char_val_handle, &battery_percent, sizeof(uint8_t));
	ble_status = at_ble_notification_send(0,bruxa_service_characs[6].char_val_handle);
}

void tc_callback_sensor(struct tc_module *const module_inst)
{
	//adc_set_positive_input(&adc_instance_sensor,ADC_POSITIVE_INPUT_PIN0);
	//adc_read_buffer_job(&adc_instance_sensor, adc_sensor_buffer, ADC_SAMPLES);
	//port_pin_toggle_output_level(LED0_PIN);

	uint8_t result;
	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN0);
	adc_set_gain(&adc_instance,ADC_GAIN_FACTOR_1X);
	adc_flush(&adc_instance);
	adc_start_conversion(&adc_instance);
	do{
		/* Wait for conversion to be done and read out result */
	} while(adc_read(&adc_instance, &result) == STATUS_BUSY);
	//DBG_LOG("Sensor :: %d",result);
	
	/* Pulled from ADC callback */
	write_buffer[write_buffer_counter] = result;
	DBG_LOG("%d,%d",write_buffer[write_buffer_counter],write_buffer_counter);
	
	/*Increment buffer counter */
	write_buffer_counter += 1;
	
	/* Write buffer not yet filled up */
	if (write_buffer_counter==AT25DFX_BUFFER_SIZE) {
		
		/* Then write the 256 byte page of ADC data */
		at25dfx_chip_write_buffer(&at25dfx_chip, memory_address, write_buffer, AT25DFX_BUFFER_SIZE);
		memory_address += AT25DFX_BUFFER_SIZE;
		write_buffer_counter=0;
		
		port_pin_toggle_output_level(LED0_PIN);
		DBG_LOG("Page Saved!");
	}
}

void configure_tc_sensor(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size = TC_COUNTER_SIZE_8BIT;
	config_tc.clock_source = GCLK_GENERATOR_3;
	config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV64;
	config_tc.counter_8_bit.period = 50; // 32000/64/10hz
	tc_init(&tc_instance, CONF_TC_MODULE, &config_tc);
	tc_enable(&tc_instance);
	tc_stop_counter(&tc_instance);
}

void configure_tc_callbacks_sensor(void)
{
	tc_register_callback(&tc_instance, tc_callback_sensor,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance, TC_CALLBACK_OVERFLOW);
}

void configure_adc(void)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);

	config_adc.clock_source	   = GCLK_GENERATOR_1;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
	config_adc.reference       = ADC_REFERENCE_INTVCC1;
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN0;
	config_adc.gain_factor     = ADC_GAIN_FACTOR_1X;//ADC_GAIN_FACTOR_DIV2;
	config_adc.resolution      = ADC_RESOLUTION_8BIT;
	config_adc.pin_scan.offset_start_scan = 0;
	config_adc.pin_scan.inputs_to_scan = ADC_SAMPLES;

	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}

static void at25dfx_init(void)
{
	struct at25dfx_chip_config at25dfx_chip_config;
	struct spi_config at25dfx_spi_config;
	at25dfx_spi_get_config_defaults(&at25dfx_spi_config);
	at25dfx_spi_config.mode_specific.master.baudrate = AT25DFX_CLOCK_SPEED;
	at25dfx_spi_config.mux_setting = AT25DFX_SPI_PINMUX_SETTING;
	at25dfx_spi_config.pinmux_pad0 = AT25DFX_SPI_PINMUX_PAD0;
	at25dfx_spi_config.pinmux_pad1 = AT25DFX_SPI_PINMUX_PAD1;
	at25dfx_spi_config.pinmux_pad2 = AT25DFX_SPI_PINMUX_PAD2;
	at25dfx_spi_config.pinmux_pad3 = AT25DFX_SPI_PINMUX_PAD3;
	spi_init(&at25dfx_spi, AT25DFX_SPI, &at25dfx_spi_config);
	spi_enable(&at25dfx_spi);
	
	at25dfx_chip_config.type = AT25DFX_MEM_TYPE;
	at25dfx_chip_config.cs_pin = AT25DFX_CS;
	at25dfx_chip_init(&at25dfx_chip, &at25dfx_spi, &at25dfx_chip_config);
}

/** Configure LED0, turn it off*/
static void config_led(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED0_PIN, &pin_conf);
	port_pin_set_output_level(LED0_PIN, LED0_INACTIVE);
}

static uint8_t adv_data[] = {
	0x06,// AD2 Length = 8 (AD_TYPE + AD)
	0x09,// AD2 Type = Complete local Name
	'B','r','u','x','A' // AD2 = “BruxA”
};

void start_advertisement (void){
	DBG_LOG_DEV("Assignment 2.1: Start Advertisement");
	
	/*Set advertisement data*/
	ble_status = at_ble_adv_data_set(adv_data,sizeof(adv_data),NULL,0);
	DBG_LOG("Start advertising");
	
	/*Start Advertisement*/
	ble_status = at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED,\
	AT_BLE_ADV_GEN_DISCOVERABLE,NULL,AT_BLE_ADV_FP_ANY,1600,655,0);
	if(ble_status != AT_BLE_SUCCESS){
		DBG_LOG("*** Failed to start advertisement");
	}
}

/* Callback registered for AT_BLE_CONNECTED event*/
static at_ble_status_t ble_connected_cb (void *param)
{
	at_ble_connected_t *connected = (at_ble_connected_t *)param;
	DBG_LOG("Application connected ");
	connected_flag = true;
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}
/* Callback registered for AT_BLE_DISCONNECTED event */
static at_ble_status_t ble_disconnected_cb (void *param)
{
	DBG_LOG("Application disconnected ");
	connected_flag = false;
	start_advertisement();
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

static const ble_gap_event_cb_t app_gap_handle = {
	.connected = ble_connected_cb, // AT_BLE_CONNECTED
	.disconnected = ble_disconnected_cb, // AT_BLE_DISCONNECTED
};

/* Register GAP callbacks at BLE manager level*/
void register_btlc1000_callbacks(void){
	/* Register GAP Callbacks */
	DBG_LOG_DEV("Assignment 2.2: Register ATBTLC1000 callbacks");
	ble_status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,\
	BLE_GAP_EVENT_TYPE,&app_gap_handle);
	if (ble_status != true){
		DBG_LOG("##Error when Registering ATBTLC1000 callbacks");
	}
}

void bruxa_service_init(void) {
	
	at_ble_uuid_t bruxa_service_uuid;
	
	uint8_t serv_uuid[] = {BRUXA_SERVICE_UUID};
	uint8_t charac0_uuid[] = {PRESSURE_CHAR_UUID};
	uint8_t charac1_uuid[] = {TIMESTAMP_CHAR_UUID};
	uint8_t charac2_uuid[] = {RECORD_CHAR_UUID};
	uint8_t charac3_uuid[] = {SYNC_CHAR_UUID};
	uint8_t charac4_uuid[] = {STOP_SYNC_CHAR_UUID};
	uint8_t charac5_uuid[] = {PRESSURE_READ_CHAR_UUID};
	uint8_t charac6_uuid[] = {BATTERY_CHAR_UUID};
	uint8_t charac7_uuid[] = {BATTERY_READ_CHAR_UUID};
	uint8_t charac8_uuid[] = {STREAMING_CHAR_UUID};
	uint8_t charac9_uuid[] = {STREAM_DATA_CHAR_UUID};
	
	/* Set service UUID */
	bruxa_service_uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_uuid.uuid ,serv_uuid,16);
	
	/* Define pressure characteristic */
	bruxa_service_characs[0].user_desc = (uint8_t *)"Pressure";
	bruxa_service_characs[0].user_desc_len = 8;
	bruxa_service_characs[0].user_desc_max_len = 8;
	bruxa_service_characs[0].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[0].uuid.uuid,charac0_uuid,16);
	bruxa_service_characs[0].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_WRITE;
	bruxa_service_characs[0].value_max_len = AT25DFX_BUFFER_SIZE;
	bruxa_service_characs[0].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	//bruxa_service_characs[0].presentation_format.format = AT_BLE_PRES_FORMAT_UINT8;
	
	/* Define timestamp characteristic */
	bruxa_service_characs[1].user_desc = (uint8_t *)"Timestamp";
	bruxa_service_characs[1].user_desc_len = 9;
	bruxa_service_characs[1].user_desc_max_len = 9;
	bruxa_service_characs[1].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[1].uuid.uuid,charac1_uuid,16);
	bruxa_service_characs[1].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_WRITE;;
	bruxa_service_characs[1].value_max_len = sizeof(uint32_t);
	bruxa_service_characs[1].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	
	/* Define record characteristic */
	bruxa_service_characs[2].user_desc = (uint8_t *)"Record";
	bruxa_service_characs[2].user_desc_len = 6;
	bruxa_service_characs[2].user_desc_max_len = 6;
	bruxa_service_characs[2].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[2].uuid.uuid,charac2_uuid,16);
	bruxa_service_characs[2].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_WRITE;
	bruxa_service_characs[2].value_max_len = sizeof(uint8_t);
	bruxa_service_characs[2].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	
	/* Define sync characteristic */
	bruxa_service_characs[3].user_desc = (uint8_t *)"Sync";
	bruxa_service_characs[3].user_desc_len = 4;
	bruxa_service_characs[3].user_desc_max_len = 4;
	bruxa_service_characs[3].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[3].uuid.uuid,charac3_uuid,16);
	bruxa_service_characs[3].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_WRITE;
	bruxa_service_characs[3].value_max_len = sizeof(uint8_t);
	bruxa_service_characs[3].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	
	/* Define stop sync characteristic */
	bruxa_service_characs[4].user_desc = (uint8_t *)"Stop Sync";
	bruxa_service_characs[4].user_desc_len = 9;
	bruxa_service_characs[4].user_desc_max_len = 9;
	bruxa_service_characs[4].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[4].uuid.uuid,charac4_uuid,16);
	bruxa_service_characs[4].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_WRITE;
	bruxa_service_characs[4].value_max_len = sizeof(uint8_t);
	bruxa_service_characs[4].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	
	/* Define pressure read characteristic */
	bruxa_service_characs[5].user_desc = (uint8_t *)"Pressure Read";
	bruxa_service_characs[5].user_desc_len = 13;
	bruxa_service_characs[5].user_desc_max_len = 13;
	bruxa_service_characs[5].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[5].uuid.uuid,charac5_uuid,16);
	bruxa_service_characs[5].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_WRITE;
	bruxa_service_characs[5].value_max_len = sizeof(uint8_t);
	bruxa_service_characs[5].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	
	/* Define battery characteristic */
	bruxa_service_characs[6].user_desc = (uint8_t *)"Battery";
	bruxa_service_characs[6].user_desc_len = 7;
	bruxa_service_characs[6].user_desc_max_len = 7;
	bruxa_service_characs[6].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[6].uuid.uuid,charac6_uuid,16);
	bruxa_service_characs[6].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_WRITE;
	bruxa_service_characs[6].value_max_len = sizeof(uint8_t);
	bruxa_service_characs[6].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR);
	
	/* Define battery characteristic */
	bruxa_service_characs[7].user_desc = (uint8_t *)"Battery Read";
	bruxa_service_characs[7].user_desc_len = 12;
	bruxa_service_characs[7].user_desc_max_len = 12;
	bruxa_service_characs[7].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[7].uuid.uuid,charac7_uuid,16);
	bruxa_service_characs[7].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_WRITE;
	bruxa_service_characs[7].value_max_len = sizeof(uint8_t);
	bruxa_service_characs[7].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	
	/* Define streaming characteristic */
	bruxa_service_characs[8].user_desc = (uint8_t *)"Streaming";
	bruxa_service_characs[8].user_desc_len = 9;
	bruxa_service_characs[8].user_desc_max_len = 9;
	bruxa_service_characs[8].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[8].uuid.uuid,charac8_uuid,16);
	bruxa_service_characs[8].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_WRITE;
	bruxa_service_characs[8].value_max_len = sizeof(uint8_t);
	bruxa_service_characs[8].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);
	
	/* Define streaming characteristic */
	bruxa_service_characs[9].user_desc = (uint8_t *)"Stream Data";
	bruxa_service_characs[9].user_desc_len = 11;
	bruxa_service_characs[9].user_desc_max_len = 11;
	bruxa_service_characs[9].uuid.type = AT_BLE_UUID_128;
	memcpy(bruxa_service_characs[9].uuid.uuid,charac9_uuid,16);
	bruxa_service_characs[9].properties = AT_BLE_CHAR_READ | AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_WRITE;
	bruxa_service_characs[9].value_max_len = sizeof(uint8_t);
	bruxa_service_characs[9].value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR);
	
	/* Service configuration in ATBTLC1000 */
	if(at_ble_primary_service_define(&bruxa_service_uuid,\
	&bruxa_service_handler,NULL,0,\
	bruxa_service_characs,10)!=AT_BLE_SUCCESS){
		DBG_LOG("Failed to Initialize custom environment service");
	}
	else{
		DBG_LOG_DEV("Initialize custom environment service");
	}
}

/* Check if all values in array are the same 255 */
bool check(){
	int n = AT25DFX_BUFFER_SIZE;
	while(--n>0 && read_buffer[n]==255);
	return n==0;
}

/* Determine how many pages of data are saved on device */
uint16_t count_pages_saved(){
	uint16_t i;
	
	for (i=0; i<AT25DFX_SIZE/AT25DFX_BUFFER_SIZE; i++){
		at25dfx_chip_read_buffer(&at25dfx_chip, TIMESTAMP_DATA_SIZE+AT25DFX_BUFFER_SIZE*i,
		read_buffer, AT25DFX_BUFFER_SIZE);
		
		if (check(read_buffer, AT25DFX_BUFFER_SIZE)){
			last_page = i;
			DBG_LOG("Pages of data :: %d",last_page);
			break;
		}
	}
}

/* Update the timestamp characteristic with the saved value */
static void update_timestamp_char(){
	/* Write timestamp to timestamp characteristic */
	while(at25dfx_chip_read_buffer(&at25dfx_chip, 0, timestamp_buffer, TIMESTAMP_DATA_SIZE) != STATUS_OK);
	while(at_ble_characteristic_value_set(bruxa_service_characs[1].char_val_handle,
	(uint32_t *)&timestamp_buffer[0], sizeof(uint32_t)) != AT_BLE_SUCCESS);
}

/* Read next page of pressure write to pressure characteristic */
void send_pressure_data(void){
	
	if (current_page<last_page){

		at25dfx_chip_read_buffer(&at25dfx_chip, TIMESTAMP_DATA_SIZE+AT25DFX_BUFFER_SIZE*(current_page), read_buffer, AT25DFX_BUFFER_SIZE);
		DBG_LOG("Reading data...");
		/* Update attribute data base */
		ble_status = at_ble_characteristic_value_set(bruxa_service_characs[0].char_val_handle, (uint8_t *)&read_buffer[0], AT25DFX_BUFFER_SIZE);
		ble_status = at_ble_notification_send(0,bruxa_service_characs[0].char_val_handle);
		
		current_page++;

		if(ble_status != AT_BLE_SUCCESS){
			DBG_LOG("fail to send update notification ");
		}
	}
	else{
		/* Change sync value to 0 to signify end of data on device */
		sync_stop_value[0] = 0;
		ble_status = at_ble_characteristic_value_set(bruxa_service_characs[4].char_val_handle, (uint8_t *)&sync_stop_value[0], sizeof(uint8_t));
		ble_status = at_ble_notification_send(0,bruxa_service_characs[4].char_val_handle);
		if(ble_status != AT_BLE_SUCCESS){
			DBG_LOG("Fail to signify end of data on sync characteristic ");
		}
		DBG_LOG("End of data");
	}
}

static void begin_data_recording(void){
	uint16_t length;
	
	DBG_LOG("Starting to record");
	
	/* Unprotect the chip */
	while(at25dfx_chip_set_global_sector_protect(&at25dfx_chip,false)!=STATUS_OK);
	
	/* TEMP - Update timestamp char - should be handled by app at same time as starting recording*/
	update_timestamp_char();
	
	/* Erase the chip */
	while(at25dfx_chip_erase(&at25dfx_chip)!=STATUS_OK);
	
	/* Get the current timestamp from timestamp characteristic */
	at_ble_characteristic_value_get(bruxa_service_characs[1].char_val_handle, timestamp_buffer, length);
	DBG_LOG("Timestamp :: %d",timestamp_buffer[0]);
	
	/* Reset memory_address */
	memory_address = 0;
	
	///* Write the temp timestamp to the first flash values */
	//while(at25dfx_chip_write_buffer(&at25dfx_chip, memory_address, temp_timestamp_buffer, TIMESTAMP_DATA_SIZE) != STATUS_OK);
	//DBG_LOG("Timestamp written to flash");
	//
	//update_timestamp_char();
	
	/* Write the current timestamp to the first flash values */
	at25dfx_chip_write_buffer(&at25dfx_chip, memory_address, timestamp_buffer, TIMESTAMP_DATA_SIZE);
	DBG_LOG("Timestamp written to flash");
	
	/* Increment memory address */
	memory_address += TIMESTAMP_DATA_SIZE;
	
	/* Reset write buffer counter */
	write_buffer_counter = 0;
	
	/* Start timer to record */
	tc_set_count_value(&tc_instance,0);
	tc_start_counter(&tc_instance);
}

static void stop_data_recording(void){
	DBG_LOG("Stopping recording");
	
	/* Stop recording timer */
	tc_stop_counter(&tc_instance);
	
	/* Protect the chip */
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip,true);
}

/* Callback registered for char changed event*/
static at_ble_status_t bruxa_char_changed_event (void *param)
{
	uint16_t length;
	at_ble_characteristic_changed_t *char_changed_param_handle = (at_ble_characteristic_changed_t *)param;
	
	/* Handle changes to record characteristic */
	if (bruxa_service_characs[2].char_val_handle == char_changed_param_handle->char_handle){
		at_ble_characteristic_value_get(bruxa_service_characs[2].char_val_handle, recording_value, length);
		DBG_LOG("Recording value :: %d",recording_value[0]);
		if (recording_value[0]==49){
			// Start recording
			begin_data_recording();
		}
		else {
			// Stop recording
			stop_data_recording();
		}
		if (ble_status != AT_BLE_SUCCESS){
			DBG_LOG("Fail to read recording value");
		}
	}
	
	/* Handle changes to sync characteristic */
	if (bruxa_service_characs[3].char_val_handle == char_changed_param_handle->char_handle){
		at_ble_characteristic_value_get(bruxa_service_characs[3].char_val_handle, sync_value, length);
		DBG_LOG("Sync Value :: %d",sync_value[0]);
		if (sync_value[0]==49){
			// Begin sync
			DBG_LOG("Starting to sync");
			
			/* First determine number of saved pages */
			count_pages_saved();
			
			/* Update timestamp characteristic from saved values */
			update_timestamp_char();
			
			/* Reset current page number */
			current_page = 0;
			
			/* Then begin update pressure characteristic for first page and until all pages are read */
			send_pressure_data();
			
		}
		else {
			// Stop recording
			DBG_LOG("Stopping sync");

		}
		if (ble_status != AT_BLE_SUCCESS){
			DBG_LOG("Fail to read sync value");
		}
	}
	
	/* Handle changes to pressure read characteristic */
	if ((bruxa_service_characs[5].char_val_handle == char_changed_param_handle->char_handle)){
		send_pressure_data();
	}
	
	/* Handle changes to battery read characteristic */
	if ((bruxa_service_characs[7].char_val_handle == char_changed_param_handle->char_handle)){
		send_battery_data();
	}
	
	/* Handle changes to battery read characteristic */
	if ((bruxa_service_characs[8].char_val_handle == char_changed_param_handle->char_handle)){
		if (!streaming){
			streaming = true;
		}
		else {
			streaming = false;
		}
	}
	
	return AT_BLE_SUCCESS;
}

static const ble_gatt_server_event_cb_t app_gatt_server_handle = {
	.characteristic_changed = bruxa_char_changed_event
};

/* Register GATT callbacks at BLE manager level*/
void register_gatt_server_callbacks(void){
	/* Register GAP Callbacks */
	DBG_LOG_DEV("*** Assignment 4.1: Register GATT Server callbacks");
	ble_status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,\
	BLE_GATT_SERVER_EVENT_TYPE,&app_gatt_server_handle);
	if (ble_status != true){
		DBG_LOG("##Error when Registering ATBTLC1000 callbacks");
}	
}

int main (void)
{
	system_init();
	
	//system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
	//system_sleep();

	/* Initialize the AT25DFX chip . */
	at25dfx_init();
	
	/* Wake the AT25DFX chip . */
	at25dfx_chip_wake(&at25dfx_chip);
	//at25dfx_chip_sleep(&at25dfx_chip);
	
	/* Initialize serial console */
	serial_console_init();
	
	config_led();
	
	delay_init();
	
	configure_adc();
	
	configure_tc_sensor();
	configure_tc_callbacks_sensor();
	
	system_interrupt_enable_global();
	
	/* Check for flash memory */
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		// Handle missing or non-responsive device
		//DBG_LOG("No AT25DFX Chip Found!");
		port_pin_set_output_level(LED0_PIN,LED0_ACTIVE);
		delay_s(2);
		port_pin_set_output_level(LED0_PIN,LED0_INACTIVE);
	}
	else{
		port_pin_set_output_level(LED0_PIN,LED0_INACTIVE);
	}

	/* initialize the BLE chip  and Set the Device Address */
	ble_device_init(NULL);
	
	/* Register ATBTLC1000 Gap Callbacks */
	register_btlc1000_callbacks();
	
	/* Custom service init */
	bruxa_service_init();
	
	/* Register Gatt server callbacks */
	register_gatt_server_callbacks();
	
	/* Start Advertising process */
	start_advertisement();
	
	while (true){
		/* BLE Event Task */
		ble_event_task();
		//delay_ms(100);
	
		if (streaming){
			uint8_t result;
			adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN0);
			adc_set_gain(&adc_instance,ADC_GAIN_FACTOR_1X);
			adc_flush(&adc_instance);
			adc_start_conversion(&adc_instance);
			do{
				/* Wait for conversion to be done and read out result */
			} while(adc_read(&adc_instance, &result) == STATUS_BUSY);
			DBG_LOG("Sensor :: %d",result);
		
			ble_status = at_ble_characteristic_value_set(bruxa_service_characs[9].char_val_handle, &result, sizeof(uint8_t));
			ble_status = at_ble_notification_send(0,bruxa_service_characs[9].char_val_handle);
		
			delay_ms(100);
		}
		//port_pin_toggle_output_level(LED0_PIN);
	}
}
