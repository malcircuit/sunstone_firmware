/**
 *	Sunstone device firmware
 *
 *	Matt Sutter
 *	July, 2015
 */

//#define DEBUG

// COMMON DEPS
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include "nrf_delay.h"
#include "nrf51.h"
#include "sdk_common.h"
#include "app_error.h"
#include "nrf_drv_common.h"
#include "nrf_drv_config.h"

// I2C DEPS
#include "nrf_drv_twi.h"

// PWM DEPS
#include "app_pwm.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"

// BLE DEPS
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_nus.h"

/***************** Function Declarations *****************/
void set_r(uint8_t red);
void set_g(uint8_t green);
void set_b(uint8_t blue);
void set_warm(uint8_t warm);
void set_cool(uint8_t cool);

void set_white_brightness(uint8_t new_brightness);
void set_rgb_brightness(uint8_t new_brightness);

void rf_circuit_init();
void pwm_init();
void i2c_init();
void ble_init(ret_code_t * err_code);

/**
 * PWM callback function. Null for the moment
 */
void pwm_ready_callback(uint32_t pwm_id)
{

}

/***************** Constants *****************/

// PWM control pins
#define WHITE_PWM_PIN 0
#define COLOR_PWM_PIN 1

// Digital potentiometer I2C addresses
#define WHITE_POT 0x28	// White LED current control
#define RG_POT 0x29		// Red and Green LED current control
#define B_POT 0x2A		// Blue LED current control

// Digital potentiometer write addresses
#define REG_A 0x11
#define REG_B 0x12
#define REG_A_AND_B 0x13

// Default LED brightness values (current control)
#define R 0x00
#define G 0x00
#define B 0xFF
#define WARM 0xFF
#define COOL 0xFF

// Default LED brightness value (PWM control)
#define WHITE_BRIGHTNESS 0x00
#define RGB_BRIGHTNESS 0x80

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0
#define DEVICE_NAME                     "Sunstone"                               /**< Name of device. Will be included in the advertising data. */
#define SUNSTONE_BASE_UUID				{{0xBB, 0x8B, 0x6F, 0x8D, 0x9C, 0x15, 0xC3, 0x9C, 0xE8, 0x46, 0x2C, 0x58, 0x00, 0x00, 0xA4, 0x1B}}
#define SUNSTONE_SERVICE_UUID			0x0001
#define SUNSTONE_SERVICE_UUID_TYPE		BLE_UUID_TYPE_VENDOR_BEGIN

#define WHITE_BRIGHTNESS_CHAR_UUID		0x0002
#define COLOR_BRIGHTNESS_CHAR_UUID		0x0003
#define WARM_CHAR_UUID					0x0004
#define COOL_CHAR_UUID					0x0005
#define RED_CHAR_UUID					0x0006
#define GREEN_CHAR_UUID					0x0007
#define BLUE_CHAR_UUID					0x0008
#define SUNSTONE_CHAR_NUM				7

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2 // + BSP_APP_TIMERS_NUMBER)                 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define UNITLESS 						0x2700										// BLE Spec for a characteristic value with no physical units

/***************** Variable declarations *****************/

// Create I2C instance
const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(0);

// Create PWM instance
APP_PWM_INSTANCE(PWM1, 1);

// BLE variables
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;    								/**< Handle of the current connection. */
static ble_uuid_t m_adv_uuids[] = {{SUNSTONE_SERVICE_UUID, SUNSTONE_SERVICE_UUID_TYPE}};  	/**< Universally unique service identifier. */
uint16_t sunstone_service_handle, ble_conn_handle;
ble_gatts_char_handles_t sunstone_char_handles[SUNSTONE_CHAR_NUM];

// Variables for brightness values
volatile uint8_t red = R,
        		 green = G,
        		 blue = B,
        		 warm = WARM,
        		 cool = COOL,
        		 white_brightness = WHITE_BRIGHTNESS,
        		 color_brightness = RGB_BRIGHTNESS;

/** Variable to hold whatever the PWM library decides the cycle length (clock ticks) should be.
 *  Needed to get a full 8 bits of PWM resolution (alternative is only 0-100% duty) */
uint16_t cycle_ticks = 0;

volatile bool is_ble_adv_idle = false;

/**
 * Struct for Sunstone's BLE Characteristics
 */
typedef struct {
	ble_uuid_t char_uuid;
	uint8_t *user_desc;
	ble_gatts_char_handles_t gatt_char_handles;
	volatile uint8_t *attr_value;
} sunstone_char_t;

// Sunstone BLE Characteristic value descriptions
uint8_t * sunstone_char_user_descs[] =
{
		(uint8_t*)("Brightness of the white LEDs"),
		(uint8_t*)("Brightness of the color LEDs"),
		(uint8_t*)("Brightness of the warm white LEDs"),
		(uint8_t*)("Brightness of the cool white LEDs"),
		(uint8_t*)("Brightness of the red LEDs"),
		(uint8_t*)("Brightness of the green LEDs"),
		(uint8_t*)("Brightness of the blue LEDs")
};

// Sunstone BLE Characteristic value format
ble_gatts_char_pf_t sunstone_brightness_format =
{
		.format = BLE_GATT_CPF_FORMAT_UINT8,				// Unsigned 8-bit integer
		.exponent = 0,										// Start at 0
		.unit = UNITLESS,									// Value has no physical significance
		.name_space = BLE_GATT_CPF_NAMESPACE_BTSIG,			// Normal namespace
		.desc = BLE_GATT_CPF_NAMESPACE_DESCRIPTION_UNKNOWN	// No description for this format
};

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**
 * Function to add a Sunstone BLE Characteristic to the Sunstone BLE Service
 */
static uint32_t sunstone_char_add(sunstone_char_t * sunstone_char)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read			 = true;
    char_md.char_props.write         = true;
    char_md.char_props.write_wo_resp = true;
    char_md.char_ext_props.wr_aux	 = false;
    char_md.p_char_user_desc         = sunstone_char->user_desc;
    char_md.char_user_desc_max_size  = (uint16_t) strlen((char *)sunstone_char->user_desc);
    char_md.char_user_desc_size		 = (uint16_t)(strlen((char *)sunstone_char->user_desc) - 1);
    char_md.p_char_pf                = &sunstone_brightness_format;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_USER;	// R/W characteristic value directly from a variable in RAM
    attr_md.rd_auth = false;
    attr_md.wr_auth = false;
    attr_md.vlen    = false;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &(sunstone_char->char_uuid);
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 1;
    attr_char_value.p_value	  = (uint8_t *)sunstone_char->attr_value;

    return sd_ble_gatts_characteristic_add(	sunstone_service_handle,
    										&char_md,
    										&attr_char_value,
    										&(sunstone_char->gatt_char_handles));
}


/**
 * @brief Function for initializing BLE services that will be used by the application.
 */
static void services_init(void)
{
	uint8_t i;
	uint32_t      err_code;
	ble_uuid_t    sunstone_service_uuid =
	{
		.uuid = SUNSTONE_SERVICE_UUID,
		.type = SUNSTONE_SERVICE_UUID_TYPE
	};

	sunstone_char_t sunstone_chars[SUNSTONE_CHAR_NUM];

	ble_uuid_t sunstone_char_uuids[] =
	{
		{.uuid = WHITE_BRIGHTNESS_CHAR_UUID, 	.type = SUNSTONE_SERVICE_UUID_TYPE},
		{.uuid = COLOR_BRIGHTNESS_CHAR_UUID, 	.type = SUNSTONE_SERVICE_UUID_TYPE},
		{.uuid = WARM_CHAR_UUID, 				.type = SUNSTONE_SERVICE_UUID_TYPE},
		{.uuid = COOL_CHAR_UUID, 				.type = SUNSTONE_SERVICE_UUID_TYPE},
		{.uuid = RED_CHAR_UUID, 				.type = SUNSTONE_SERVICE_UUID_TYPE},
		{.uuid = GREEN_CHAR_UUID, 				.type = SUNSTONE_SERVICE_UUID_TYPE},
		{.uuid = BLUE_CHAR_UUID, 				.type = SUNSTONE_SERVICE_UUID_TYPE}
	};

	volatile uint8_t * values[] =
	{
		&white_brightness,
		&color_brightness,
		&warm,
		&cool,
		&red,
		&green,
		&blue
	};


	for (i = 0; i < SUNSTONE_CHAR_NUM; i++)
	{
		sunstone_chars[i].char_uuid = sunstone_char_uuids[i];
		sunstone_chars[i].user_desc = sunstone_char_user_descs[i];
		sunstone_chars[i].gatt_char_handles = sunstone_char_handles[i];
		sunstone_chars[i].attr_value = values[i];
	}

	ble_uuid128_t sunstone_base_uuid = SUNSTONE_BASE_UUID;

	err_code = sd_ble_uuid_vs_add(&sunstone_base_uuid, &sunstone_service_uuid.type);
	APP_ERROR_CHECK(err_code);

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &sunstone_service_uuid, &sunstone_service_handle);
	APP_ERROR_CHECK(err_code);

	for (i = 0; i < SUNSTONE_CHAR_NUM; i++)
	{
		err_code = sunstone_char_add(&sunstone_chars[i]);
		APP_ERROR_CHECK(err_code);
	}

}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
	uint32_t err_code;

	if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    //uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
        	is_ble_adv_idle = true;
            break;
        default:
            break;
    }
}


/**@brief Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in] p_ble_evt S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);

}


/**@brief Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	uint32_t err_code;

	rf_circuit_init();		// Activate RF circuitry
	i2c_init();				// Start I2C bus
	pwm_init();				// Start PWM bus

	ble_init(&err_code);	// Start BLE advertising

	while (true)
	{
		set_white_brightness(white_brightness);
		set_rgb_brightness(color_brightness);

		set_r(red);
		set_g(green);
		set_b(blue);
		set_warm(warm);
		set_cool(cool);

		// Constantly look for a connection
		if (is_ble_adv_idle)
		{
			err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);

			is_ble_adv_idle = false;
		}

		nrf_delay_ms(20);
	}

}


/**
 * Activate RF transmitter circuitry. Only applies to the Laird BL600 BLE module.
 */
void rf_circuit_init()
{
#ifdef BL600
	NRF_GPIO->DIRSET |= (1 << 20);
	NRF_GPIO->OUTCLR |= (1 << 20);
#endif
}

/**
 * Initialize I2C Bus
 */
void i2c_init(){
	ret_code_t err_code;
	err_code = nrf_drv_twi_init(&twi, NULL, NULL);	// Default config, no event handler (blocking mode)
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&twi);
}

/**
 * Initialize PWM channels
 */
void pwm_init(){

	/* The Nordic SDK does not initialize the values in the PWM struct, which
	 * it turns out causes a ton of problems/glitches.  I'm just going to go
	 * ahead and preemptively initialize all of the fields in the struct.
	 *
	 * I guessed on a few of these, so YMMV.
	 */
	PWM1.p_cb->channels_cb[0].gpio_pin = 1;
	PWM1.p_cb->channels_cb[0].pulsewidth = 0x00000000;
	PWM1.p_cb->channels_cb[0].ppi_channels[0] = NRF_PPI_CHANNEL26;
	PWM1.p_cb->channels_cb[0].ppi_channels[1] = NRF_PPI_CHANNEL27;
	PWM1.p_cb->channels_cb[0].polarity = APP_PWM_POLARITY_ACTIVE_LOW ;
	PWM1.p_cb->channels_cb[0].initialized = 0;

	PWM1.p_cb->channels_cb[1].gpio_pin = 0;
	PWM1.p_cb->channels_cb[1].pulsewidth = 0x00000000;
	PWM1.p_cb->channels_cb[1].ppi_channels[0] = NRF_PPI_CHANNEL28;
	PWM1.p_cb->channels_cb[1].ppi_channels[1] = NRF_PPI_CHANNEL29;
	PWM1.p_cb->channels_cb[1].polarity = APP_PWM_POLARITY_ACTIVE_LOW;
	PWM1.p_cb->channels_cb[1].initialized = 0;

	PWM1.p_cb->period = 0xFFFFFFFF;
	PWM1.p_cb->p_ready_callback = pwm_ready_callback;
	PWM1.p_cb->ppi_channels[0] = NRF_PPI_CHANNEL30;
	PWM1.p_cb->ppi_channels[1] = NRF_PPI_CHANNEL31;
	PWM1.p_cb->ppi_group = NRF_PPI_CHANNEL_GROUP3;
	PWM1.p_cb->state = NRF_DRV_STATE_UNINITIALIZED;

	// Initialize PWM channels 0 and 1 to 2kHz on pins 1 and 0, respectively. (Yeah, I messed that one up)
	app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(500L, COLOR_PWM_PIN, WHITE_PWM_PIN);
	ret_code_t err_code = app_pwm_init(&PWM1, &pwm1_cfg, pwm_ready_callback);
	APP_ERROR_CHECK(err_code);

	// Grab the number of clock ticks (resolution) of the PWM channel so we can actually get 8 bits of resolution.
	cycle_ticks = app_pwm_cycle_ticks_get(&PWM1);
	app_pwm_enable(&PWM1);
}


/**
 *  Initialize BLE stack and start advertising
 */
void ble_init(ret_code_t * err_code)
{
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
	ble_stack_init();
	gap_params_init();
	services_init();
	advertising_init();
	conn_params_init();

	*err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(*err_code);
}

/*
 * Set the PWM for the white LEDs
 */
void set_white_brightness(uint8_t new_brightness){
	uint16_t duty = (uint16_t)((float)new_brightness / 0xFF * cycle_ticks);
    while (app_pwm_channel_duty_ticks_set(&PWM1, WHITE_PWM_PIN, duty) == NRF_ERROR_BUSY);
}

/*
 *  Set the PWM for the color LEDs
 */
void set_rgb_brightness(uint8_t new_brightness){
	uint16_t duty = (uint16_t)((float)new_brightness / 0xFF * cycle_ticks);
    while (app_pwm_channel_duty_ticks_set(&PWM1, COLOR_PWM_PIN, duty) == NRF_ERROR_BUSY);
}

/**
 * Set the Red LED current
 */
void set_r(uint8_t red)
{
    uint8_t tx_data[] = {REG_A, red};
	uint32_t err_code = nrf_drv_twi_tx(&twi, RG_POT, tx_data, sizeof(tx_data), false);
    APP_ERROR_CHECK(err_code);
}

/**
 * Set the Green LED current
 */
void set_g(uint8_t green)
{
    uint8_t tx_data[] = {REG_B, green};
	uint32_t err_code = nrf_drv_twi_tx(&twi, RG_POT, tx_data, sizeof(tx_data), false);
    APP_ERROR_CHECK(err_code);
}

/**
 * Set the Blue LED current
 */
void set_b(uint8_t blue)
{
    uint8_t tx_data[] = {REG_A, blue};
	uint32_t err_code = nrf_drv_twi_tx(&twi, B_POT, tx_data, sizeof(tx_data), false);
    APP_ERROR_CHECK(err_code);
}

/**
 * Set the Warm White LED current
 */
void set_warm(uint8_t warm)
{
    uint8_t tx_data[] = {REG_A, warm};
	uint32_t err_code = nrf_drv_twi_tx(&twi, WHITE_POT, tx_data, sizeof(tx_data), false);
    APP_ERROR_CHECK(err_code);
}

/**
 * Set the Cool White LED current
 */
void set_cool(uint8_t cool)
{
    uint8_t tx_data[] = {REG_B, cool};
	uint32_t err_code = nrf_drv_twi_tx(&twi, WHITE_POT, tx_data, sizeof(tx_data), false);
    APP_ERROR_CHECK(err_code);
}
