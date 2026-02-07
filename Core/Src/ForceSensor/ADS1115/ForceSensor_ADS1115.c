#include "ForceSensor/ADS1115/ForceSensor_ADS1115.h"

#define LBF_TO_NEWTON 4.44822
#define ADS1115_VOLTAGE 5.1

extern I2C_HandleTypeDef* forcesensor_ads1115_i2c_handle;

// Global interrupts
static volatile bool ads1115_alert_status = false;

static bool forcesensor_ads1115_enabled = false;

extern child_board_usart_combined_data_t usart_output_data;


child_board_function_status_t ForceSensorADS1115_Init(child_board_force_sensor_ads1115_settings* settings)
{
    if (settings == NULL)
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t)ERROR_FORCE_SENSOR_ADS1115_NULL_SETTINGS_PTR);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    if (!ADS1115_Constr(forcesensor_ads1115_i2c_handle, settings->address))
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t) ERROR_FORCE_SENSOR_ADS1115_CONSTR_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }
    
    if (!ADS1115_SetMultiplexer(settings->multiplexer))
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t) ERROR_FORCE_SENSOR_ADS1115_SET_MULTIPLEXER_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    if (!ADS1115_SetComparatorMode(settings->comparator_mode))
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t) ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    if (!ADS1115_SetComparatorPolarity(settings->comparator_polarity))
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t) ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_POLARITY_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    if (!ADS1115_SetComparatorLatchEnabled(settings->comparator_latch_enabled))
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t)ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_LATCH_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    if (!ADS1115_SetComparatorQueueMode(settings->comparator_queue_mode))
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t)ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_QUEUE_MODE_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    
    // Set device mode to single-shot
	if (!ADS1115_SetMode(settings->mode))
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t)ERROR_FORCE_SENSOR_ADS1115_SET_MODE_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    // Set data rate (slow for demonstration or high depending on application)
	if (!ADS1115_SetRate(settings->data_rate))
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t)ERROR_FORCE_SENSOR_ADS1115_SET_RATE_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    // Set PGA (programmable gain amplifier)
	if (!ADS1115_SetGain(settings->gain))
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t)ERROR_FORCE_SENSOR_ADS1115_SET_GAIN_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

	if (!ADS1115_SetConversionReadyPinMode())
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t)ERROR_FORCE_SENSOR_ADS1115_SET_CONVERSION_READY_PIN_MODE_FAIL);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    return CHILD_BOARD_FUNCTION_SUCCESS;

}

child_board_function_status_t ForceSensorADS1115_Enable(bool enable)
{
    forcesensor_ads1115_enabled = enable;
    return CHILD_BOARD_FUNCTION_SUCCESS;
}

child_board_function_status_t ForceSensorADS1115_Run(void)
{
    if (!forcesensor_ads1115_enabled)
    {
        return CHILD_BOARD_FUNCTION_DISABLED;
    }

    ads1115_alert_status = false;

    // --- Trigger conversion ---
    if (!ADS1115_TriggerConversion()) 
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t)WARNING_FORCE_SENSOR_ADS1115_TRIGGER_CONVERSION_FAILURE);
        return CHILD_BOARD_FUNCTION_WARNING;
    }

    // --- Wait for alert GPIO to indicate conversion complete ---
    if (!ads1115_alert_status)
    {
        return CHILD_BOARD_FUNCTION_WAITING_FOR_DATA;
    }

    // --- Read conversion and populate output ---
    int16_t rawVal;
    if (!ADS1115_GetConversion(&rawVal, false)) 
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115, (uint32_t)WARNING_FORCE_SENSOR_ADS1115_GET_CONVERSION_FAILURE);
        return CHILD_BOARD_FUNCTION_WARNING;
    }

    usart_output_data.sensor_data.force_sensor_raw_value = rawVal;
    return CHILD_BOARD_FUNCTION_SUCCESS;
}





void forcesensor_ads1115_gpio_alert_interrupt(void)
{
    ads1115_alert_status = true;
}
