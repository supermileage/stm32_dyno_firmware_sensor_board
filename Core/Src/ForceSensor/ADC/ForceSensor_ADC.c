#include <ForceSensor/ADC/ForceSensor_ADC.h>

#include <stdint.h>

#define LBF_TO_NEWTON 4.44822

extern ADC_HandleTypeDef* forcesensor_adc_handle;

// Global interrupts
static volatile uint16_t adc_value = 0;
static volatile bool adc_conversion_complete = false;

static bool forcesensor_adc_enabled = false;

extern child_board_usart_combined_data_t usart_output_data;


child_board_function_status_t ForceSensorADC_Init()
{
	return true;
}

child_board_function_status_t ForceSensorADC_Enable(bool enable)
{
    forcesensor_adc_enabled = enable;
    return CHILD_BOARD_FUNCTION_SUCCESS;
}

child_board_function_status_t ForceSensorADC_Run(void)
{

    // Skip processing if the latest state says disabled
    if (!forcesensor_adc_enabled)
    {
        return CHILD_BOARD_FUNCTION_DISABLED;
    }

    adc_conversion_complete = false;

    // --- Trigger ADC conversion via interrupt ---
    if (HAL_ADC_Start_IT(forcesensor_adc_handle) != HAL_OK)
    {
        ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADC, (uint32_t)ERROR_FORCE_SENSOR_ADC_START_FAILURE);
        return CHILD_BOARD_FUNCTION_ERROR;
    }

    if (!adc_conversion_complete)
    {
        return CHILD_BOARD_FUNCTION_WAITING_FOR_DATA; // Wait for ADC conversion to complete
    }

    usart_output_data.sensor_data.force_sensor_raw_value = adc_value;
    return CHILD_BOARD_FUNCTION_SUCCESS;

}

void forcesensor_adc_interrupt()
{
	adc_value = HAL_ADC_GetValue(forcesensor_adc_handle);
    adc_conversion_complete = true;
}
