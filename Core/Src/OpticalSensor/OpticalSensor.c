#include <OpticalSensor/OpticalSensor.h>

static volatile uint32_t timer_counter_value = 0;
static volatile uint32_t timer_counter_value_num_overflows = 0;
static volatile uint32_t optical_count_posedges = 0;

static bool optical_encoder_enabled = false;

extern TIM_HandleTypeDef* optical_timer;
extern TIM_HandleTypeDef* optical_posedges_counter_timer;

extern child_board_usart_combined_data_t usart_output_data;

static void OpticalSensor_Reset()
{
    __HAL_TIM_SET_COUNTER(optical_posedges_counter_timer, 0);
    __HAL_TIM_SET_COUNTER(optical_timer, 0);
    timer_counter_value = 0;
    timer_counter_value_num_overflows = 0;
    optical_count_posedges = 0;
}

child_board_function_status_t OpticalSensor_Init()
{
    if (!optical_encoder_enabled)
    {
        return CHILD_BOARD_FUNCTION_DISABLED;
    }
    
    OpticalSensor_Reset();
    return CHILD_BOARD_FUNCTION_SUCCESS;
}

child_board_function_status_t OpticalSensor_Enable(bool enable)
{
    optical_encoder_enabled = enable;
    OpticalSensor_Reset();
    if (enable)
    {
        if (HAL_TIM_Base_Start_IT(optical_timer) != HAL_OK)
        {
            ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_OPTICAL_SENSOR, (uint32_t)ERROR_OPTICAL_SENSOR_TIMER_START_FAIL);
            return CHILD_BOARD_FUNCTION_ERROR;  
        }

        if (HAL_TIM_Base_Start_IT(optical_posedges_counter_timer) != HAL_OK)
        {
            ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_OPTICAL_SENSOR, (uint32_t)ERROR_OPTICAL_SENSOR_TIMER_POSEDGES_COUNTER_START_FAIL);
            return CHILD_BOARD_FUNCTION_ERROR;  
        }   
        
    }
    else
    {
        if (HAL_TIM_Base_Stop_IT(optical_timer) != HAL_OK)
        {
            ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_OPTICAL_SENSOR, (uint32_t)ERROR_OPTICAL_SENSOR_TIMER_STOP_FAIL);
            return CHILD_BOARD_FUNCTION_ERROR;  
        }
        if (HAL_TIM_Base_Stop_IT(optical_posedges_counter_timer) != HAL_OK)
        {
            ChildBoardPopulateTaskErrorDataStruct(&usart_output_data.error_data, CHILD_BOARD_TASK_ID_OPTICAL_SENSOR, (uint32_t)ERROR_OPTICAL_SENSOR_TIMER_POSEDGES_COUNTER_STOP_FAIL);
            return CHILD_BOARD_FUNCTION_ERROR;  
        }   
    }

    return CHILD_BOARD_FUNCTION_SUCCESS;
}

child_board_function_status_t OpticalSensor_Run(void)
{
    if (!optical_encoder_enabled)
    {
        return CHILD_BOARD_FUNCTION_DISABLED;
    }

    __disable_irq();
    usart_output_data.sensor_data.optical_count_posedges = __HAL_TIM_GET_COUNTER(optical_posedges_counter_timer);
    usart_output_data.sensor_data.optical_timer_counter_value = __HAL_TIM_GET_COUNTER(optical_timer) + (timer_counter_value_num_overflows * (__HAL_TIM_GET_AUTORELOAD(optical_timer) + 1));
    timer_counter_value_num_overflows = 0;
    __enable_irq();

    __HAL_TIM_SET_COUNTER(optical_posedges_counter_timer, 0);
    __HAL_TIM_SET_COUNTER(optical_timer, 0);

    return CHILD_BOARD_FUNCTION_SUCCESS;
}

void OpticalSensor_Overflow_Interrupt()
{
    timer_counter_value_num_overflows = timer_counter_value_num_overflows + 1;
}


