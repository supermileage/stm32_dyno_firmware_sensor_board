#ifndef INC_MESSAGES_MESSAGES_PUBLIC_H_
#define INC_MESSAGES_MESSAGES_PUBLIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>



typedef enum : uint32_t
{
    CHILD_BOARD_TASK_INVALID_TASK_ID = 0,
    CHILD_BOARD_TASK_ID_OPTICAL_SENSOR = 100,
    CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADC,
    CHILD_BOARD_TASK_ID_FORCE_SENSOR_ADS1115
} child_board_task_ids_t;

#if defined(STM32H7xx_H) || defined(STM32G0xx_H)
_Static_assert(sizeof(child_board_task_ids_t) == 4, "Size of child_board_task_ids_t must be 4 bytes");
#endif

typedef struct 
{
    child_board_task_ids_t task_id;
    bool enable;
    uint32_t command_payload_length;
} mother_board_to_child_board_usart_command_header_t;

#if defined(STM32H7xx_H) || defined(STM32G0xx_H)
_Static_assert(sizeof(mother_board_to_child_board_usart_command_header_t) == 4 + 4 + 4, "Size of mother_board_to_child_board_usart_command_header_t must be 9 bytes");
#endif

typedef struct __attribute__((packed))
{
    uint8_t address;
    uint8_t multiplexer;
    uint8_t comparator_mode;
    uint8_t comparator_polarity;
    uint8_t comparator_latch_enabled;
    uint8_t comparator_queue_mode;
    uint8_t mode;
    uint8_t data_rate;
    uint8_t gain;
} child_board_force_sensor_ads1115_settings;


typedef struct __attribute__((packed))
{
    child_board_task_ids_t task_id;
    uint32_t error_id;
} child_board_task_error_data;

#if defined(STM32H7xx_H) || defined(STM32G0xx_H)
_Static_assert(sizeof(child_board_task_error_data) == 4 + 4, "Size of child_board_task_error_data must be 8 bytes");
#endif


typedef struct __attribute__((packed))
{
    uint32_t optical_count_posedges;
    uint32_t optical_timer_counter_value;
    uint32_t force_sensor_raw_value;
} child_board_usart_sensor_data_t;

#if defined(STM32H7xx_H) || defined(STM32G0xx_H)
_Static_assert(sizeof(child_board_usart_sensor_data_t) == 12, "Size of child_board_usart_sensor_data_t must be 12 bytes");
#endif

typedef struct
{
    child_board_usart_sensor_data_t sensor_data;
    child_board_task_error_data error_data;
} child_board_usart_combined_data_t;

#if defined(STM32H7xx_H) || defined(STM32G0xx_H)
_Static_assert(sizeof(child_board_usart_combined_data_t) == sizeof(child_board_usart_sensor_data_t) + sizeof(child_board_task_error_data), "Size of child_board_usart_combined_data_t must be size of child_board_usart_sensor_data_t + size of child_board_task_error_data");
#endif



typedef enum : uint32_t
{
    ERROR_FORCE_SENSOR_ADC_START_FAILURE = 0

} child_board_force_sensor_adc_error_ids;

#if defined(STM32H7xx_H) || defined(STM32G0xx_H)
_Static_assert(sizeof(child_board_force_sensor_adc_error_ids) == 4, "Size of child_board_force_sensor_adc_error_ids must be 4 bytes");
#endif

typedef enum : uint32_t
{
    ERROR_FORCE_SENSOR_ADS1115_NULL_SETTINGS_PTR = 0,
    ERROR_FORCE_SENSOR_ADS1115_CONSTR_FAIL,
    ERROR_FORCE_SENSOR_ADS1115_SET_MULTIPLEXER_FAIL,
    ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_FAIL,
    ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_POLARITY_FAIL,
    ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_LATCH_FAIL,
    ERROR_FORCE_SENSOR_ADS1115_SET_COMPARATOR_QUEUE_MODE_FAIL,
    ERROR_FORCE_SENSOR_ADS1115_SET_MODE_FAIL,
    ERROR_FORCE_SENSOR_ADS1115_SET_RATE_FAIL,
    ERROR_FORCE_SENSOR_ADS1115_SET_GAIN_FAIL,
    ERROR_FORCE_SENSOR_ADS1115_SET_CONVERSION_READY_PIN_MODE_FAIL,
    WARNING_FORCE_SENSOR_ADS1115_TRIGGER_CONVERSION_FAILURE = 10000,
    WARNING_FORCE_SENSOR_ADS1115_GET_CONVERSION_FAILURE
} child_board_force_sensor_ads1115_error_ids;

#if defined(STM32H7xx_H) || defined(STM32G0xx_H)
_Static_assert(sizeof(child_board_force_sensor_ads1115_error_ids) == 4, "Size of child_board_force_sensor_ads1115_error_ids must be 4 bytes");
#endif

typedef enum : uint32_t
{
    ERROR_OPTICAL_SENSOR_TIMER_START_FAIL = 0,
    ERROR_OPTICAL_SENSOR_TIMER_POSEDGES_COUNTER_START_FAIL = 1,
    ERROR_OPTICAL_SENSOR_TIMER_STOP_FAIL = 2,
    ERROR_OPTICAL_SENSOR_TIMER_POSEDGES_COUNTER_STOP_FAIL = 3
} child_board_optical_sensor_error_ids;

#if defined(STM32H7xx_H) || defined(STM32G0xx_H)
_Static_assert(sizeof(child_board_optical_sensor_error_ids) == 4, "Size of child_board_optical_sensor_error_ids must be 4 bytes");
#endif

typedef enum : uint32_t
{
    ERROR_USB_RX_INVALID_TASK_ID = 0,
    ERROR_USB_RX_CB_START_FAILURE = 1
} child_board_usb_error_ids;

static inline void ChildBoardPopulateTaskErrorDataStruct(child_board_task_error_data* error_data, child_board_task_ids_t task_id, uint32_t error_id)
{
    error_data->task_id = task_id;
    error_data->error_id = error_id;
}

#ifdef __cplusplus
}
#endif

#endif // INC_MESSAGES_MESSAGES_PUBLIC_H_