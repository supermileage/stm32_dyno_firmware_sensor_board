#ifndef INC_FORCESENSOR_ADS1115_FORCESENSOR_ADS1115_H_
#define INC_FORCESENSOR_ADS1115_FORCESENSOR_ADS1115_H_

#include "Messages/messages_public.h"
#include "Messages/messages_private.h"

#include "ADS1115.h"

#ifdef __cplusplus
extern "C" {
#endif

child_board_function_status_t ForceSensorADS1115_Init(child_board_force_sensor_ads1115_settings* settings);
child_board_function_status_t ForceSensorADS1115_Enable(bool enable);
child_board_function_status_t ForceSensorADS1115_Run(void);

#ifdef __cplusplus
}
#endif



#endif /* INC_TASKS_FORCESENSOR_ADS1115_FORCESENSOR_ADS1115_H_ */
