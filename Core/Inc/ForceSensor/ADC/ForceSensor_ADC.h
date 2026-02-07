#ifndef INC_FORCESENSOR_ADC_FORCESENSOR_ADC_H_
#define INC_FORCESENSOR_ADC_FORCESENSOR_ADC_H_

#include "main.h"

#include <stdbool.h>

#include "Messages/messages_public.h"
#include "Messages/messages_private.h"

#ifdef __cplusplus
extern "C" {
#endif

child_board_function_status_t ForceSensorADC_Init(void);
child_board_function_status_t ForceSensorADC_Enable(bool enable);
child_board_function_status_t ForceSensorADC_Run(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_FORCESENSOR_ADC_FORCESENSOR_ADC_H_ */
