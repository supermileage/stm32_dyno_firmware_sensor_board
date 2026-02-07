#ifndef INC_OPTICALSENSOR_OPTICALSENSOR_H_
#define INC_OPTICALSENSOR_OPTICALSENSOR_H_

#include <stdbool.h>
#include "main.h"

#include <Messages/messages_public.h>
#include <Messages/messages_private.h>

child_board_function_status_t OpticalSensor_Init(void);
child_board_function_status_t OpticalSensor_Enable(bool enable);
child_board_function_status_t OpticalSensor_Run(void);

void OpticalSensor_Overflow_Interrupt();


#endif // INC_OPTICALSENSOR_OPTICALSENSOR_H_