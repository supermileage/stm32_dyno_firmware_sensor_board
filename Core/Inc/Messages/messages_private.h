#ifndef INC_MESSAGES_MESSAGES_PRIVATE_H_
#define INC_MESSAGES_MESSAGES_PRIVATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>



typedef enum : uint32_t
{  
    CHILD_BOARD_FUNCTION_SUCCESS = 0,
    CHILD_BOARD_FUNCTION_DISABLED = 1,
    CHILD_BOARD_FUNCTION_WAITING_FOR_DATA = 2,
    CHILD_BOARD_FUNCTION_ERROR = 3, 
    CHILD_BOARD_FUNCTION_WARNING = 4
} child_board_function_status_t;

#ifdef __cplusplus
}
#endif



#endif // INC_MESSAGES_MESSAGES_PRIVATE_H_