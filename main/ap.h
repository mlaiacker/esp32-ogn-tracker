#pragma once

#include "hal.h"

// write a byte to tcp? socket
void AP_Write(char Byte);
// return true if at leat on client is connected to our AP
int AP_isClientConnected();

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskAP(void* pvParameters);
