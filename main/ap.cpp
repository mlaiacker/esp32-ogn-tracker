#include <ctype.h>

#include "hal.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"

#include "format.h"
#include "fifo.h"
#include "socket.h"
#include "proc.h"
#include "wifi.h"
#include "http.h"

// #define DEBUG_PRINT

#ifdef WITH_AP

// ============================================================================================================

DataServer PortServer;

static FIFO<char, 1024> AP_TxFIFO;
static FIFO<uint8_t, 256> AP_RxFIFO;

void AP_Write(char Byte) {
	AP_TxFIFO.Write(Byte);
}

static int AP_TxPush(size_t MaxLen = 256) // transmit part of the TxFIFO to the TCP clients
{
	char *Data;
	size_t Len = AP_TxFIFO.getReadBlock(Data); // see how much data is there in the queue for transmission
	if (Len == 0)
		return 0;                              // if block is empty then give up
	if (Len > MaxLen)
		Len = MaxLen;                                    // limit the block size
	int Ret = PortServer.Send(Data, Len); // write the block to the Stratux socket
	if (Ret < 0)
		return -1;                                   // if an error then give up
	AP_TxFIFO.flushReadBlock(Len); // remove the transmitted block from the FIFO
	return Len;
}                                          // return number of transmitted bytes

// ============================================================================================================

esp_err_t AP_start(){
	esp_err_t err = WIFI_StartAP(Parameters.APname, Parameters.APpass);
	WIFI_setTxPower(Parameters.APtxPwr);
	WIFI_setPowerSave(1);
	return err;
}

int AP_isClientConnected(){
	wifi_sta_list_t clients;
	esp_wifi_ap_get_sta_list(&clients);
	return clients.num;
}


extern "C" void vTaskAP(void *pvParameters) {
	esp_err_t Err;
	AP_TxFIFO.Clear();
	AP_RxFIFO.Clear();
	vTaskDelay(1000);

	WIFI_State.Flags = 0;
	Err = WIFI_Init();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "WIFI_Init() => ");
  if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  AP_start();

	Err = PortServer.Listen(Parameters.APport);
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "PortServer.Listen() => ");
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

	Err = HTTP_Start();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "HTTP_Start() => ");
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

	for (;;)                                              // main (endless) loop
	{
		Err = AP_TxPush();
		if (Err > 0) {
			vTaskDelay(1);
			continue;
		}
		vTaskDelay(50);
		Err = PortServer.Accept();
		if(PowerMode<2){
			wifi_sta_list_t clients;
			esp_wifi_ap_get_sta_list(&clients);
			if(clients.num==0){
				// try to save some power by turning off AP
				WIFI_Stop();
				// wait until we are back in full power mode
				while(PowerMode<2) {
					vTaskDelay(1000);
				}
				// start AP again
				AP_start();
				vTaskDelay(100);
			}
		}
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "PortServer.Accept() => ");
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif

	}

}
#else
void AP_Write(char Byte) {
	// do nothing
}
int AP_isClientConnected(){
	return 0;
}
#endif // WITH_AP
