#include "hal.h"
#ifdef WITH_APRS

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include <ctype.h>
#include "format.h"

#include "socket.h"

#include "proc.h"
#include "gps.h"

#include "wifi.h"
#include "aprs.h"

#define DEBUG_PRINT

// ======================================================================================================

static wifi_ap_record_t AP[8];                        // lists of Access Points from the WiFi scan
static uint16_t APs=0;

const int MaxLineLen=512;
static char Line[MaxLineLen];                         // for printing and buffering

static void AP_Print(void (*Output)(char))            // print lists of AP's
{ for(uint16_t Idx=0; Idx<APs; Idx++)
  { Line[0]='0'+Idx; Line[1]=':'; Line[2]=' ';
    uint8_t Len=3+AP_Print(Line+3, AP+Idx);
    Format_String(Output, Line); }
}

FIFO<OGN_RxPacket<OGN_Packet>, 16> APRSrx_FIFO;                     // packets received on the air, to be passed to the APRS
FIFO<OGN_TxPacket<OGN_Packet>,  4> APRStx_FIFO;                     // own position/status/info packets to besent to APRS

static Socket APRS_Socket;                                          // socket to talk to APRS server
bool APRS_isConnected(void) { return APRS_Socket.isConnected(); }

static const char *APRS_Host = "aprs.glidernet.org";                // server address
static const char *APRS_Port = "14580";                             // server socket
static const char *APRS_Soft = "OGN-Tracker";                       // software name for APRS server

static uint16_t APRS_CallPass(const char *Call)                     // calc. password for the APRS server login
{ uint16_t Hash=0x73e2;
  for( ; ; )
  { uint16_t High=(*Call++); if((High==0)||(High=='-')) break;
    High=toupper(High); Hash^=(High<<8);
    uint16_t Low =(*Call++); if((Low ==0)||(Low =='-')) break;
    Low=toupper(Low); Hash^=(Low); }
  return Hash&0x7FFF; }

static uint8_t LoginLine(char *Line, uint16_t Range=0 )              // produce login line for the APRS server
{ uint8_t Len=0;
  Len += Format_String(Line, "user ");                               // user name
  Len += Parameters.getAprsCall(Line+Len);                           // produce from the parameters
  uint16_t Pass = APRS_CallPass(Line+5);
  Len += Format_String(Line+Len, " pass ");                          // password
  Len += Format_UnsDec(Line+Len, Pass);
  Len += Format_String(Line+Len, " vers OGN-Tracker v0.0");          // software name and version
  if(Range)
  { Len += Format_String(Line+Len, " filter m/");   // ask for traffic 5km around own location
    Len += Format_UnsDec(Line+Len, Range); }
  Line[Len++]='\n';
  Line[Len]=0;
  return Len; }

static int APRS_RxMsg(const char *Msg)     // Handle messages received from APRS
{
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "APRS -> ");
  Format_String(CONS_UART_Write, Msg);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  return 0; }

static int APRS_Dialog(void)               // connect to APRS and talk to the server exchaging data
{ uint32_t AcftID = Parameters.AcftID;                     // remember ID in case it changes
  int ConnErr=APRS_Socket.Connect(APRS_Host, APRS_Port);   // connect to the APRS server
  if(ConnErr>=0)                                           // if connection succesfull
  { uint8_t LoginLen = LoginLine(Line, 5);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Connected to ");
    IP_Print(CONS_UART_Write, APRS_Socket.getIP());
    Format_String(CONS_UART_Write, "\nAPRS <- ");
    Format_String(CONS_UART_Write, Line);
    xSemaphoreGive(CONS_Mutex);
#endif
    TickType_t LastRx = xTaskGetTickCount();               // to detect idle connection
    APRS_Socket.setReceiveTimeout(1);                      // [sec] receive timeout
    // APRS_Socket.setBlocking(0);
    int Write=APRS_Socket.Send(Line, LoginLen);            // send login to the APRS server
    int LinePtr=0;
    for( ; ; )                                             // the dialog loop
    { if(AcftID!=Parameters.AcftID) break;                 // stop when aircraft ID changes: we need to relogin
      int Left = MaxLineLen-LinePtr; if(Left<128) break;   // how much space left for receive data
      int Read=APRS_Socket.Receive(Line+LinePtr, Left-1);  // read more data from the socket
#ifdef DEBUG_PRINT
      if(Read<0)                                                         // if error then print it
      { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "APRS: ");
        Format_SignDec(CONS_UART_Write, Read);
        Format_String(CONS_UART_Write, "\n");
        xSemaphoreGive(CONS_Mutex); }
#endif
      if(Read<0) break;                                                  // if an error reading from the APRS server
      if(Read==0)                                                        // no more data on the receive
      { TickType_t Idle = xTaskGetTickCount()-LastRx;                    // [ms] reception idling
        if(Idle>=60000) break;                                           // if idle for more than one minute then break the loop thus go for disconnect
        bool TimeValid = GPS_DateTime.isTimeValid() && GPS_DateTime.isDateValid();
        uint32_t Time = GPS_DateTime.getUnixTime();
        char *Msg = Line+LinePtr;
        while(APRStx_FIFO.Full())                                        // if own packets to be sent to the APRS
        { OGN_TxPacket<OGN_Packet> *TxPacket=APRStx_FIFO.getRead();
          if(TimeValid)
          { uint8_t Len = TxPacket->Packet.WriteAPRS(Msg, Time); Msg[Len++]='\n'; Msg[Len]=0;
#ifdef DEBUG_PRINT
            xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
            Format_String(CONS_UART_Write, "APRS <- ");
            Format_String(CONS_UART_Write, Msg);
            xSemaphoreGive(CONS_Mutex);
#endif
            int Write = APRS_Socket.Send(Msg, Len); if(Write<0) break; } // send the message to APRS, break the look when an error
          APRStx_FIFO.Read(); }
        while(APRSrx_FIFO.Full())                                        // if received packets to be forwarded to the APRS
        { OGN_RxPacket<OGN_Packet> *RxPacket=APRSrx_FIFO.getRead();
          uint8_t RxErr = RxPacket->RxErr;
          if(TimeValid && RxErr<=8)
          { uint8_t Len = RxPacket->Packet.WriteAPRS(Msg, Time, "OGNTRK");
            if(RxErr) { Msg[Len++]=' '; Msg[Len++]='0'+RxErr; Msg[Len++]='e'; }
            Msg[Len++]='\n'; Msg[Len]=0;
#ifdef DEBUG_PRINT
            xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
            Format_String(CONS_UART_Write, "APRS <- ");
            Format_String(CONS_UART_Write, Msg);
            xSemaphoreGive(CONS_Mutex);
#endif
            int Write = APRS_Socket.Send(Msg, Len); if(Write<0) break; }
          APRSrx_FIFO.Read(); }
        continue; }
      LastRx = xTaskGetTickCount();
      int LineLen = LinePtr+Read;
      int MsgPtr = 0;
      Line[LineLen]=0;
      for(int Idx=LinePtr; Idx<LineLen; Idx++)
      { char ch=Line[Idx]; if(ch==0) break;
        if(ch=='\n')
        { Line[Idx]=0; APRS_RxMsg(Line+MsgPtr); MsgPtr=Idx+1; }
      }
      if(MsgPtr>0)
      { int Copy=LineLen-MsgPtr;
        if(Copy>0)
        { memcpy(Line, Line+MsgPtr, Copy); LinePtr=0; }
      }
    }
  }
#ifdef DEBUG_PRINT
  else                                                      // if connection failed to the APRS server
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Failed to connect to APRS -> ");
    Format_SignDec(CONS_UART_Write, ConnErr);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex); }
#endif
  APRS_Socket.Disconnect();
  return 0; }

extern "C"
void vTaskAPRS(void* pvParameters)
{ esp_err_t Err;
  vTaskDelay(1000);

  Err=WIFI_Init();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "WIFI_Init() => ");
  if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  for( ; ; )                                                       // main (endless) loop
  { vTaskDelay(15000);                                             // this wait should be more intelligent
    Err=WIFI_Start();                                              // start WiFi
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_Start() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
/*
    vTaskDelay(5000);
    APs=8;
    Err=WIFI_ActiveScan(AP, APs);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_ActiveScan() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    CONS_UART_Write('/');
    Format_UnsDec(CONS_UART_Write, APs);
    Format_String(CONS_UART_Write, "\n");
    if(Err==ESP_OK) AP_Print(CONS_UART_Write);
    xSemaphoreGive(CONS_Mutex);
#endif
*/
    vTaskDelay(1000);
    APs=8;
    Err=WIFI_PassiveScan(AP, APs);                           // perform a passive scan: find Access Points around
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_PassiveScan() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    CONS_UART_Write('/');
    Format_UnsDec(CONS_UART_Write, APs);
    Format_String(CONS_UART_Write, "\n");
    if(Err==ESP_OK) AP_Print(CONS_UART_Write);
    xSemaphoreGive(CONS_Mutex);
#endif

    if(Err==ESP_OK)                                           // if WiFi scan went well
    { for(uint16_t Idx=0; Idx<APs; Idx++)                     // loop over Access Points
      { const char *NetName  = (const char *)(AP[Idx].ssid);
        const char *NetPass = 0;
        if(AP[Idx].authmode!=WIFI_AUTH_OPEN)                  // if not an open network
        { NetPass=Parameters.getWIFIpass(NetName);            // then search the password
          if(NetPass==0) continue; }                          // give up if no password for this network
        Err=WIFI_Connect(AP+Idx, NetPass);
#ifdef DEBUG_PRINT
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, NetName);
        if(NetPass) { CONS_UART_Write('/'); Format_String(CONS_UART_Write, NetPass); }
        CONS_UART_Write(':');
        CONS_UART_Write(' ');
        Format_String(CONS_UART_Write, "WIFI_Connect() => ");
        if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
        Format_SignDec(CONS_UART_Write, Err);
        Format_String(CONS_UART_Write, "\n");
        xSemaphoreGive(CONS_Mutex);
#endif
        if(Err) continue;                                     // if connection failed then give up ad move on to the next AP

        WIFI_IP.ip.addr = 0; WIFI_IP.gw.addr=0;
        for(uint8_t Idx=0; Idx<10; Idx++)                     // wait to obtain local IP from DHCP
        { vTaskDelay(1000);
          // if(WIFI_State.isConnected==1) break;
          if(WIFI_getLocalIP())
          { if(WIFI_IP.ip.addr && WIFI_IP.gw.addr) break; }
        }

#ifdef DEBUG_PRINT
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "Local IP: ");
        IP_Print(CONS_UART_Write, WIFI_IP.ip.addr);
        Format_String(CONS_UART_Write, " GW: ");
        IP_Print(CONS_UART_Write, WIFI_IP.gw.addr);
        Format_String(CONS_UART_Write, "\n");
        xSemaphoreGive(CONS_Mutex);
#endif
        if(WIFI_IP.ip.addr==0) { WIFI_Disconnect(); continue; }     // if getting local IP failed then give up and try another AP

        APRS_Dialog();                                              // talk to the APRS server

        vTaskDelay(5000);
        WIFI_Disconnect(); WIFI_IP.ip.addr=0;
        vTaskDelay(2000);
      }
    }

    vTaskDelay(2000);
    Err=WIFI_Stop();
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_Stop() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  }
}

#endif // WITH_APRS

