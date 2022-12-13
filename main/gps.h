
#include "hal.h"

#include "timesync.h"
#include "ogn.h"
#include "lowpass2.h"
#include "flight.h"

// extern uint8_t GPS_PowerMode;               // 0=shutdown, 1=reduced, 2=normal

#ifdef WITH_ESP32
const  uint8_t GPS_PosPipeSize         = 16; // number of GPS positions held in a pipe
#else
const  uint8_t GPS_PosPipeSize         =  4; // number of GPS positions held in a pipe
#endif

extern uint8_t      GPS_PosIdx;                   // Pipe index, increments with every GPS position received
extern GPS_Position GPS_Pos[GPS_PosPipeSize];    // GPS position pipe

extern          GPS_Time GPS_DateTime;      // Time and date from the GPS
extern           int32_t GPS_Altitude;      // [0.1m] altitude (height above Geoid)
extern           int32_t GPS_Latitude;      // [0.0001/60 deg]
extern           int32_t GPS_Longitude;     // [0.0001/60 deg]
extern           int16_t GPS_GeoidSepar;    // [0.1m]
extern          uint16_t GPS_LatCosine;     // [1.0/(1<<12)] Latitude Cosine for distance calculations
extern          uint32_t GPS_TimeSinceLock; // [sec] time since GPS has a valid lock
extern          uint32_t GPS_Random;        // random number produced from the GPS data
extern          uint16_t GPS_PosPeriod;     // [msec] how often (which period) the GPS/MAV is sending the positions

extern          uint16_t GPS_SatSNR;        // [0.25dB] average SNR for satellites being tracked
extern           uint8_t GPS_SatCnt;        // [0.25dB] number of satellites being tracked

typedef union
         { uint8_t  Flags;
           struct
           { bool       NMEA:1; // got at least one valid NMEA message
             bool        UBX:1; // got at least one valid UBX message
             bool        MAV:1; // got at least one valid MAV message
             bool        PPS:1; // got at least one PPS signal
             bool BaudConfig:1; // baudrate is configured
             bool ModeConfig:1; // navigation mode is configured
             bool RateConfig:1; // navigation rate is configured
             bool           :1; //
           } ;
         } Status;                          //

extern Status GPS_Status;                   // GPS status bits

uint32_t GPS_getBaudRate(void);             // [bps]

GPS_Position *GPS_getPosition(void);
GPS_Position *GPS_getPosition(int8_t Sec);                                                  // return GPS position for given Sec
GPS_Position *GPS_getPosition(uint8_t &BestIdx, int16_t &BestRes, int8_t Sec, int16_t Frac, bool Ready=1); // return GPS position closest to the given Sec.Frac

int16_t GPS_AverageSpeed(void);             // [0.1m/s] calc. average speed based on most recent GPS positions

#ifdef WITH_MAVLINK
extern uint16_t MAVLINK_BattVolt;   // [mV]
extern uint16_t MAVLINK_BattCurr;   // [10mA]
extern uint8_t  MAVLINK_BattCap;    // [%]
extern uint8_t MAVLINK_sats;  // for debug
extern uint32_t MAVLINK_msgs;  // total messages received
#endif

extern EventGroupHandle_t GPS_Event;
const EventBits_t GPSevt_PPS    = 0x01;
const EventBits_t GPSevt_NewPos = 0x02;

extern FlightMonitor Flight;                // detect/monitor takeoff/flight/landing

#ifdef __cplusplus
  extern "C"
#endif
void vTaskGPS(void* pvParameters);

