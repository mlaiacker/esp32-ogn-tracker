#include <stdio.h>

#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "hal.h"

#include "sens.h"
#include "rf.h"
#include "ctrl.h"
#include "proc.h"

#include "stratux.h"
#include "wifi.h"
#include "bt.h"
#include "ap.h"

#include "gps.h"
#include "format.h"

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)

static char disp_str[128];

// ========================================================================================================================

#ifdef WITH_OLED

#include "disp_oled.h"

int OLED_DisplayStatus(uint32_t Time, uint8_t LineIdx)
{
  Format_String(disp_str, "OGN Tx/Rx      ");
  Format_HHMMSS(disp_str + 10, Time);
  OLED_PutLine(LineIdx++, disp_str);
  Parameters.Print(disp_str);
  OLED_PutLine(LineIdx++, disp_str);
  return 0;
}

int OLED_DisplayPosition(GPS_Position *GPS, uint8_t LineIdx)
{
  if (GPS && GPS->isValid())
  {
    disp_str[0] = ' ';
    Format_SignDec(disp_str + 1, GPS->Latitude / 60, 6, 4);
    disp_str[9] = ' ';
    Format_UnsDec(disp_str + 10, GPS->Altitude / 10, 5, 0);
    disp_str[15] = 'm';
    OLED_PutLine(LineIdx, disp_str);
    Format_SignDec(disp_str, GPS->Longitude / 60, 7, 4);
    Format_SignDec(disp_str + 10, GPS->ClimbRate, 4, 1);
    OLED_PutLine(LineIdx + 1, disp_str);
    Format_UnsDec(disp_str, GPS->Speed, 4, 1);
    Format_String(disp_str + 5, "m/s  ");
    Format_UnsDec(disp_str + 10, GPS->Heading, 4, 1);
    disp_str[15] = '^';
    OLED_PutLine(LineIdx + 2, disp_str);
    Format_String(disp_str, "0D/00sat DOP00.0");
    disp_str[0] += GPS->FixMode;
    Format_UnsDec(disp_str + 3, GPS->Satellites, 2);
    Format_UnsDec(disp_str + 12, (uint16_t)GPS->HDOP, 3, 1);
    OLED_PutLine(LineIdx + 3, disp_str);
  }
  else
  {
    OLED_PutLine(LineIdx, 0);
    OLED_PutLine(LineIdx + 1, 0);
    OLED_PutLine(LineIdx + 2, 0);
    OLED_PutLine(LineIdx + 3, 0);
  }
  if (GPS && GPS->isDateValid())
  {
    Format_UnsDec(disp_str, (uint16_t)GPS->Day, 2, 0);
    disp_str[2] = '.';
    Format_UnsDec(disp_str + 3, (uint16_t)GPS->Month, 2, 0);
    disp_str[5] = '.';
    Format_UnsDec(disp_str + 6, (uint16_t)GPS->Year, 2, 0);
    disp_str[8] = ' ';
    disp_str[9] = ' ';
  }
  else
    Format_String(disp_str, "          ");
  if (GPS && GPS->isTimeValid())
  {
    Format_UnsDec(disp_str + 10, (uint16_t)GPS->Hour, 2, 0);
    Format_UnsDec(disp_str + 12, (uint16_t)GPS->Min, 2, 0);
    Format_UnsDec(disp_str + 14, (uint16_t)GPS->Sec, 2, 0);
  }
  else
    disp_str[10] = 0;
  OLED_PutLine(LineIdx + 4, disp_str);
  disp_str[0] = 0;
  if (GPS && GPS->hasBaro)
  {
    Format_String(disp_str, "0000.0hPa 00000m");
    Format_UnsDec(disp_str, GPS->Pressure / 40, 5, 1);
    Format_UnsDec(disp_str + 10, GPS->StdAltitude / 10, 5, 0);
  }
  OLED_PutLine(LineIdx + 5, disp_str);
  return 0;
}
#endif

// ========================================================================================================================

#ifdef WITH_U8G2_OLED

void OLED_DrawLogo(u8g2_t *OLED, GPS_Position *GPS) // draw logo and hardware options in software
{
  u8g2_DrawCircle(OLED, 96, 32, 30, U8G2_DRAW_ALL);
  u8g2_DrawCircle(OLED, 96, 32, 34, U8G2_DRAW_UPPER_RIGHT);
  u8g2_DrawCircle(OLED, 96, 32, 38, U8G2_DRAW_UPPER_RIGHT);
  // u8g2_SetFont(OLED, u8g2_font_open_iconic_all_4x_t);
  // u8g2_DrawGlyph(OLED, 64, 32, 0xF0);
  u8g2_SetFont(OLED, u8g2_font_ncenB14_tr);
  u8g2_DrawStr(OLED, 74, 31, "OGN");
  u8g2_SetFont(OLED, u8g2_font_8x13_tr);
  u8g2_DrawStr(OLED, 69, 43, "Tracker");

#ifdef WITH_FollowMe
  u8g2_DrawStr(OLED, 0, 16, "FollowMe");
#endif
#ifdef WITH_TTGO
  u8g2_DrawStr(OLED, 0, 16, "TTGO");
#endif
#if defined(WITH_HELTEC) || defined(WITH_HELTEC_V2) || defined(WITH_HELTEC_V3)
  u8g2_DrawStr(OLED, 0, 16, "HELTEC");
#endif
#if defined(WITH_TBEAM) || defined(WITH_TBEAM_V10)
  u8g2_DrawStr(OLED, 0, 16, "T-BEAM");
#endif

#ifdef WITH_GPS_MTK
  u8g2_DrawStr(OLED, 0, 28, "MTK GPS");
#endif
#ifdef WITH_GPS_UBX
  u8g2_DrawStr(OLED, 0, 28, "UBX GPS");
#endif
#ifdef WITH_GPS_SRF
  u8g2_DrawStr(OLED, 0, 28, "SRF GPS");
#endif
#ifdef WITH_MAVLINK
  u8g2_DrawStr(OLED, 0, 28, "MAVLINK");
#endif

#ifdef WITH_RFM95
  u8g2_DrawStr(OLED, 0, 40, "RFM95");
#endif
#ifdef WITH_RFM69
  u8g2_DrawStr(OLED, 0, 40, "RFM69");
#endif
#ifdef WITH_SX1262
  u8g2_DrawStr(OLED, 0, 40, "SX1262");
#endif

#ifdef WITH_BMP180
  u8g2_DrawStr(OLED, 0, 52, "BMP180");
#endif
#ifdef WITH_BMP280
  u8g2_DrawStr(OLED, 0, 52, "BMP280");
#endif
#ifdef WITH_BME280
  u8g2_DrawStr(OLED, 0, 52, "BME280");
#endif
#ifdef WITH_MS5607
  u8g2_DrawStr(OLED, 0, 52, "MS5607");
#endif
#ifdef WITH_MS5611
  u8g2_DrawStr(OLED, 0, 52, "MS5611");
#endif

#ifdef WITH_BT_SPP
  u8g2_DrawStr(OLED, 0, 64, "BT SPP");
#endif
}

void OLED_PutLine(u8g2_t *OLED, uint8_t LineIdx, const char *disp_str)
{
  if (disp_str == 0)
    return;
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "OLED_PutLine( ,");
  Format_UnsDec(CONS_UART_Write, (uint16_t)LineIdx);
  CONS_UART_Write(',');
  Format_String(CONS_UART_Write, disp_str);
  Format_String(CONS_UART_Write, ")\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  // u8g2_SetFont(OLED, u8g2_font_5x8_tr);
  u8g2_SetFont(OLED, u8g2_font_amstrad_cpc_extended_8r);
  u8g2_DrawStr(OLED, 0, (LineIdx + 1) * 8, disp_str);
}

void OLED_DrawStatus(u8g2_t *OLED, uint32_t Time, uint8_t LineIdx = 0)
{
  Format_String(disp_str, "OGN Tx/Rx      ");
  Format_HHMMSS(disp_str + 10, Time);
  disp_str[16] = 0;
  OLED_PutLine(OLED, LineIdx++, disp_str);
  Parameters.Print(disp_str);
  disp_str[16] = 0;
  OLED_PutLine(OLED, LineIdx++, disp_str);
}

void OLED_DrawPosition(u8g2_t *OLED, GPS_Position *GPS = 0, uint8_t LineIdx = 2)
{
  if (GPS && GPS->isValid())
  {
    disp_str[0] = ' ';
    Format_SignDec(disp_str + 1, GPS->Latitude / 60, 6, 4);
    disp_str[9] = ' ';
    Format_UnsDec(disp_str + 10, GPS->Altitude / 10, 5, 0);
    disp_str[15] = 'm';
    OLED_PutLine(OLED, LineIdx, disp_str);
    Format_SignDec(disp_str, GPS->Longitude / 60, 7, 4);
    Format_SignDec(disp_str + 10, GPS->ClimbRate, 4, 1);
    OLED_PutLine(OLED, LineIdx + 1, disp_str);
    Format_UnsDec(disp_str, GPS->Speed, 4, 1);
    Format_String(disp_str + 5, "m/s  ");
    Format_UnsDec(disp_str + 10, GPS->Heading, 4, 1);
    disp_str[15] = '^';
    OLED_PutLine(OLED, LineIdx + 2, disp_str);
    Format_String(disp_str, "0D/00sat DOP00.0");
    disp_str[0] += GPS->FixMode;
    Format_UnsDec(disp_str + 3, GPS->Satellites, 2);
    Format_UnsDec(disp_str + 12, (uint16_t)GPS->HDOP, 3, 1);
    OLED_PutLine(OLED, LineIdx + 3, disp_str);
  }
  // else { OLED_PutLine(OLED, LineIdx, 0); OLED_PutLine(OLED, LineIdx+1, 0); OLED_PutLine(LineIdx+2, 0); OLED_PutLine(LineIdx+3, 0); }
  if (GPS && GPS->isDateValid())
  {
    Format_UnsDec(disp_str, (uint16_t)GPS->Day, 2, 0);
    disp_str[2] = '.';
    Format_UnsDec(disp_str + 3, (uint16_t)GPS->Month, 2, 0);
    disp_str[5] = '.';
    Format_UnsDec(disp_str + 6, (uint16_t)GPS->Year, 2, 0);
    disp_str[8] = ' ';
    disp_str[9] = ' ';
  }
  else
    Format_String(disp_str, "          ");
  if (GPS && GPS->isTimeValid())
  {
    Format_UnsDec(disp_str + 10, (uint16_t)GPS->Hour, 2, 0);
    Format_UnsDec(disp_str + 12, (uint16_t)GPS->Min, 2, 0);
    Format_UnsDec(disp_str + 14, (uint16_t)GPS->Sec, 2, 0);
  }
  else
    disp_str[10] = 0;
  OLED_PutLine(OLED, LineIdx + 4, disp_str);
  disp_str[0] = 0;
  if (GPS && GPS->hasBaro)
  {
    Format_String(disp_str, "0000.0hPa 00000m");
    Format_UnsDec(disp_str, GPS->Pressure / 40, 5, 1);
    Format_UnsDec(disp_str + 10, GPS->StdAltitude / 10, 5, 0);
  }
  OLED_PutLine(OLED, LineIdx + 5, disp_str);
}

void OLED_DrawGPS(u8g2_t *OLED, GPS_Position *GPS) // GPS time, position, altitude
{
  bool isAltitudeUnitMeter = Parameters.AltitudeUnit == 0; // display altitude in meters
  bool isAltitudeUnitFeet = Parameters.AltitudeUnit == 1;  // display altitude in feet
  // u8g2_SetFont(OLED, u8g2_font_ncenB14_tr);
  u8g2_SetFont(OLED, u8g2_font_7x13_tf); // 5 lines, 12 pixels/line
  uint8_t Len = 0;
  /*
    Len+=Format_String(disp_str+Len, "GPS ");
    if(GPS && GPS->isValid())
    { disp_str[Len++]='0'+GPS->FixMode; disp_str[Len++]='D'; disp_str[Len++]='/';
      Len+=Format_UnsDec(disp_str+Len, GPS->Satellites, 1);
      Len+=Format_String(disp_str+Len, "sat DOP");
      Len+=Format_UnsDec(disp_str+Len, (uint16_t)GPS->HDOP, 2, 1); }
    else
    { Len+=Format_String(disp_str+Len, "(no lock)"); }
    disp_str[Len]=0;
    u8g2_DrawStr(OLED, 0, 12, disp_str);
  */
  if (GPS && GPS->isDateValid())
  {
    Format_UnsDec(disp_str, (uint16_t)GPS->Day, 2, 0);
    disp_str[2] = '.';
    Format_UnsDec(disp_str + 3, (uint16_t)GPS->Month, 2, 0);
    disp_str[5] = '.';
    Format_UnsDec(disp_str + 6, (uint16_t)GPS->Year, 2, 0);
    disp_str[8] = ' ';
  }
  else
    Format_String(disp_str, "  .  .   ");
  if (GPS && GPS->isTimeValid())
  {
    Format_UnsDec(disp_str + 9, (uint16_t)GPS->Hour, 2, 0);
    disp_str[11] = ':';
    Format_UnsDec(disp_str + 12, (uint16_t)GPS->Min, 2, 0);
    disp_str[14] = ':';
    Format_UnsDec(disp_str + 15, (uint16_t)GPS->Sec, 2, 0);
  }
  else
    Format_String(disp_str + 9, "  :  :  ");
  disp_str[17] = 0;
  u8g2_DrawStr(OLED, 0, 24, disp_str);

  Len = 0;
  Len += Format_String(disp_str + Len, "Lat:  ");
  if (GPS && GPS->isValid())
  {
    Len += Format_SignDec(disp_str + Len, GPS->Latitude / 6, 7, 5);
    disp_str[Len++] = 0xB0;
  }
  else
    Len += Format_String(disp_str + Len, "---.-----");
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 36, disp_str);
  Len = 0;
  Len += Format_String(disp_str + Len, "Lon: ");
  if (GPS && GPS->isValid())
  {
    Len += Format_SignDec(disp_str + Len, GPS->Longitude / 6, 8, 5);
    disp_str[Len++] = 0xB0;
  }
  else
    Len += Format_String(disp_str + Len, "----.-----");
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 48, disp_str);
  Len = 0;
  Len += Format_String(disp_str + Len, "Alt: ");
  if (GPS && GPS->isValid())
  {
    int32_t Alt = GPS->Altitude;
    if (Alt >= 0)
      disp_str[Len++] = ' ';
    if (isAltitudeUnitMeter) // display altitude in meters
    {
      Len += Format_SignDec(disp_str + Len, Alt, 1, 1, 1); // [0.1m]
      disp_str[Len++] = 'm';
    }
    else if (isAltitudeUnitFeet) // display altitude in feet
    {
      Alt = (Alt * 336 + 512) >> 10;                   // [0.1m] => [feet]
      Len += Format_SignDec(disp_str + Len, Alt, 1, 0, 1); // [feet]
      disp_str[Len++] = 'f';
      disp_str[Len++] = 't';
    }
    // for( ; Len<14; ) disp_str[Len++]=' ';                          // tail of spaces to cover older printouts
  }
  else
    Len += Format_String(disp_str + Len, "-----.-  ");
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 60, disp_str);
}

void OLED_DrawRF(u8g2_t *OLED, GPS_Position *GPS) // RF
{
  u8g2_SetFont(OLED, u8g2_font_7x13_tf); // 5 lines. 12 pixels/line
  uint8_t Len = 0;
#ifdef WITH_RFM69
  Len += Format_String(disp_str + Len, "RFM69"); // Type of RF chip used
  if (Parameters.RFchipTypeHW)
    disp_str[Len++] = 'H';
  disp_str[Len++] = 'W';
#endif
#ifdef WITH_RFM95
  Len += Format_String(disp_str + Len, "RFM95");
#endif
#ifdef WITH_SX1262
  Len += Format_String(disp_str + Len, "SX1262");
#endif
#ifdef WITH_SX1272
  Len += Format_String(disp_str + Len, "SX1272");
#endif
  disp_str[Len++] = ':';
  Len += Format_UnsDec(disp_str + Len, (int16_t)Parameters.TxPower); // Tx power
  Len += Format_String(disp_str + Len, "dBm");
  disp_str[Len++] = ' ';
  Len += Format_SignDec(disp_str + Len, (int32_t)Parameters.RFchipFreqCorr, 2, 1); // frequency correction
  Len += Format_String(disp_str + Len, "ppm");
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 24, disp_str);
  Len = 0;
  Len += Format_String(disp_str + Len, "Rx:");                    //
  Len += Format_SignDec(disp_str + Len, -5 * TRX.averRSSI, 2, 1); // noise level seen by the receiver
  Len += Format_String(disp_str + Len, "dBm ");
  Len += Format_UnsDec(disp_str + Len, RX_OGN_Count64); // received packet/min
  Len += Format_String(disp_str + Len, "/min");
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 36, disp_str);
  Len = 0;
  //Len += Format_SignDec(disp_str + Len, (int16_t)TRX.chipTemp); // RF chip internal temperature (not calibrated)
  //Len += Format_String(disp_str + Len, "\260C");
  Len += Format_UnsDec(disp_str + Len, (int16_t)TX_OGN_Count); // transmitted packages
  Len += Format_String(disp_str + Len, "pkg RxFIFO:");
  Len += Format_UnsDec(disp_str + Len, RF_RxFIFO.Full()); // how many packets wait in the RX queue
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 48, disp_str);
  // u8g2_DrawStr(OLED, 0, 48, RF_FreqPlan.getPlanName());
  Len = 0;
  Len += Format_String(disp_str + Len, RF_FreqPlan.getPlanName()); // name of the frequency plan
  disp_str[Len++] = ' ';
  Len += Format_UnsDec(disp_str + Len, (uint16_t)(RF_FreqPlan.getCenterFreq() / 100000), 3, 1); // center frequency
  Len += Format_String(disp_str + Len, "MHz");
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 60, disp_str);
}

void OLED_DrawRelay(u8g2_t *OLED, GPS_Position *GPS)
{
  u8g2_SetFont(OLED, u8g2_font_amstrad_cpc_extended_8r);
  uint8_t Len = Format_String(disp_str, "Relay:");
  if (GPS && GPS->Sec >= 0)
  {
    Len += Format_UnsDec(disp_str + Len, (uint16_t)(GPS->Sec), 2);
    disp_str[Len++] = 's';
  }
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 24, disp_str);
  uint8_t LineIdx = 1;
  for (uint8_t Idx = 0; Idx < RelayQueueSize; Idx++)
  {
    OGN_RxPacket<OGN_Packet> *Packet = RelayQueue.Packet + Idx;
    if (Packet->Rank == 0)
      continue;
    uint8_t Len = 0;
    disp_str[Len++] = '0' + Packet->Packet.Header.AddrType;
    disp_str[Len++] = ':';
    Len += Format_Hex(disp_str + Len, Packet->Packet.Header.Address, 6);
    disp_str[Len++] = ' ';
    Len += Format_Hex(disp_str + Len, Packet->Rank);
    disp_str[Len++] = ' ';
    disp_str[Len++] = ':';
    Len += Format_UnsDec(disp_str + Len, Packet->Packet.Position.Time, 2);
    disp_str[Len] = 0;
    u8g2_DrawStr(OLED, 0, (LineIdx + 3) * 8, disp_str);
    LineIdx++;
    if (LineIdx >= 8)
      break;
  }
}

#ifdef WITH_LOOKOUT
void OLED_DrawLookout(u8g2_t *OLED, GPS_Position *GPS)
{
  u8g2_SetFont(OLED, u8g2_font_amstrad_cpc_extended_8r);
  uint8_t Len = Format_String(disp_str, "=> ");
  if (Look.WarnLevel)
  {
    const LookOut_Target *Tgt = Look.Target + Look.WorstTgtIdx;
    Len += Format_Hex(disp_str + Len, Tgt->ID, 7);
    disp_str[Len++] = '/';
    disp_str[Len++] = '0' + Tgt->WarnLevel;
    disp_str[Len++] = ' ';
    Len += Format_UnsDec(disp_str + Len, Tgt->TimeMargin >> 1);
    disp_str[Len++] = 's';
  }
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 24, disp_str);
  uint8_t LineIdx = 1;
  for (uint8_t Idx = 0; Idx < Look.MaxTargets; Idx++)
  {
    const LookOut_Target *Tgt = Look.Target + Idx;
    if (!Tgt->Alloc)
      continue;
    uint8_t Len = 0;
    Len += Format_Hex(disp_str + Len, Tgt->ID, 7);
    disp_str[Len++] = ' ';
    if (Tgt->DistMargin)
    {
      Len += Format_UnsDec(disp_str + Len, Tgt->HorDist >> 1);
      disp_str[Len++] = 'm';
    }
    else
    {
      Len += Format_UnsDec(disp_str + Len, Tgt->TimeMargin >> 1);
      disp_str[Len++] = 's';
      disp_str[Len++] = ' ';
      Len += Format_UnsDec(disp_str + Len, Tgt->MissDist >> 1);
      disp_str[Len++] = 'm';
    }
    disp_str[Len] = 0;
    u8g2_DrawStr(OLED, 0, (LineIdx + 3) * 8, disp_str);
    LineIdx++;
    if (LineIdx >= 8)
      break;
  }
}

void OLED_DrawTrafWarn(u8g2_t *OLED, GPS_Position *GPS)
{
  u8g2_SetFont(OLED, u8g2_font_9x15_tr);
  if (Look.WarnLevel == 0)
    return;
  const LookOut_Target *Tgt = Look.Target + Look.WorstTgtIdx;
  uint8_t Len = 0;
  Len += Format_Hex(disp_str + Len, Tgt->ID, 7); // ID of the target
  disp_str[Len++] = '/';
  disp_str[Len++] = '0' + Tgt->WarnLevel; // warning level
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 30, disp_str);
  Len = 0;
  Len += Format_UnsDec(disp_str + Len, Tgt->TimeMargin * 5, 2, 1); // time-to-impact
  disp_str[Len++] = 's';
  disp_str[Len++] = ' ';
  Len += Format_UnsDec(disp_str + Len, Tgt->MissDist * 5, 2, 1); // miss-distance
  disp_str[Len++] = 'm';
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 45, disp_str);
  Len = 0;
  Len += Format_UnsDec(disp_str + Len, Tgt->getRelHorSpeed() * 5, 2, 1); // horizontal speed
  disp_str[Len++] = 'm';
  disp_str[Len++] = '/';
  disp_str[Len++] = 's';
  disp_str[Len++] = ' ';
  Len += Format_UnsDec(disp_str + Len, Tgt->HorDist * 5, 2, 1); // horizontal distance
  disp_str[Len++] = 'm';
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 60, disp_str);
}
#endif // WITH_LOOKOUT

void OLED_DrawBaro(u8g2_t *OLED, GPS_Position *GPS)
{
  bool isAltitudeUnitMeter = Parameters.AltitudeUnit == 0; // display altitude in meters
  bool isAltitudeUnitFeet = Parameters.AltitudeUnit == 1;  // display altitude in feet

  bool isVarioUnitMPS = Parameters.VarioUnit == 0; // display Vario in m/s
  bool isVarioUnitFPM = Parameters.VarioUnit == 1; // display Vario in fpm

  u8g2_SetFont(OLED, u8g2_font_7x13_tf); // 5 lines, 12 pixels/line
  uint8_t Len = 0;
#ifdef WITH_BMP180
  Len += Format_String(disp_str + Len, "BMP180 ");
#endif
#ifdef WITH_BMP280
  Len += Format_String(disp_str + Len, "BMP280 ");
#endif
#ifdef WITH_BME280
  Len += Format_String(disp_str + Len, "BME280 ");
#endif
#ifdef WITH_MS5607
  Len += Format_String(disp_str + Len, "MS5607 ");
#endif
#ifdef WITH_MS5611
  Len += Format_String(disp_str + Len, "MS5611 ");
#endif
  if (GPS && GPS->hasBaro)
  {
    Len += Format_UnsDec(disp_str + Len, GPS->Pressure / 4, 5, 2);
    Len += Format_String(disp_str + Len, "hPa ");
  }
  else
    Len += Format_String(disp_str + Len, "----.--hPa ");
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 24, disp_str);
  Len = 0;
  if (GPS && GPS->hasBaro)
  {
    if (isAltitudeUnitMeter)
    {
      Len += Format_SignDec(disp_str + Len, GPS->StdAltitude, 5, 1);
      Len += Format_String(disp_str + Len, "m ");
    }
    else if (isAltitudeUnitFeet)
    {
      Len += Format_SignDec(disp_str + Len, (GPS->StdAltitude * 336 + 512) >> 10, 5, 0);
      Len += Format_String(disp_str + Len, "ft ");
    }
    if (isVarioUnitMPS)
    {
      Len += Format_SignDec(disp_str + Len, GPS->ClimbRate, 2, 1);
      Len += Format_String(disp_str + Len, "m/s ");
    }
    else if (isVarioUnitFPM)
    {
      Len += Format_SignDec(disp_str + Len, (GPS->ClimbRate * 5039 + 128) >> 8, 2, 0);
      Len += Format_String(disp_str + Len, "fpm ");
    }
  }
  else
  {
    if (isAltitudeUnitMeter)
      Len += Format_String(disp_str + Len, "-----.-m");
    else if (isAltitudeUnitFeet)
      Len += Format_String(disp_str + Len, "-----ft  ");
    Len += Format_String(disp_str + Len, " --.-m/s ");
  }
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 36, disp_str);
  Len = 0;
  if (GPS && GPS->hasBaro)
  {
    Len += Format_SignDec(disp_str + Len, GPS->Temperature, 2, 1);
    disp_str[Len++] = 0xB0;
    disp_str[Len++] = 'C';
    disp_str[Len++] = ' ';
    Len += Format_SignDec(disp_str + Len, GPS->Humidity, 2, 1);
    disp_str[Len++] = '%';
  }
  else
    Len += Format_String(disp_str + Len, "---.- C --.-% ");
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 48, disp_str);
}

void OLED_DrawBattery(u8g2_t *OLED, GPS_Position *GPS) // draw battery status page
{
	int8_t Cap = -1;
	if(GPS_Status.MAV){ // if we have a mavlink connection
		Cap = MAVLINK_BattCap; // [%] from the drone's telemetry
	} else {
		// no mavlink connection, use local battery volatage
		Cap = BattCapacity(BatteryVoltage >> 8); // [%] est. battery capacity based on the voltage readout
	}
  // u8g2_SetFont(OLED, u8g2_font_battery19_tn);
  // u8g2_DrawGlyph(OLED, 120, 60, '0'+(Cap+10)/20);

  // u8g2_SetFont(OLED, u8g2_font_6x10_tr);
  // u8g2_DrawStr(OLED, 0, 24, disp_str);

  // u8g2_DrawStr(OLED, 0, 24, "Battery");

  u8g2_SetFont(OLED, u8g2_font_9x15_tr);

  if (Cap >= 0)
  {
    strcpy(disp_str, "   %");
    if (Cap >= 100)
      Format_UnsDec(disp_str, (uint8_t)Cap, 3);
    else if (Cap >= 10)
      Format_UnsDec(disp_str + 1, (uint8_t)Cap, 2);
    else
      disp_str[2] = '0' + Cap;
    u8g2_DrawStr(OLED, 16, 32, disp_str);     // print battery est. capacity
    u8g2_DrawFrame(OLED, 12, 20, 42, 14); // draw battery empty box around it
    u8g2_DrawBox(OLED, 8, 23, 4, 8);
  } // and the battery tip

  strcpy(disp_str, " .   V");
  if(GPS_Status.MAV){ // we have a mavlink connection
	  Format_UnsDec(disp_str, MAVLINK_BattVolt, 4, 3);
  } else {
	  Format_UnsDec(disp_str, BatteryVoltage >> 8, 4, 3); // print the battery voltage readout
  }
  u8g2_DrawStr(OLED, 0, 48, disp_str);

  if(GPS_Status.MAV){
	  strcpy(disp_str, "  .  A");
	  Format_UnsDec(disp_str, MAVLINK_BattCurr, 4, 2);
  } else {
	  strcpy(disp_str, "   . mV/min ");                   // print the battery voltage rate
	  Format_SignDec(disp_str, (600 * BatteryVoltageRate + 128) >> 8, 3, 1);
  }
  u8g2_DrawStr(OLED, 0, 60, disp_str);

#ifdef WITH_BQ
  uint8_t Fault = BQ.readFault();          // read fault register
  uint8_t ChargeErr = (Fault >> 4) & 0x3F; // charging fault:
  bool BattErr = Fault & 0x08;             // Battery OVP
  bool OTGerr = Fault & 0x40;              // VBus problem
  uint8_t NTCerr = Fault & 0x03;           //
  uint8_t Status = BQ.readStatus();        // read status register
  uint8_t State = (Status >> 4) & 0x03;    // charging status
  const char *StateName[4] = {"" /* "Not charging" */, "Pre-charge", "Charging", "Full"};
  // u8g2_SetFont(OLED, u8g2_font_amstrad_cpc_extended_8r);
  u8g2_SetFont(OLED, u8g2_font_6x10_tr);
  if (Fault)
  {
    strcpy(disp_str, "Fault: ");
    Format_Hex(disp_str + 7, Fault);
    disp_str[9] = 0;
    u8g2_DrawStr(OLED, 60, 28, disp_str);
  }
  else
    u8g2_DrawStr(OLED, 60, 28, StateName[State]);
    // u8g2_DrawStr(OLED, 0, 60, Status&0x04?"Power-is-Good":"Power-is-not-Good");

/* print BQ registers for debug
  u8g2_SetFont(OLED, u8g2_font_6x10_tr);
  for(uint8_t Reg=0; Reg<=10; Reg++)
  { uint8_t Val = BQ.readReg(Reg);
    Format_Hex(disp_str+3*Reg, Val); disp_str[3*Reg+2]=' '; }
  disp_str[33]=0;
  u8g2_DrawStr(OLED, 0, 60, disp_str+15);
  disp_str[15]=0; u8g2_DrawStr(OLED, 0, 50, disp_str);
*/
#endif

#ifdef WITH_AXP
  uint16_t InpCurr = AXP.readBatteryInpCurrent(); // [mA]
  uint16_t OutCurr = AXP.readBatteryOutCurrent(); // [mA]
  int16_t Current = InpCurr - OutCurr;
  // uint8_t Charging = Current>0;
  u8g2_SetFont(OLED, u8g2_font_6x10_tr);
  strcpy(disp_str, "    mA ");
  Format_SignDec(disp_str, Current, 3);
  u8g2_DrawStr(OLED, 60, 28, disp_str);
#endif
}

void OLED_DrawStatusBar(u8g2_t *OLED, GPS_Position *GPS) // status bar on top of the OLED
{
  static bool Odd = 0;
  int8_t Cap = -1;
  if(GPS_Status.MAV){
	  Cap = MAVLINK_BattCap; // [%]
  } else {
	  Cap = BattCapacity(BatteryVoltage >> 8); // [%] est. battery capacity
  }
  uint8_t BattLev = (Cap + 10) / 20; // [0..5] convert to display scale
  uint8_t Charging = 0;              // charging or not changing ?
#ifdef WITH_BQ
  uint8_t Status = BQ.readStatus();
  Charging = (Status >> 4) & 0x03;
#endif
#ifdef WITH_AXP
  uint16_t InpCurr = AXP.readBatteryInpCurrent(); // [mA]
  uint16_t OutCurr = AXP.readBatteryOutCurrent(); // [mA]
  int16_t Current = InpCurr - OutCurr;
  Charging = Current > 0;
#endif
#if defined(WITH_BQ) || defined(WITH_AXP)
  static uint8_t DispLev = 0;
  if (Charging == 1 || Charging == 2)
  {
    DispLev++;
    if (DispLev > 5)
      DispLev = BattLev ? BattLev - 1 : 0;
  }
  else
  {
    DispLev = BattLev;
  }
#else
  uint8_t &DispLev = BattLev;
#endif
  if (Cap >= 0)
  {
    if (BattLev == 0 && !Charging && Odd) // when battery is empty, then flash it at 0.5Hz
    {
    }    // thus here avoid printing the battery symbol for flashing effect
    else // print the battery symbol with DispLev
    {
      u8g2_SetFont(OLED, u8g2_font_battery19_tn);
      u8g2_SetFontDirection(OLED, 3);
      u8g2_DrawGlyph(OLED, 20, 10, '0' + DispLev);
      u8g2_SetFontDirection(OLED, 0);
    }
    Odd = !Odd;
  }
  if (SD_isMounted())
  {
    u8g2_SetFont(OLED, u8g2_font_twelvedings_t_all);
    u8g2_DrawGlyph(OLED, 24, 12, 0x73);
  }
  if (BT_SPP_isConnected())
  {
    u8g2_SetFont(OLED, u8g2_font_open_iconic_all_1x_t);
    u8g2_DrawGlyph(OLED, 36, 11, 0x5E);
  } // 0x4A

   if(Stratux_isConnected())
   { u8g2_SetFont(OLED, u8g2_font_open_iconic_all_1x_t);
     u8g2_DrawGlyph(OLED, 43, 11, 0x50);
   }
  if (WIFI_isConnected())
  {
    u8g2_SetFont(OLED, u8g2_font_open_iconic_all_1x_t);
    u8g2_DrawGlyph(OLED, 43, 11, 0x119);
  } // 0x50
  // blink if connected
  if (WIFI_isAP() && (Odd || !AP_isClientConnected()))
  {
    u8g2_SetFont(OLED, u8g2_font_open_iconic_all_1x_t);
    u8g2_DrawGlyph(OLED, 43, 11, 0xF8);
  } // 0x50

  // u8g2_SetFont(OLED, u8g2_font_5x7_tr);
  // u8g2_SetFont(OLED, u8g2_font_5x8_tr);
  static uint8_t gps_sat_display = 0;
  u8g2_SetFont(OLED, u8g2_font_6x12_tr);
  // strcpy(disp_str, "[   %] --sat --:--");
  strcpy(disp_str, "??sat ??:??Z");
  if (GPS && GPS->isTimeValid())
  {
    Format_UnsDec(disp_str + 6, (uint16_t)GPS->Hour, 2, 0);
    disp_str[8] = ':';
    Format_UnsDec(disp_str + 9, (uint16_t)GPS->Min, 2, 0);
  }
  else{
	// display time since boot in seconds
	uint32_t t = TimeSync_Time();
    Format_UnsDec(disp_str+6, t, 5);
    //Format_String(disp_str + 6, "--:--");
  }
  if (GPS)
  {
    if (gps_sat_display)
    {
      Format_UnsDec(disp_str, (uint16_t)GPS->Satellites, 2);
      memcpy(disp_str + 2, "sat", 3);
    }
    else
    {
      Format_UnsDec(disp_str, (uint16_t)(GPS_SatSNR + 2) / 4, 2);
      memcpy(disp_str + 2, "dB ", 3);
    }
  } else {
    Format_String(disp_str, "--sat");
  }
  u8g2_DrawStr(OLED, 52, 10, disp_str);
  gps_sat_display++;
  if (gps_sat_display >= 3)
    gps_sat_display = 0;
}

void OLED_DrawSystem(u8g2_t *OLED, GPS_Position *GPS)
{
  u8g2_SetFont(OLED, u8g2_font_7x13_tf); // 5 lines, 12 pixels/line
  uint8_t Len = 0;
  if(GPS_Status.MAV){
	  Len += Format_String(disp_str + Len, "MAVLINK ");
  } else if(GPS_Status.UBX){
	  Len += Format_String(disp_str + Len, "UBX ");
  } else if(GPS_Status.NMEA){
	  Len += Format_String(disp_str + Len, "GPS ");
  }

  Len += Format_UnsDec(disp_str + Len, GPS_getBaudRate(), 1);
  Len += Format_String(disp_str + Len, "bps");
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 24, disp_str);

  Len = 0;
#ifdef WITH_RFM69
  Len += Format_String(disp_str + Len, "RFM69 v"); // Type of RF chip used
  if (Parameters.RFchipTypeHW)
    disp_str[Len++] = 'H';
  disp_str[Len++] = 'W';
#endif
#ifdef WITH_RFM95
  Len += Format_String(disp_str + Len, "RFM95 v");
#endif
#ifdef WITH_SX1262
  Len += Format_String(disp_str + Len, "SX1262 v");
#endif
#ifdef WITH_SX1272
  Len += Format_String(disp_str + Len, "SX1272 v");
#endif
  Len += Format_Hex(disp_str + Len, TRX.chipVer);
  disp_str[Len++] = ' ';
#ifdef WITH_SX1262
  disp_str[Len++] = '0';
  disp_str[Len++] = 'x';
  Len += Format_Hex(disp_str + Len, TRX.getStatus());
#else
  Len += Format_SignDec(disp_str + Len, (int16_t)TRX.chipTemp);
  disp_str[Len++] = 0xB0;
  disp_str[Len++] = 'C';
#endif
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 36, disp_str);

  Len = 0;
#ifdef WITH_BMP180
  Len += Format_String(disp_str + Len, "BMP180 0x");
  Len += Format_Hex(disp_str + Len, Baro.ADDR);
#endif
#ifdef WITH_BMP280
  Len += Format_String(disp_str + Len, "BMP280 0x");
  Len += Format_Hex(disp_str + Len, Baro.ADDR);
#endif
#ifdef WITH_BME280
  Len += Format_String(disp_str + Len, "BME280 0x");
  Len += Format_Hex(disp_str + Len, Baro.ADDR);
#endif
#ifdef WITH_MS5607
  Len += Format_String(disp_str + Len, "MS5607 0x");
  Len += Format_Hex(disp_str + Len, Baro.ADDR);
#endif
#ifdef WITH_MS5611
  Len += Format_String(disp_str + Len, "MS5611 0x");
  Len += Format_Hex(disp_str + Len, Baro.ADDR);
#endif
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 48, disp_str);

#ifdef WITH_SPIFFS
  Len = 0;
  Len += Format_String(disp_str + Len, "SPIFFS ");
  size_t Total, Used;
  if (SPIFFS_Info(Total, Used) == 0) // get the SPIFFS usage summary
  {
    Len += Format_UnsDec(disp_str + Len, (Total - Used) / 1024);
    Len += Format_String(disp_str + Len, "/");
    Len += Format_UnsDec(disp_str + Len, Total / 1024);
    Len += Format_String(disp_str + Len, "kB");
  }
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 60, disp_str);
#endif
  /*
  #ifdef WITH_SD
    Len=0;
    Len+=Format_String(disp_str+Len, "SD ");
    if(SD_isMounted())
    { Len+=Format_UnsDec(disp_str+Len, (uint32_t)SD_getSectors());
      disp_str[Len++]='x';
      Len+=Format_UnsDec(disp_str+Len, (uint32_t)SD_getSectorSize()*5/512, 2, 1);
      Len+=Format_String(disp_str+Len, "KB"); }
    else
    { Len+=Format_String(disp_str+Len, "none"); }
    disp_str[Len]=0;
    u8g2_DrawStr(OLED, 0, 60, disp_str);
  #endif
  */
}

void OLED_DrawID(u8g2_t *OLED, GPS_Position *GPS)
{
  u8g2_SetFont(OLED, u8g2_font_9x15_tr);
  Parameters.Print(disp_str);
  disp_str[10] = 0;
  u8g2_DrawStr(OLED, 26, 28, disp_str);
  // u8g2_SetFont(OLED, u8g2_font_10x20_tr);
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);
  u8g2_DrawStr(OLED, 0, 27, "ID:");
  if (Parameters.Pilot[0] || Parameters.Reg[0])
  {
    strcpy(disp_str, "Reg: ");
    strcat(disp_str, Parameters.Reg);
    u8g2_DrawStr(OLED, 0, 54, disp_str);
    strcpy(disp_str, "Pilot: ");
    strcat(disp_str, Parameters.Pilot);
    u8g2_DrawStr(OLED, 0, 42, disp_str);
  }
  else
  {
#ifdef WITH_FollowMe
    u8g2_DrawStr(OLED, 15, 44, "FollowMe868");
    u8g2_DrawStr(OLED, 20, 56, "by AVIONIX");
#endif
  }
  u8g2_SetFont(OLED, u8g2_font_5x8_tr);
  if(strlen(GIT_TAG)>0){
	  u8g2_DrawStr(OLED, 96, 62, GIT_TAG);
  } else {
	  u8g2_DrawStr(OLED, 88, 62, GIT_REV);
  }
}

void OLED_DrawAltitudeAndSpeed(u8g2_t *OLED, GPS_Position *GPS)
{
  uint8_t Len;

  bool isAltitudeUnitMeter = Parameters.AltitudeUnit == 0; // display altitude in meters
  bool isAltitudeUnitFeet = Parameters.AltitudeUnit == 1;  // display altitude in feet

  bool isSpeedUnitKMH = Parameters.SpeedUnit == 0;  // display Speed in km/h
  bool isSpeedUnitKnot = Parameters.SpeedUnit == 1; // display Speed in knot

  bool isVarioUnitMPS = Parameters.VarioUnit == 0; // display Vario in m/s
  bool isVarioUnitFPM = Parameters.VarioUnit == 1; // display Vario in fpm

  // Standard Pressure Altitude
  if (GPS && (GPS->hasBaro || GPS->isValid())) // if GPS has lock or just the pressure data
  {
    int32_t Alt = GPS->Altitude; // [0.1m/s] take GPS (geometrical) altitude
    if (GPS->hasBaro)
      Alt = GPS->StdAltitude; // but if pressure sensor is there then replace with pressure altitude
    if (isAltitudeUnitMeter)
      Alt = (Alt + 5) / 10; // [0.1m] => [m]     // or to meters
    else if (isAltitudeUnitFeet)
      Alt = (Alt * 336 + 512) >> 10; // [0.1m] => [feet]  // convert to feet
    Len = Format_SignDec(disp_str, Alt, 1, 0, 1);
  } // print altitude into the string
  else
    Len = Format_String(disp_str, "----"); // if altitude not available then print place holders
  disp_str[Len] = 0;
  u8g2_SetFont(OLED, u8g2_font_fub20_tr);                // relatively big font
  uint8_t Altitude_width = u8g2_GetStrWidth(OLED, disp_str); // how wide the string would be on the OLED
  u8g2_DrawStr(OLED, 60 - Altitude_width, 40, disp_str);     // print the string

  u8g2_SetFont(OLED, u8g2_font_9x15_tr); // smaller font
  if (isAltitudeUnitMeter)
    u8g2_DrawStr(OLED, 64, 40, "m");
  else if (isAltitudeUnitFeet)
    u8g2_DrawStr(OLED, 62, 40, "ft"); // print units

  u8g2_SetFont(OLED, u8g2_font_fub17_tr);
  if (GPS && GPS->isValid())
  {
    uint16_t Heading = (GPS->Heading + 5) / 10;
    if (Heading >= 360)
      Heading -= 360;
    Len = Format_UnsDec(disp_str, Heading, 3);
  }
  else
    Len = Format_String(disp_str, "---");
  disp_str[Len] = 0;
  uint8_t Track_width = u8g2_GetStrWidth(OLED, disp_str);
  u8g2_DrawStr(OLED, 118 - Track_width, 40, disp_str);

  u8g2_SetFont(OLED, u8g2_font_6x12_tr); // small font
  u8g2_DrawStr(OLED, 122, 28, "o");      // degree sign

  if (GPS && (GPS->hasBaro || GPS->isValid())) // if GPS has lock or just the pressure data
  {
    int16_t vario_value = GPS->ClimbRate; // [0.1m/s] climb rate
    if (isVarioUnitFPM)
      vario_value = (vario_value * 5039 + 128) >> 8; // [0.1m/s] => [feet per meter]
    if (vario_value < 0)
    {
      vario_value = (-vario_value);
      const int minus_width = 10;
      const int minus_height = 17;
      static unsigned char minus_bits[] =
          {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0xfe, 0x01, 0xfe, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      u8g2_DrawXBM(OLED, 0, 47, minus_width, minus_height, minus_bits);
    }
    else
    {
      const int plus_width = 10;
      const int plus_height = 17;
      static unsigned char plus_bits[] =
          {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x00,
           0x30, 0x00, 0xfe, 0x01, 0xfe, 0x01, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      u8g2_DrawXBM(OLED, 0, 47, plus_width, plus_height, plus_bits);
    }
    if (isVarioUnitMPS)
      Len = Format_UnsDec(disp_str, vario_value, 2, 1);
    if (isVarioUnitFPM)
      Len = Format_UnsDec(disp_str, vario_value, 2, 0);
  }
  else
    Len = Format_String(disp_str, "-.-");
  disp_str[Len] = 0;
  u8g2_SetFont(OLED, u8g2_font_fub17_tr);
  uint8_t Vario_width = u8g2_GetStrWidth(OLED, disp_str);
  u8g2_DrawStr(OLED, 54 - Vario_width, 64, disp_str);

  if (isVarioUnitMPS)
  {
    const int ms_width = 7;
    const int ms_height = 17;
    static unsigned char ms_bits[] =
        {0x00, 0x00, 0x16, 0x2a, 0x2a, 0x2a, 0x2a, 0x00, 0x7f, 0x00, 0x1c, 0x22,
         0x02, 0x1c, 0x20, 0x22, 0x1c};
    u8g2_DrawXBM(OLED, 58, 47, ms_width, ms_height, ms_bits);
  }
  else if (isVarioUnitFPM)
  {
    const int ms_width = 7;
    const int ms_height = 17;
    static unsigned char ms_bits[] =
        {0x84, 0xa2, 0xf7, 0xa2, 0xa2, 0xa2, 0xc2, 0x80, 0xff, 0x80, 0x00, 0x16, 0x2a, 0x2a, 0x2a, 0x2a, 0x00};
    u8g2_DrawXBM(OLED, 58, 47, ms_width, ms_height, ms_bits);
  }

  // Speed
  if (GPS && GPS->isValid())
  {
    uint16_t speed = (GPS->Speed * 9 + 12) / 25;
    if (isSpeedUnitKnot)
      speed = (GPS->Speed * 199 + 512) >> 10;
    Len = Format_UnsDec(disp_str, speed, 1, 0);
  }
  else
    Len = Format_String(disp_str, "--");
  disp_str[Len] = 0;
  u8g2_SetFont(OLED, u8g2_font_fub17_tr);
  uint8_t Speed_width = u8g2_GetStrWidth(OLED, disp_str);
  u8g2_DrawStr(OLED, 114 - Speed_width, 64, disp_str);

  if (isSpeedUnitKMH)
  {
    const int kmh_width = 10;
    const int kmh_height = 17;
    static unsigned char kmh_bits[] =
        {0x01, 0x00, 0x01, 0x00, 0x69, 0x01, 0xa5, 0x02, 0xa3, 0x02, 0xa5, 0x02,
         0xa9, 0x02, 0x00, 0x00, 0xff, 0x03, 0x00, 0x00, 0x08, 0x00, 0x08, 0x00,
         0x08, 0x00, 0x38, 0x00, 0x48, 0x00, 0x48, 0x00, 0x48, 0x00};
    u8g2_DrawXBM(OLED, 118, 47, kmh_width, kmh_height, kmh_bits);
  }
  else if (isSpeedUnitKnot)
  {
    const int kmh_width = 10;
    const int kmh_height = 17;
    static unsigned char kmh_bits[] =
        {0x00, 0xfc, 0x00, 0xfc, 0x00, 0xfc, 0x00, 0xfc, 0x01, 0xfc, 0x01, 0xfc,
         0xe9, 0xfc, 0x25, 0xfd, 0x23, 0xfd, 0x25, 0xfd, 0x29, 0xfd, 0x00, 0xfc,
         0x00, 0xfc, 0x00, 0xfc, 0x00, 0xfc, 0x00, 0xfc, 0x00, 0xfc};
    u8g2_DrawXBM(OLED, 118, 47, kmh_width, kmh_height, kmh_bits);
  }
}

void OLED_DrawFlight(u8g2_t *OLED, GPS_Position *GPS) // draw flight status page
{
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);
  u8g2_DrawStr(OLED, 0, 28, "Flight");
}

void OLED_DrawLoRaWAN(u8g2_t *OLED, GPS_Position *GPS) // draw LoRaWAN status page
{
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);
#ifndef WITH_LORAWAN
  u8g2_DrawStr(OLED, 0, 28, "LoRaWAN: -OFF-");
#endif
#ifdef WITH_LORAWAN
  const char *StateName[4] = {"Not-Joined", "Join-Req", "+Joined+", "PktSend"};
  int Len = Format_String(disp_str, "LoRaWAN: ");
  if (WANdev.State == 2)
    Len += Format_Hex(disp_str + Len, WANdev.DevAddr);
  else if (WANdev.State <= 3)
    Len += Format_String(disp_str + Len, StateName[WANdev.State]);
  else
    Len += Format_Hex(disp_str + Len, WANdev.State);
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 0, 24, disp_str);

  // if(WANdev.State<2)
  // { Len =Format_HexBytes(disp_str, WANdev.AppKey  , 8); disp_str[Len]=0; u8g2_DrawStr(OLED, 12, 36, disp_str);
  //   Len =Format_HexBytes(disp_str, WANdev.AppKey+8, 8); disp_str[Len]=0; u8g2_DrawStr(OLED, 12, 48, disp_str);
  //   u8g2_SetFont(OLED, u8g2_font_open_iconic_all_1x_t); u8g2_DrawGlyph(OLED, 1, 36, 0xC1); u8g2_DrawGlyph(OLED, 1, 48, 0xC1); }
  // else
  if (WANdev.State >= 2)
  {
    Len = Format_String(disp_str, "Up: ");
    Len += Format_Hex(disp_str + Len, (uint16_t)WANdev.UpCount);
    Len += Format_String(disp_str + Len, "  Dn: ");
    Len += Format_Hex(disp_str + Len, (uint16_t)WANdev.DnCount);
    disp_str[Len] = 0;
    u8g2_DrawStr(OLED, 0, 36, disp_str);

    Len = Format_String(disp_str, "Rx:");
    Len += Format_SignDec(disp_str + Len, (int16_t)WANdev.RxRSSI, 3);
    Len += Format_String(disp_str + Len, "dBm ");
    Len += Format_SignDec(disp_str + Len, ((int16_t)WANdev.RxSNR * 10 + 2) >> 2, 2, 1);
    Len += Format_String(disp_str + Len, "dB");
    disp_str[Len] = 0;
    u8g2_DrawStr(OLED, 0, 48, disp_str);
  }
  Len = Format_HexBytes(disp_str, WANdev.AppKey, 2);
  disp_str[Len++] = '.';
  disp_str[Len++] = '.';
  Len += Format_Hex(disp_str + Len, WANdev.AppKey[15]);
  disp_str[Len] = 0;
  u8g2_DrawStr(OLED, 72, 60, disp_str);
  u8g2_SetFont(OLED, u8g2_font_open_iconic_all_1x_t);
  u8g2_DrawGlyph(OLED, 61, 60, 0xC1);

  // if(WANdev.State>=2) { }
/*
  Len=0;
  for(int Idx=0; Idx<16; Idx++)
  { Len+=Format_Hex(disp_str+Len, WANdev.AppKey[Idx]); }
  disp_str[Len]=0;
  u8g2_SetFont(OLED, u8g2_font_5x8_tr);
  u8g2_DrawStr(OLED, 0, 48, disp_str);
*/
#endif // WITH_LORAWAN
}

#endif

// ========================================================================================================================
