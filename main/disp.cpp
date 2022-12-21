#include <stdio.h>

#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_system.h"
// #include "esp_sleep.h"

#include "hal.h"

#include "sens.h"
#include "rf.h"
#include "ctrl.h"
#include "proc.h"
#include "log.h"

#include "gps.h"
#include "ubx.h"
#include "timesync.h"
#include "format.h"

#include "disp_oled.h"
#include "disp_lcd.h"

// #define DEBUG_PRINT
#ifndef DISPLAY_TIMEOUT
#define DISPLAY_TIMEOUT	45 // should be seconds because we only update display if time or gps changed
#endif

#ifdef WITH_U8G2_OLED
const uint8_t DISP_Pages = 12;
static uint8_t DISP_Page = DEFAULT_DispPage; // Fab501 before 0
#endif
#if defined(WITH_ST7789) || defined(WITH_ILI9341)
const uint8_t DISP_Pages = 9;
static uint8_t DISP_Page = DEFAULT_DispPage; // Fab501 before 0
#endif

extern "C" void vTaskDISP(void *pvParameters) {
	int no_activity_counter = 0; // turn display off if button is not pressed for awhile
#ifdef WITH_U8G2_OLED
	u8g2_ClearBuffer(&U8G2_OLED);
	OLED_DrawLogo(&U8G2_OLED); // draw logo
	u8g2_SendBuffer(&U8G2_OLED);
	vTaskDelay(2000); // allow 2sec for the user to see the logo
	DISP_Page = Parameters.InitialPage;
#endif
#if defined(WITH_ST7789) || defined(WITH_ILI9341)
  // LCD_Start();
  LCD_LogoPage_Draw();
  LCD_SetBacklightLevel(6); // backlight level
#endif

	uint32_t PrevTime = 0;
	GPS_Position *PrevGPS = 0;
	for (;;) //
			{
#if defined(WITH_ST7789) || defined(WITH_ILI9341)
    if (PowerMode == 0)
    {
      vTaskDelay(200);
      LCD_SetBacklightLevel(0);
      continue;
    }
#endif
#ifdef WITH_U8G2_OLED
		if (PowerMode == 0) {
			vTaskDelay(200); /* u8g2_SetPowerSave(&U8G2_OLED, 1); */
			continue;
		}
#endif
		vTaskDelay(10); //

		bool PageChange = 0;
		uint8_t Key = 0;
		KeyBuffer.Read(Key); // read key pressed from the buffer

		uint32_t Time = TimeSync_Time();
		bool TimeChange = Time != PrevTime; // did time change = a new second ?
		uint32_t Sec = (Time - 1) % 60;
		GPS_Position *GPS = GPS_getPosition(Sec);
		//GPS_Position *GPS = GPS_getPosition();

		bool GPSchange = GPS != PrevGPS; // did GPS data change = new position ?
		if (Key == 0) {
			if (!(TimeChange || GPSchange))
				continue;
			PrevTime = Time;
			PrevGPS = GPS;
		}

#if defined(WITH_U8G2_OLED) || defined(WITH_ST7789) || defined(WITH_ILI9341)
		if (Key) {
			if (no_activity_counter >= DISPLAY_TIMEOUT) {
				no_activity_counter = -1; // signal to reactivate the display
				PowerMode = 2;
				// but dont change page
			} else {
				no_activity_counter = 0;
				if (Key & 0x40) {
					if (DISP_Page == 0)
						DISP_Page = DISP_Pages - 1;
					else
						DISP_Page--;
				} else {
					DISP_Page++;
					if (DISP_Page >= DISP_Pages)
						DISP_Page = 0;
				}
			}
			PageChange = 1;
		} else {
			if (no_activity_counter < DISPLAY_TIMEOUT) {
				no_activity_counter++;
				if(no_activity_counter == DISPLAY_TIMEOUT){
					PowerMode = 1;
				}
			} else {
				continue;
			}
		}
#endif

#if defined(WITH_ST7789) || defined(WITH_ILI9341)
    static uint8_t LCD_Backlight = 8 * 16 + 8;
    const uint8_t LCD_BacklightLimit = 4 * 16 + 8; // lower limit for the backlight
#ifdef WITH_AXP
    uint16_t Vbus = AXP.readVbusVoltage(); // external supply (USB) voltage
    if (PageChange || Vbus >= 4000)
      LCD_Backlight = 8 * 16 + 8; // high backlight on page-change or when charging
#else
    if (PageChange)
      LCD_Backlight = 8 * 16 + 8; // high backlight on page change
#endif
    switch (DISP_Page)
    {
    case 0:
      if (PageChange)
        LCD_LogoPage_Draw(Time, GPS); // logo with basic information
      LCD_LogoPage_Update(Time, GPS, TimeChange, GPSchange);
      break;
    case 1:
      if (PageChange)
        LCD_GPSpage_Draw(Time, GPS); // GPS data
      LCD_GPSpage_Update(Time, GPS, TimeChange, GPSchange);
      break;
    case 2:
      if (PageChange)
        LCD_RFpage_Draw(Time, GPS); // RF data
      LCD_RFpage_Update(Time, GPS, TimeChange, GPSchange);
      break;
    case 3:
      if (PageChange)
        LCD_BaroPage_Draw(Time, GPS); // Baro data
      LCD_BaroPage_Update(Time, GPS, TimeChange, GPSchange);
      break;
    case 4:
      if (PageChange)
        LCD_BattPage_Draw(Time, GPS); // Battery
      LCD_BattPage_Update(Time, GPS, TimeChange, GPSchange);
      break;
    case 5:
      if (PageChange)
        LCD_ParmPage_Draw(Time, GPS); // ID and parameters
      LCD_ParmPage_Update(Time, GPS, TimeChange, GPSchange);
      break;
    case 6:
      if (PageChange)
        LCD_RelayPage_Draw(Time, GPS); // RELAY list
      LCD_RelayPage_Update(Time, GPS, TimeChange, GPSchange);
      break;
    case 7:
      if (PageChange)
        LCD_LookPage_Draw(Time, GPS); // Look targets
      LCD_LookPage_Update(Time, GPS, TimeChange, GPSchange);
      break;
    case 8:
      if (PageChange)
        LCD_SysPage_Draw(Time, GPS); // System overview
      LCD_SysPage_Update(Time, GPS, TimeChange, GPSchange);
      break;
    }
    if (TimeChange) // on each new second
    {
      LCD_SetBacklightLevel(LCD_Backlight / 16); //
      if (LCD_Backlight > LCD_BacklightLimit)
        LCD_Backlight--;
    }  // reduce backlight a lttle if above minimum
#endif // if WITH_ST7789 or WITH_ILI9341

#ifdef WITH_OLED
    // if(Button_SleepRequest)
    // { OLED_DisplayON(0); }
    // else
    {
      esp_err_t StatErr = ESP_OK;
      esp_err_t PosErr = ESP_OK;
      if (TimeChange)
      {
        StatErr = OLED_DisplayStatus(Time, 0);
      }
      if (GPSchange)
      {
        PosErr = OLED_DisplayPosition(GPS, 2);
      }
    }
#endif // WITH_OLED

#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    if (TimeChange)
    {
      Format_String(CONS_UART_Write, "TimeChange: ");
      // Format_SignDec(CONS_UART_Write, StatErr);
      Format_String(CONS_UART_Write, "\n");
    }
    if (GPSchange)
    {
      Format_String(CONS_UART_Write, "GPSchange: ");
      // Format_SignDec(CONS_UART_Write, PosErr);
      Format_String(CONS_UART_Write, "\n");
    }
    xSemaphoreGive(CONS_Mutex);
#endif

#ifdef WITH_U8G2_OLED
		if (no_activity_counter >= DISPLAY_TIMEOUT) {
			u8g2_SetPowerSave(&U8G2_OLED, 1);
		} else if (PageChange || (GPS ? GPSchange : TimeChange)) {
			if (no_activity_counter == -1) {
				u8g2_SetPowerSave(&U8G2_OLED, 0);
			}
			u8g2_ClearBuffer(&U8G2_OLED);
			// #ifdef WITH_LOOKOUT
			//       if(Look.WarnLevel)
			//       { OLED_DrawTrafWarn(&U8G2_OLED, GPS); }
			//       else
			// #endif
			// skip not enabled pages
			if (Parameters.PageMask) {
				while (((Parameters.PageMask >> DISP_Page) & 1) == 0) {
					DISP_Page++;
					if (DISP_Page >= DISP_Pages)
						DISP_Page = 0;
				}
			}

#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "DISP_Page: ");
      Format_Hex(CONS_UART_Write, DISP_Page);
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex);
#endif

			{
				switch (DISP_Page) {
				case 0:
					OLED_DrawID(&U8G2_OLED, GPS);
					break;
				case 1:
					OLED_DrawGPS(&U8G2_OLED, GPS);
					break;
				case 2:
					OLED_DrawRF(&U8G2_OLED, GPS);
					break;
				case 3:
					OLED_DrawBaro(&U8G2_OLED, GPS);
					break;
				case 4:
					OLED_DrawSystem(&U8G2_OLED, GPS);
					break;
				case 5:
					OLED_DrawBattery(&U8G2_OLED, GPS);
					break;
				case 6:
					OLED_DrawAltitudeAndSpeed(&U8G2_OLED, GPS);
					break;
				case 7:
					OLED_DrawRelay(&U8G2_OLED, GPS);
					break;
				case 8:
					OLED_DrawFlight(&U8G2_OLED, GPS);
					break;
				case 9:
					OLED_DrawLoRaWAN(&U8G2_OLED, GPS);
					break;
#ifdef WITH_LOOKOUT
				case 10:
					OLED_DrawLookout(&U8G2_OLED, GPS);
					break;
				case 11:
					OLED_DrawTrafWarn(&U8G2_OLED, GPS);
					break;
#endif
				}
			}
			// if ( DISP_Page != 6 )
			OLED_DrawStatusBar(&U8G2_OLED, GPS);
			u8g2_SendBuffer(&U8G2_OLED);
		}
#endif
	}
}
