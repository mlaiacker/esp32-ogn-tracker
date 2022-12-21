#pragma once

#define DEFAULT_AcftType       13          // [0..15] default aircraft-type: Powered Aircraft
#define DEFAULT_GeoidSepar     40          // [m]
#define DEFAULT_CONbaud    115200
#define DEFAULT_PPSdelay      100
#define DEFAULT_FreqPlan        1
#define DEFAULT_DispPage        3          // Fab501 Page to Display After Boot or Reset	
#define WIFI_ADDRESS_IP1	192	 // 192.168.7.1 for IP Address
#define WIFI_ADDRESS_IP2	168
#define WIFI_ADDRESS_IP3	7
#define WIFI_ADDRESS_IP4	1
#define WIFI_ADDRESS_GW1	0	// 0.0.0.0 for Gateway
#define WIFI_ADDRESS_GW2	0
#define WIFI_ADDRESS_GW3	0
#define WIFI_ADDRESS_GW4	0
#define WIFI_ADDRESS_MK1	255	// 255.255.255.0 for Mask
#define WIFI_ADDRESS_MK2	255
#define WIFI_ADDRESS_MK3	255
#define WIFI_ADDRESS_MK4	0

// #define WITH_HELTEC                        // HELTEC module: PCB LED on GPI025
#define WITH_HELTEC_V2                     // HELTEC module v2
//#define WITH_HELTEC_V3                     // HELTEC module v3
// #define WITH_TTGO                          // TTGO module: PCB LED on GPIO2, GPIO25 free to use as DAC2 output
// #define WITH_TBEAM                          // T-Beam module
//#define WITH_TBEAM_V10                      // T-Beam module
// #define WITH_M5_JACEK                         // JACEK M5 ESP32 OGN-Tracker
// #define WITH_FollowMe                         // by Avionix

// #define WITH_ILI9341                        // 320x240 M5stack
// #define WITH_ST7789                         // IPS 240x240 ST7789
// #define WITH_TFT_LCD                       // TFT LCD
// #define WITH_OLED                          // OLED display on the I2C: some TTGO modules are without OLED display
// #define WITH_OLED2                         // 2nd OLED display, I2C address next higher
//#define WITH_U8G2_OLED                     // I2C OLED through the U8g2 library
//#define WITH_U8G2_SH1106                   // correct controller for the bigger OLED
// #define WITH_U8G2_FLIP                     // flip the OLED screen (rotate by 180deg)

//#define WITH_RFM95                         // RF chip selection:  both HELTEC and TTGO use sx1276 which is same as RFM95
//#define WITH_SX1262                         // SX1262 Support used in heltec V3

// #define WITH_SLEEP                         // with software sleep mode controlled by the long-press on the button

//#define WITH_AXP                           // with AXP192 power controller (T-BEAM V1.0)
// #define WITH_BQ                            // with BQ24295  power controller (FollowMe)

// #define WITH_LED_RX
// #define WITH_LED_TX

// #define WITH_GPS_ENABLE                    // use GPS_ENABLE control line to turn the GPS ON/OFF
//#define WITH_GPS_PPS                       // use the PPS signal from GPS for precise time-sync.
//#define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode

//#define WITH_GPS_UBX                       // GPS understands UBX
//#define WITH_GPS_MTK                       // GPS understands MTK
//#define WITH_GPS_SRF
//#define WITH_MAVLINK

//#define WITH_GPS_UBX_PASS                  // to pass directly UBX packets to/from GPS
//#define WITH_GPS_NMEA_PASS                  // to pass directly NMEA to/from GPS

// #define WITH_BMP180                        // BMP180 pressure sensor
// #define WITH_BMP280                        // BMP280 pressure sensor
//#define WITH_BME280                        // BMP280 with humidity (but still works with BMP280)
// #define WITH_MS5607                        // MS5607 pressure sensor
// #define WITH_MS5611                        // MS5611 pressure sensor

// #define WITH_BMX055                        // BMX055 magnetic and IMU sensor

//#define WITH_LORAWAN                       // LoRaWAN connectivity
//#define WITH_FANET                         // FANET transmission and reception
//#define WITH_PAW			   // Add PAW transmission

//#define WITH_PFLAA                         // PFLAU and PFLAA for compatibility with XCsoar and LK8000
//#define WITH_POGNT
// #define WITH_GDL90
// #define WITH_PGAV5
//#define WITH_LOOKOUT

//#define WITH_SKYDEMON			//Adapt NMEA Output for SKYDEMON

//#define WITH_CONFIG                        // interpret the console input: $POGNS to change parameters

//#define WITH_BEEPER                        // with digital buzzer
// #define WITH_SOUND                         // with analog sound produced by DAC on pin 25

// #define WITH_KNOB
// #define WITH_VARIO

// #define WITH_SD                            // use the SD card in SPI mode and FAT file system
//#define WITH_SPIFFS                        // use SPIFFS file system in Flash
// #define WITH_SPIFFS_FAT
//#define WITH_LOG                           // log own positions and other received to SPIFFS
// #define WITH_SDLOG                         // log own position and other data to uSD card

//#define WITH_STRATUX
//#define WITH_BT_SPP                        // Bluetooth serial port for smartphone/tablet link
//#define WITH_BLE_SPP
//#define WITH_WIFI                          // attempt to connect to the wifi router for uploading the log files
//#define WITH_AP                            // Open Access Point MOde
//#define WITH_HTTP                           // Open Web Interface

// #define WITH_ENCRYPT                       // Encrypt (optionally) the position

//#define DEBUG_PRINT


#if defined(WITH_HELTEC_V3)
#define WITH_SX1262
#define WITH_U8G2_OLED

#define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode
#define WITH_GPS_UBX                       // GPS understands UBX
#define WITH_GPS_MTK                       // GPS understands MTK
#define WITH_GPS_SRF
#define PIN_GPS_ENA	GPIO_NUM_36
#define GPS_ON_LEVEL	0
#define WITH_MAVLINK

#define WITH_PFLAA                         // PFLAU and PFLAA for compatibility with XCsoar and LK8000
#define WITH_LOOKOUT

#define WITH_FANET                         // FANET transmission and reception

#define WITH_BME280                        // BMP280 with humidity (but still works with BMP280)

#define WITH_CONFIG

#define WITH_SPIFFS                        // use SPIFFS file system in Flash
#define WITH_LOG                           // log own positions and other received to SPIFFS
//#define WITH_BLE_SPP
#define WITH_AP                            // Open Access Point MOde
#define WITH_HTTP                           // Open Web Interface

#elif defined(WITH_HELTEC_V2)
#define WITH_RFM95                         // RF chip selection:  both HELTEC and TTGO use sx1276 which is same as RFM95
#define WITH_U8G2_OLED

#define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode
#define WITH_GPS_UBX                       // GPS understands UBX
#define WITH_GPS_MTK                       // GPS understands MTK
#define WITH_GPS_SRF
#define PIN_GPS_ENA	GPIO_NUM_21 // actice low
#define GPS_ON_LEVEL	0
#define WITH_MAVLINK

#define WITH_PFLAA                         // PFLAU and PFLAA for compatibility with XCsoar and LK8000
#define WITH_LOOKOUT

#define WITH_SPIFFS                        // use SPIFFS file system in Flash
#define WITH_LOG                           // log own positions and other received to SPIFFS
//#define WITH_BT_SPP
#define WITH_AP                            // Open Access Point MOde
#define WITH_HTTP                           // Open Web Interface
#define WITH_BME280

#elif defined(WITH_TBEAM_V10)

#define WITH_RFM95                         // RF chip selection:  both HELTEC and TTGO use sx1276 which is same as RFM95

#define WITH_AXP                           // with AXP192 power controller (T-BEAM V1.0)
#define WITH_GPS_PPS                       // use the PPS signal from GPS for precise time-sync.
#define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode

#define WITH_GPS_UBX                       // GPS understands UBX
#define WITH_BME280                        // BMP280 with humidity (but still works with BMP280)

#define WITH_PFLAA                         // PFLAU and PFLAA for compatibility with XCsoar and LK8000
#define WITH_LOOKOUT

#define WITH_CONFIG                        // interpret the console input: $POGNS to change parameters

#define WITH_BEEPER                        // with digital buzzer
#define WITH_LORAWAN

#define WITH_AP                            // WiFi Access Point: can work together with BT_SPP
#define WITH_AP_BUTTON                     // only starts when button pressed at sartup
#define WITH_BT_SPP                        // Bluetooth serial port for smartphone/tablet link: can work together with WiFi Access point
#define WITH_APRS                          // alpha-code: attempt to connect to the wifi router for uploading the log files to APRS

#define WITH_HTTP                          // HTTP server, works with AP dna should work with Stratux as well

#define WITH_SPIFFS_FAT
#define WITH_SPIFFS                        // use SPIFFS file system in Flash
#define WITH_LOG                           // log own positions and other received to SPIFFS and possibly to uSD

#if defined(WITH_STRATUX) || defined(WITH_APRS) || defined(WITH_AP)
#define WITH_WIFI
#endif
#elif defined(WITH_TBEAM)                          // T-Beam module
#define WITH_U8G2_OLED                     // I2C OLED through the U8g2 library
#define WITH_RFM95                         // RF chip selection:  both HELTEC and TTGO use sx1276 which is same as RFM95

#define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode
#define WITH_GPS_UBX                       // GPS understands UBX
#define WITH_BME280                        // BMP280 with humidity (but still works with BMP280)

#define WITH_PFLAA                         // PFLAU and PFLAA for compatibility with XCsoar and LK8000
#define WITH_LOOKOUT
#define WITH_CONFIG                        // interpret the console input: $POGNS to change parameters

#define WITH_BEEPER                        // with digital buzzer
#define WITH_SPIFFS                        // use SPIFFS file system in Flash
#define WITH_LOG                           // log own positions and other received to SPIFFS and possibly to uSD

#define WITH_BT_SPP                        // Bluetooth serial port for smartphone/tablet link
#define WITH_ENCRYPT                       // Encrypt (optionally) the position

#elif defined(WITH_FollowMe)                         // by Avionix

#define WITH_U8G2_OLED                     // I2C OLED through the U8g2 library
#define WITH_U8G2_SH1106                   // the bigger OLED controller
#define WITH_U8G2_FLIP                     // flip the OLED screen
#define WITH_RFM95                         // RF chip selection: FollowMe, HELTEC and TTGO use sx1276 which is same as RFM95 module
#define WITH_BQ                            // with BQ24295  power controller (new FollowMe)

#define WITH_GPS_ENABLE                    // use GPS_ENABLE control line to turn the GPS ON/OFF
#define WITH_GPS_PPS                       // use the PPS signal from GPS for precise time-sync.
#define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode

#define WITH_GPS_MTK                       // GPS understands MTK
#define WITH_BME280                        // BMP280 with humidity (still works with BMP280)

#define WITH_PFLAA                         // $PFLAU and $PFLAA for compatibility with XCsoar and LK8000
#define WITH_LOOKOUT
#define WITH_CONFIG                        // interpret the console input: $POGNS to change parameters
#define WITH_LORAWAN                       // LoRaWAN TTN connectivity
#define WITH_AP                            // create WiFi Access Point for data, setup and log files
#define WITH_AP_BUTTON                     // activate Access Point only when button pressed at startup
#define WITH_HTTP                          // HTTP server, works with AP and Stratux
#define WITH_BT_SPP                        // (classic) Bluetooth serial port for smartphone/tablet link

#define WITH_SD                            // use the SD card in SPI mode and FAT file system
#define WITH_SPIFFS_FAT
#define WITH_SPIFFS                        // use SPIFFS file system in Flash
#define WITH_LOG                           // log own positions and other received to SPIFFS and possibly to uSD
#define WITH_SDLOG                         //

#if defined(WITH_STRATUX) || defined(WITH_APRS) || defined(WITH_AP)
#define WITH_WIFI
#endif

#else
#error "you need to define a board"
#endif
