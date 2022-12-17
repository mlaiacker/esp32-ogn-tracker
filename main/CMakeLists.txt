            

idf_component_register(SRCS "stratux.cpp" "aprs.cpp" "main.cpp"
							"aero.cpp"
							"atmosphere.cpp"
							"bitcount.cpp"
							"ctrl.cpp"
							"disp.cpp"
							"disp_lcd.cpp"
							"disp_oled.cpp"
							"format.cpp"
							"gps.cpp"
							"hal.cpp"
							"intmath.cpp"
							"ldpc.cpp"
							"log.cpp"
							"main.cpp"
							"nmea.cpp"
							"ognconv.cpp"
							"proc.cpp"
							"rf.cpp"
							"sens.cpp"
							"st7789.cpp"
							"timesync.cpp"
                            "wifi.cpp"
                            "ap.cpp"
                            "http.cpp"
                            "bt.cpp"                  
                            ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
						EMBED_FILES "OGN_logo_240x240.jpg"
                       INCLUDE_DIRS "."
                       INCLUDE_DIRS "mavlink/common"
					   INCLUDE_DIRS "../components")

ADD_CUSTOM_COMMAND(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
           ${CMAKE_CURRENT_BINARY_DIR}/_version.cpp
    COMMAND ${CMAKE_COMMAND} -P
            ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake)
