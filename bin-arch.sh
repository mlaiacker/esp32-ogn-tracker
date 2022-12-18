
cd utils
make serial_dump
make read_log
cd ..
RELEASE=heltec-v3-ogn-tracker-bin
tar cvzf $RELEASE.tgz flash_ACM0.sh esptool.py flash_COM?.bat esptool.py \
build/partition_table/partition-table.bin build/bootloader/bootloader.bin build/esp32-ogn-tracker.bin \
utils/read_log utils/serial_dump main/config.h

zip -u $RELEASE.zip flash_ACM0.sh esptool.py flash_COM?.bat esptool.py \
build/partition_table/partition-table.bin build/bootloader/bootloader.bin build/esp32-ogn-tracker.bin \
utils/read_log utils/serial_dump main/config.h
