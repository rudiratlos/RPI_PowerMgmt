#!/bin/bash
#
# http://www.engbedded.com/fusecalc
#http://www.joewest.de/usb-isp-programmer-fuer-atmel-avr-unter-ubuntu-einrichten/
#apt-get install avrdude
#
echo "ATTiny45 align to socket bottom"
#
echo hfuse burn
avrdude -F -p attiny45 -P /dev/ttyACM0 -c stk500V2 -U lfuse:w:0xe2:m -U hfuse:w:0x5f:m -U flash:w:RPI_ShutDownReboot.hex
#
