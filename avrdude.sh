#!/bin/bash
#
# http://www.joewest.de/usb-isp-programmer-fuer-atmel-avr-unter-ubuntu-einrichten/
# https://www.engbedded.com/fusecalc/
#
# apt update
# apt -qy install avrdude
#
echo "ATTiny85 align to socket bottom"
#
echo "hfuse burn reset disabled (RSTDISBL), ATtiny45 can not be reprogrammed (or use a high voltage programmer). Use it for burning a production chip"
avrdude -F -p attiny85 -P /dev/ttyACM0 -c stk500V2 -U lfuse:w:0xe2:m -U hfuse:w:0x5f:m -U flash:w:/usr/local/sbin/RPI_ShutDownReboot.hex
#avrdude -F -p attiny85 -P usb -c stk500V2 -U lfuse:w:0xe2:m -U hfuse:w:0x5f:m -U flash:w:/usr/local/sbin/RPI_ShutDownReboot.hex
#
#echo "hfuse burn reset enabled (no RELAIS output on Pin1 PB5), just for testing, chip can be reused"
#avrdude -F -p attiny85 -P /dev/ttyACM0 -c stk500V2 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U flash:w:/usr/local/sbin/RPI_ShutDownReboot.hex
#
# https://groenholdt.net/Microcontroller/programming-(unbricking)-the-digispark.html
#echo "Burn micronucleus to a digispark, just using a rpi"
#wget -N https://raw.githubusercontent.com/micronucleus/micronucleus/master/firmware/releases/t85_default.hex
#wget -N https://github.com/micronucleus/micronucleus/raw/80419704f68bf0783c5de63a6a4b9d89b45235c7/firmware/releases/micronucleus-1.06.hex
#avrdude -p attiny85 -C /usr/local/sbin/avrdude_gpio.conf -c rpi -U lfuse:w:0xF1:m -U hfuse:w:0x5F:m -U flash:w:/usr/local/sbin/t85_default.hex:i
