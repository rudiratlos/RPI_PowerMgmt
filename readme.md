# Powermanagement for RPI

ATtiny85 as Powermanagement chip for RPI

functions:

- press push button for 1 second  to switchON relais and rpi will boot 
- press push button for 3 seconds to initiate clean shutdown, after 10 seconds Relais will switchOFF 5V power
- service script 'shutdownbutton', simulates a 3 second button press, if OS command shutdown -h was executed (no push button was pressed)
- status BiColor LED (red/green)  

If you want to add functionality, you can modify .ino source code and compile it with Arduino IDE to generate new .hex file


Video:

[![video](./img/0.jpg)](//www.youtube.com/watch?v=cuHMuZBUYbw "powermanagement chip in action")

video shows switchON, boot, ... , shutdown and switchOFF. (Installed in a prototype device).
 

Schematic:

![schematic](./img/RPI_PowerMgmt_schematic.jpg)


# Install shutdownbutton as a service

~~~bash
apt -qy install raspi-gpio 
#
cd /usr/local/sbin
wget -N https://raw.githubusercontent.com/rudiratlos/RPI_PowerMgmt/master/shutdownbutton
chmod +x shutdownbutton
#
/usr/local/sbin/shutdownbutton install
#
# finally reboot
reboot
~~~


# Install avrdude

~~~bash
apt update
apt -qy install avrdude
#
cd /usr/local/sbin
wget -N https://raw.githubusercontent.com/rudiratlos/RPI_PowerMgmt/master/RPI_ShutDownReboot.hex
wget -N https://raw.githubusercontent.com/rudiratlos/RPI_PowerMgmt/master/avrdude.sh
chmod +x avrdude.sh
~~~


# Burn ATtiny85 with avrdude

connect a stk500V2 compatible AVR programmer on rpi USB port

~~~bash
avrdude.sh
~~~ 

use .hex file directly to flash ATtiny85
avrdude.sh shellscript uses avrdude as flashing utility.

![flashing](./img/avrdude_Programmer_on_RPI_ATtiny45.jpg)
