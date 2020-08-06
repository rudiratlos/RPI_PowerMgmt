# Powermanagement for RPI

ATtiny45/85 as Powermanagement chip for RPI

functions:

- press push button for 1 second  to boot 
- press push button for 3 seconds to initiate RPI shutdown, after 12 seconds switch off 5V power
- status BiColor LED (red/green)  

Video:

[![video](./img/0.jpg)](//www.youtube.com/watch?v=cuHMuZBUYbw "powermanagement chip in action")

shows startup, shutdown and switch off. Installed in a prototype. 

Schematic:

![schematic](./img/RPI_PowerMgmt_schematic.jpg)

use .hex file directly to flash ATtiny45/85
avrdude.sh shellscript uses avrdude as flashing utility.

![flashing](./img/avrdude_Programmer_on_RPI_ATtiny45.jpg)
 
If you want to add functionality, you can modify .pas source code and compile it with mikropascal to generate .hex flash file


# Install shutdownbutton as a service

~~~bash
apt update
apt install raspi-gpio
#
cp ./shutdownbutton /usr/local/sbin
sudo chmod +x /usr/local/sbin/shutdownbutton 
#
/usr/local/sbin/shutdownbutton install
#
# finally reboot
shutdown -h now
~~~
