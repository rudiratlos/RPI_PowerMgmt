/* PumpShutdownReboot Version 5.1 Arduino IDE Sketch
Shutdown/Reboot function for RPI, with Voltage observation and
   Status signalling with BiColor LED with 3 Pins (red/green/GND).
   Use 330Ohm Resistor between LED Cathode and GND
   - 5V comes from Powersupply  
   - 3.3V comes from Raspberry PI Header Pin 1, voltage will be analyzed
   - Push-Button shorts to GND and is connected to ATTiny PB3 and a RPI GPIO IN
     (do not use external PullUps, both will/should use internal PullUps).
     1sec: Relais will be switched ON
     3sec: Relais will be switched OFF after 10sec
   - Relais OUT (use Transistor driver) which will 
     supply 5V to RPI (backpower) on rpi HDR-P1 Pins 2&4

Functions:
    - LED dimming RED:    Relais is OFF, 3.3V not in allowed range -> 0V
    - LED steady  RED:    Relais is ON,  3.3V not in allowed range
    - LED blink   RED:    Under-/Overvoltage/Overheat detection
                            Relais state and overvoltage condition:
                              is ON:  Relais will switch off immediatly
                              is OFF: Relais will not switch on
    - LED steady  GREEN:  Relais is ON: 3.3V in allowed range
    - LED blink   GREEN:  Relais in ON: during Bootprocess

     ------------- ATtiny85 (5.5V Version) --------------
                            +---v---+
           (Relais Out) PB5 |1     8| VCC (5V from Powersupply)
           (/Button IN) PB3 |2     7| PB2 (AnalogIN 3.3V RPI HDR-P1/Pin1)
    (LED Anode Red OUT) PB4 |3     6| PB1 (OUT PWM backlight LED TFT display)
                        GND |4     5| PB0 (OUT LED Anode Green)
                            +-------+
     -------------------------------------------------------

     Fuses ( http://www.engbedded.com/fusecalc ):
     CKDIV8=0        (disable) do not Divide clock by 8 internally (lfuse)
     RSTDISBL=1      (enable)  using PB5 as I/O-Pin (hfuse)
     !!!! Chip can not be reprogrammed -> reset FUSE by HighVoltage programmer

   avrdude -F -p attiny85 -P usb -c stk500V2 -U lfuse:w:0xe2:m -U hfuse:w:0x5f:m -U flash:w:<file2flash>.hex

   install ATTiny85 Tool:
   https://www.kollino.de/arduino/attiny-support-unter-arduino-1-6-installieren/
   http://drazzy.com/package_drazzy.com_index.json

   use Arduino IDE to generate .hex flash file to program ATtiny 
   
   Tools Setup Tab (BoardManagement):
   Board: ATtiny25/45/85 (No bootloader)
   Chip:  ATtiny85
   Clock: 8 MHz (internal)
*/

//PINdefinition
  const byte LEDgreenPIN = PB0; // Pin5
  const byte PWMpin =      PB1; // Pin6 
  const byte ADCpin =      PB2; // Pin7
  const byte ButtonPIN =   PB3; // Pin2
  const byte LEDredPIN =   PB4; // Pin3 
  const byte RelaisPIN =   PB5; // Pin1
 
  const long SerBaud=      300; // 300, 600, 1200, 2400, 4800, 9600, 19200

//ADC-Values
  const float BGtolD=     0.75; // ScaleDown, due to ATtiny85 BandGap tolerance
  const float BGtolU=     1.25; // ScaleUp,   due to ATtiny85 BandGap tolerance
  const float ShottkyFwdV=0.283; // FwdVoltage Shottky BAT43: 0.283V, reduces VREF
  const float VCCmaxTiny=(6.000*BGtolU)-ShottkyFwdV; // 6V max pwr for ATtiny
  const float VCCmax=    (5.500*BGtolU)-ShottkyFwdV; //5.767  max for rpi pwr input
  const float VCCokVolt=  5.250-ShottkyFwdV; // optimal supply voltage
  const float VCCmin=    (4.750*BGtolD)-ShottkyFwdV;   // 3.992 min expected
  
//Regulator   NCP1117-3.3V: // min. Vout: 3.235 // max. Vout: 3.365
  const float V33max=     3.365*BGtolU; // 3.7015
  const float V33min=     3.235*BGtolD; // 2.9115
  const float V33okVolt=  V33min+((V33max-V33min)/2);

//https://www.mikrocontroller.net/articles/LED-Fading
  const byte  PWMtabmax=31;
  const byte  PWMtab[PWMtabmax+1] = {
      0,  1,  2,  2,  2,  3,  3,  4,  5,  6,   7,   8,  10,  11,  13,  16,
     19, 23, 27, 32, 38, 45, 54, 64, 76, 91, 108, 128, 152, 181, 215, 255 };
  const byte  pwmsteady=    PWMtabmax;     // idx: pwm value for LED steady
  const byte  pwmlow=       26;            // idx: lower intensity level (dimming)
  const byte  FadeCntMax=   4;             // delay cnt between loading next pwm level
  const byte  PWMdflt=      244;           // default PWM level for display backlight ctrl
                                           // (1.8TFT VCC:5.55V 244:3.37V@58mA 255:3.44V@60mA)
//time counter divideable by 'wtim'
  const byte  wtim=            10;         // debounce time (msec). Processor runs @ 8MHz
  const word  cnt100ms=       100 / wtim;
  const word  cnt250ms=       250 / wtim;
  const word  cnt1sec=       1000 / wtim;
  const word  cntBootUp=      990 / wtim;  // 1sec Button has to be pressed: Bootstrap
  const word  cntShutDwn=    2980 / wtim;  // 3sec Button has to be pressed: ShutDown
//rpi_hal has 3000msec / 1000msec and 7msec debounce time
  const word  cntoffmax=    10000 / wtim;  //  10sec waittime for RPI shutdown-/boot-process
  const word  cntPwrENAmin=     3 * wtim;  // minimum 36msec PwrEna
  const word  cntPwrENAmax=  2980 / wtim;  // maximum  3 sec PwrEna
  const word  cntBootTime1=  3000 / wtim;  // suppose 3sec rpi BootTime phase 1
  const word  cntBootTime2=  7000 / wtim;  // suppose 7sec rpi BootTime phase 2
  const word  cntGetVREF=   60000 / wtim;  // get VREF every 60sec
  const word  cntresetVfail= cntoffmax;
  const word  cntLEDoffmax=  1500 / wtim;  // 1.5sec offtime @ dimming and idx =0

  const word  cntblinkmaxc=   500 / wtim;  // counter for 1Hz
  const word  cntblinkAmaxc=  250 / wtim;  // counter for 2Hz (ALARM)

//LED definitions
  const byte  LEDoff=0, LEDon=1, LEDdim=2; // mode/state
  
//rpiSTATEs:
  const byte  rpiNEW=0,      rpiOFF=1,        rpiSwitchOff=2, rpiSwitchOn=3;
  const byte  rpiDOboot=4,   rpiDOboot2=5,    rpiON=6;
  const byte  rpiDOreboot=7, rpiDOshutdown=8, rpiAlarmOFF=9;

  const byte  eeprom_pattern=0xf0;

  bool  PWMup=true,BUTpres=false,Blink=false;
  int   VCCstat=0,VCCstatLast=-2,V33stat=0,V33statLast=-2,TEMPstat=0,TEMPstatLast=-2;
  byte  rpiSTATE=rpiOFF,rpiSTATElast=rpiNEW,LEDstate=rpiSTATE,LED=LEDredPIN;
  byte  RelaisStat=LOW,PWMidx=0,PWMmax=PWMtabmax,FadeCnt=FadeCntMax,PWMoutLvl=PWMdflt;
  word  BGraw,BG2raw,V33raw;
  word  cntPwrENA=0,BlinkCnt=0,BlinkMax=cntblinkmaxc,ERRcnt=0,LEDoffCnt=0,gcnt=0;
  float VREF=VCCokVolt,V33=V33okVolt,BGd=1.0;

#include <EEPROM.h>
#include <SoftwareSerial.h> 
SoftwareSerial swSer(ButtonPIN, LEDgreenPIN); // RX, TX  

/*void blk(byte cnt)  {
  for (byte _i= 1; _i<=cnt; _i++) {
    digitalWrite(LEDgreenPIN, HIGH);
    delay(500);
    digitalWrite(LEDgreenPIN, LOW); 
    delay(500);
  }
}*/

void EEPROMsave() {
  EEPROM.write(0, eeprom_pattern);
  EEPROM.write(2, PWMoutLvl);
}

void EEPROMread() {
  if (EEPROM.read(0) != eeprom_pattern) { 
    PWMoutLvl=PWMdflt;
    EEPROMsave(); 
  } else PWMoutLvl= EEPROM.read(2);
}

void InitVARs() {
  LEDoffCnt= 0; FadeCnt= 0;  gcnt=   0;
  cntPwrENA= 0; BlinkCnt= 0; ERRcnt= 0;
}

void Input_Read() { BUTpres=(digitalRead(ButtonPIN)==LOW); } // active low

void SetRelais(byte relstat) {
  InitVARs();
  digitalWrite(RelaisPIN, relstat);  
  if (RelaisStat!=relstat) delay(500); // let environment stabilize
  RelaisStat= relstat;
}

void SetPWMout(byte PWMval) { analogWrite(PWMpin, PWMval); }

void TinyInit() {
  swSer.begin(SerBaud);
  swSer.setTimeout(250);

  pinMode(ButtonPIN,   INPUT_PULLUP);
  
  pinMode(LEDgreenPIN, OUTPUT);
  pinMode(LEDredPIN,   OUTPUT);
  
//pinMode(ADCpin,      INPUT);
 
  pinMode(RelaisPIN,   OUTPUT); 
  RelaisStat=HIGH; // force delay
  SetRelais(LOW);  // switch off

  pinMode(PWMpin,      OUTPUT);
  SetPWMout(0);
  EEPROMread();
}

void WriteLED(byte lednr, byte PWMval) {
  switch (lednr) {
    case LEDredPIN:
           analogWrite(LEDredPIN, PWMval);      
         break; 
    case LEDgreenPIN:
           if (PWMval>=0x80) 
                digitalWrite(LEDgreenPIN, HIGH); 
           else digitalWrite(LEDgreenPIN, LOW);
         break;
  } // switch
}

void SetLED(byte lednr, byte PWMmaxidx) {
  LED= lednr; PWMmax= PWMmaxidx;
  switch (lednr) { // switch other LED off  
    case LEDredPIN:   WriteLED(LEDgreenPIN, PWMtab[0]); break;      
    case LEDgreenPIN: WriteLED(LEDredPIN,   PWMtab[0]); break;
  } // switch
  WriteLED(lednr, PWMtab[0]); // set LED to PWM level
}

void SetPWMidx(byte mode) {
  switch (mode) {
    case     LEDoff: // steady OFF
               PWMidx= 0;      PWMup= true;  FadeCnt= 0; 
             break; 
    case     LEDon:  // steady ON
               PWMidx= PWMmax; PWMup= false; FadeCnt= 0; 
             break; 
    default: // dimming
              if (FadeCnt>=FadeCntMax) {
                FadeCnt= 0;
                if (PWMup) { // up
                  if (PWMidx>=PWMmax) {
                    PWMup= false; 
                    PWMidx=PWMmax-1;
                  } else PWMidx++;
                } else { // down
                  if (PWMidx<=0) {
                    if ((RelaisStat==HIGH) || 
                       (LEDoffCnt >= cntLEDoffmax)) {
                      PWMup=  true; 
                      PWMidx= 1; 
                      LEDoffCnt=   0;
                    } else LEDoffCnt++;
                  } else PWMidx--;
                }
              } else FadeCnt++;
            break;
  } // switch

  if (FadeCnt==0) WriteLED(LED, PWMtab[PWMidx]);
}

void StartBlink(bool alarm) {
  Blink= true;
  BlinkCnt= 0; 
  if (alarm)
       BlinkMax= cntblinkAmaxc;
  else BlinkMax= cntblinkmaxc;
}

void StopBlink() {
  StartBlink(false);
  Blink= false;
}

void InitPWM() {
//SetLED(LEDredPIN,pwmlow);
  StopBlink();
  PWMidx=  0;
  PWMup=   true;
}

void SetLED4status() {
  if (Blink) {
    if (BlinkCnt <= (BlinkMax/2))
         SetPWMidx(LEDon);
    else SetPWMidx(LEDoff);

    if (BlinkCnt>=BlinkMax)
         BlinkCnt= 0; 
    else BlinkCnt++;  
    
  } else { // dimming or steady on
    if (RelaisStat==HIGH)
         SetPWMidx(LEDon);  // steady ON 
    else SetPWMidx(LEDdim); // RPI in OFF Status (LED PWM mode)
  }
}

void SetupLED() {
  if ((V33stat>0) || (VCCstat!=0) || (TEMPstat>0)) { 
 // OverVoltage of VCC or V33 from rpi  OR UnderVoltage of VCC
    SetLED(LEDredPIN,pwmsteady);
    StartBlink(true);
  } else {
    StopBlink();
    if (RelaisStat==HIGH) { // Relais ON
      if (( rpiSTATE==rpiDOshutdown) || (rpiSTATE==rpiDOreboot)) {
        if (rpiSTATE==rpiDOshutdown) 
             SetLED(LEDredPIN,  pwmsteady);
        else SetLED(LEDgreenPIN,pwmsteady); 
        StartBlink(false);
      } else {
        if (V33stat==0)
             SetLED(LEDgreenPIN,pwmsteady);       // Voltage in OK range
        else SetLED(LEDredPIN,  pwmsteady);       // Voltage not OK
        
        if (rpiSTATE==rpiDOboot)  StartBlink(true);
        if (rpiSTATE==rpiDOboot2) StartBlink(false);
      }
    } 
    else { // Relais OFF
      if (V33stat==0)
            SetLED(LEDgreenPIN,  pwmlow);  // Voltage in OK range
       else SetLED(LEDredPIN,    pwmlow);  // Voltage outside
    }
  }
  SetLED4status();
}

void AD_ConvOK() {
  ADCSRA |= _BV(ADSC);    // Start conversion
  while (ADCSRA & _BV(ADSC));
}

void AD_ReadRAW(byte loops, word &adc) {
// loops < 60 (10Bit ADC)
word _adc=0;
  adc=0;
  for (byte _i= 1; _i<=loops; _i++) {
    AD_ConvOK();
    _adc=  ADCL;
    _adc= (ADCH << 8) | _adc; // Read 10 Bit ADC data
    adc= adc + _adc;
  }
  if (loops>0) adc= adc / loops;
}

void AD_Read(byte mode) {
const byte _loops=4;
word _raw=0;
  switch (mode) {
    case 0x8f: // ADC is connected to internal temperature sensor
           AD_ReadRAW(_loops,_raw);
           TEMPstat= 0;
           if (_raw > 385) TEMPstat= 1; 

           if (TEMPstatLast != TEMPstat) LEDstate= rpiNEW; 
           TEMPstatLast=TEMPstat;
         break;
    case 0x0c: // ADC is connected to internal 1.10V -> determine VCC/Vref
           AD_ReadRAW(_loops,BGraw);
           if ((BGraw > 0) && (BGd > 0))
                VREF= BGd * 1126.4 / BGraw;   // 1126.4/BGraw
           else VREF= VCCokVolt;        // simulate ok value

           VCCstat= 0;
           if (VREF > VCCmax) VCCstat= 1;
           if (VREF < VCCmin) VCCstat=-1;
           
           if (VCCstatLast != VCCstat) LEDstate= rpiNEW; 
           VCCstatLast=VCCstat;

           if (VCCstat!=0) VREF=VCCokVolt; // for other mode required
         break;
    case 0x9c: // ADC is connected to internal 1.10V, Ref is 2.56V -> BandGap delta
           AD_ReadRAW(_loops,BG2raw);
           if ((VREF > 3.0) && (BG2raw > 0))
                BGd= 440 / BG2raw; // ((1.1 / 2.56) * 1024.0) / BG2raw;
           else BGd= 1.0;
         break;
    default: // ADC connected to PB2, meassure 3.3V from rpi
           if (RelaisStat==HIGH) {
             AD_ReadRAW(_loops,V33raw);
             V33= V33raw * VREF / 1024.0;
           } else V33= 0; // RelaisOFF: PB2 is open, we get unreliable values

           V33stat= 0;
           if (V33 > V33max) V33stat= 1;
           if (V33 < V33min) V33stat=-1; 
           
           if (V33statLast != V33stat) LEDstate= rpiNEW; 
           V33statLast=V33stat;
    break;
  } // switch
}

void AD_Init(byte mode) {
  switch (mode) {
    case 0x9c: 
           ADMUX= ((12 << MUX0)  | // use VBG(MUX[3:0]: 1100)
                   (1  << REFS2) |
                   (2  << REFS0)); // Ref is 2.56V
         break;
    case 0x0c: 
           ADMUX=  (12 << MUX0);   // use VBG (MUX[3:0]: 1100)
         break;                    // Ref is VCC
    case 0x8f: 
           ADMUX= ((15 << MUX0) |  // use TEMP(MUX[3:0]: 1111)
                   (1  << REFS1)); // Ref is 1.1V
         break;
    default: 
           ADMUX=  (1  << MUX0);  // use single ended Input ADC1 (PB2 Pin7)
         break;
  } // switch                     // set ref. voltage (Vref) to VCC

  ADCSRA= ((7 << ADPS0) |         // set prescaler (64 6:110 / 128 7:111)
           (1 << ADEN));          // enable ADC

  delay(2);                       // wait for Vref to settle
//AD_Read(mode);                  // drop reading
}

void GetVREF() {
byte _state;
  _state=LEDstate; 
// meassure VCC and prepare for further ADC readings
  AD_Init(0x0c); // determine VCC (Value in Vref)
  AD_Read(0x0c);
  AD_Init(0x9c); // determine BandGap delta 1.1 vs. 2.56V
  AD_Read(0x9c); 
  AD_Init(0x8f); // determine chip temperature
  AD_Read(0x8f);
  AD_Init(0x00); // connect PB2 to DAC, determine Vin (3.3V from rpi)
  AD_Read(0x00);
  if (_state!=LEDstate) SetupLED();
  LEDstate=_state;
}

void DoRelais(bool stat) {
  if (stat) { // switch Relais OFF
    
    if (rpiSTATE == rpiAlarmOFF) 
         gcnt= cntoffmax; // prevent loop, immediate Relais OFF
    else gcnt= 0;
    
    rpiSTATE= rpiDOshutdown;
      
    while (gcnt < cntoffmax) { // RPI shutdown process (>10sec)
      if ((gcnt % cnt1sec)==0) GetVREF(); // every 1 sec
      if ((VCCstat<=0) && (TEMPstat<=0)) {
        SetLED4status();
        delay(wtim);   
      } else gcnt= cntoffmax;
      gcnt++;
    }

    SetPWMout(0);
 
    SetRelais(LOW);
    rpiSTATE= rpiOFF;
    
  } else { // switch Relais ON
    GetVREF();
    if ((VCCstat!=1) && (TEMPstat!=1)) {
//    prevent Relais ON
      SetRelais(HIGH);
      rpiSTATE= rpiDOboot;
      EEPROMread();
    }
  }
  GetVREF();
  SetupLED();
}

bool NextSTATE(byte state, byte nextstate, byte &setstate, word SetOnCount, word &counter) {
bool _ok=false;
  if (setstate==state) {
    if (counter>=SetOnCount) {
      counter= 0;
      setstate=nextstate;
      _ok= true;
     } else counter++;
  }
  return(_ok);
}

void STATEmachine() {
  if ((VCCstat>0) || (V33stat>0) || (TEMPstat>0)) {
//  VCC or RPI 3.3V overvoltage -> Relais OFF
    if (RelaisStat==HIGH) rpiSTATE=rpiAlarmOFF;
  } else {
    if (BUTpres) {
      NextSTATE(rpiOFF,      rpiSwitchOn,   rpiSTATE, cntBootUp,    cntPwrENA);
      NextSTATE(rpiON,       rpiDOshutdown, rpiSTATE, cntShutDwn,   cntPwrENA);
    } else {
 //   during Boot phases
      NextSTATE(rpiDOboot,   rpiDOboot2,    rpiSTATE, cntBootTime1, cntPwrENA); //3sec
      NextSTATE(rpiDOboot2,  rpiON,         rpiSTATE, cntBootTime2, cntPwrENA); //7sec
      
      NextSTATE(rpiDOreboot, rpiDOboot,     rpiSTATE, cntPwrENAmax, cntPwrENA);  //3sec->reset
      if ((rpiSTATE==rpiOFF) || (rpiSTATE==rpiON))                  cntPwrENA=0; //cncl BUTpres
    }
  }

  if (rpiSTATElast!=rpiSTATE) SetupLED();
  rpiSTATElast= rpiSTATE;  

  switch (rpiSTATE) {
    case rpiSwitchOn:   DoRelais(false); break;
    case rpiDOshutdown: DoRelais(true);  break;
    case rpiAlarmOFF:   DoRelais(true);  break;
    case rpiDOboot2:    SetPWMout(PWMoutLvl); break;
  } // switch
}

void setup() {
  TinyInit();
}

void loop() {
    Input_Read();
    
    if (gcnt>=cnt1sec) {
      if (!BUTpres) {
        GetVREF();
      }
      gcnt= 0;
    } else {
       if ((gcnt % cnt100ms)==0) {
         if (swSer.peek()!=-1) {
           String str = swSer.readStringUntil('\n');
           str.toUpperCase(); str.trim();

           if (str.indexOf("SHUT")>=0) rpiSTATE= rpiDOshutdown;
           if (str.indexOf("BOOT")>=0) rpiSTATE= rpiSwitchOn;
           if (str.indexOf("ESAV")>=0) EEPROMsave();
           
           if (str.indexOf("DIM")>=0) {
             str.replace("DIM",""); str.trim();
               if (str.length()>0) {
                 PWMoutLvl= str.toInt();
                 SetPWMout(PWMoutLvl);  
               } 
           }
         }
       }
    }
    
    STATEmachine();
    SetLED4status();
    gcnt++;
    delay(wtim);
}

