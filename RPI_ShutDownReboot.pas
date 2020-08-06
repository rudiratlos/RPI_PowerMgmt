program PumpShutdownReboot;  // Version 3.0
(* Shutdown/Reboot function for RPI, with Voltage observation and
   Status signalling with BiColor LED with 3 Pins (red/green/GND).
   Use 330Ohm Resistor between LED Kathode and GND
   - 5V comes from Powersupply
   - 3.3V comes from Raspberry PI Header Pin1, voltage will be analyzed
   - Push-Button shorts to GND and is connected to ATTiny and a RPI GPIO IN
            (do not use external PullUps, both will/should use internal PullUps).
            Relais will be switched ON, if there is a GND signal for longer than 1sec
            Relais will be switched OFF after 20sec, if there is a GND signal for longer than 3secs
   - Relais OUT (use Transistor driver) which will supply RPI backpowered on Pins 2&4 with 5V
   - RPI PowerEnable is a RPI GPIO Out, which signals ATtiny that RPI is powered on.
         If not used, leave this pin unconnected (internal PullUp is active)

   Functions:
     - LED dimming RED:         Relais is OFF, 3.3V not in allowed range -> 0V
     - LED steady  RED:         Relais is ON,  3.3V not in allowed range
     - LED blink   RED:         3.3V Overvoltage detected or
                                    Relais will switch off in 20sec
     - LED dimming GREEN:       Voltage was/is below 3.3V
                                    will be reset to steady Green after 20sec
                                    Powersupply is not sufficient
     - LED steady  GREEN:       Relais is ON, 3.3V in allowed range

     ------------- ATtiny45/85 (5.5V Version) --------------
                            +---v---+
           (Relais Out) PB5 |1     8| VCC (5V from Powersupply)
           (/Button IN) PB3 |2     7| PB2 (AnalogIN 3.3V RPI Hdr-P1/Pin1)
    (LED Anode Red OUT) PB4 |3     6| PB1 (IN  RPI PowerEnable from RPI e.g. Pin#35 GPIO#19 OUT)
                        GND |4     5| PB0 (OUT LED Anode Green)
                            +-------+
     -------------------------------------------------------

        Fuses ( http://www.engbedded.com/fusecalc ):
                        CKDIV8=0        (disable) do not Divide clock by 8 internally (lfuse)
                        RSTDISBL=1         (enable)  using PB5 as I/O-Pin (hfuse)
                        !!!! Chip can not be reprogrammed -> reset FUSE by HighVoltage programmer

        avrdude -F -p attiny45 -P /dev/ttyACM0 -c stk500V2 -U lfuse:w:0xe2:m -U hfuse:w:0x5f:m -U flash:w:<file2flash>.hex

        use mikropascal to generate .hex flash file to program ATtiny *)
const
  VCCmax=    5.8;   // 5.513V (back powered) max expected
  VCCmin=    4.6;   // 4.95 (battery powered) min expected
//VCCok=     VCCmin+((VCCmax-VCCmin)/2); // 5,2315 default Voltage
  VCCok=     5.081;// included FwdVoltage Shottky BAT43: 0.283V
  ADReadMode=0;    // 0:Adc 1:AvgAdc
//ADC-Values
//Regulator NCP1117-3.3V: // min. Vout: 3.235 // max. Vout: 3.365
  OVR33Volt= 4.2;  // 3.9 range values, that will work with a lousy Powersupply
  OK33VoltH= 3.8;  // 3.5
  OK33VoltL= 3.1;  // 2.9
  OK33Volt=  OK33VoltL+((OK33VoltH-OK33VoltL)/2);

//https://www.mikrocontroller.net/articles/LED-Fading
  PWMtabmax=31;
  PWMtab: array[0..PWMtabmax] of byte =
    ( 0,  1,  2,  2,  2,  3,  3,  4,  5,  6,   7,   8,  10,  11,  13,  16,
     19, 23, 27, 32, 38, 45, 54, 64, 76, 91, 108, 128, 152, 181, 215, 255);
  pwmsteady=        PWMtabmax;    // idx: pwm value for LED steady on with maximum brightness
  pwmlow=            26;          // idx: lower intensity level (dimming)
  pwmdelaymax=        4;          //

//time counter divideable by 'wtim'
  wtim=            12;           //  debounce 12msec; in PWM mode, pwmlow * wtim=2048 msec for dim up or dim down
  cntBootUp=      984 div wtim;  // <1sec Button has to be pressed to recognize a Bootstrap
  cntShutDwn=    2976 div wtim;  // <3sec Button has to be pressed to recognize 
//rpi_hal has 3000msec / 1000msec and 7msec debounce time
  cntoffmax=    12000 div wtim;  //  12sec waittime for RPI shutdown-/boot-process
  cntPwrENAmin=     3  *  wtim;  // minimum 36msec PwrEna
  cntPwrENAmax=  2976 div wtim;  // maximum <3 sec PwrEna
  cntresetVfail= cntoffmax;
  cntLEDoffmax=  1500 div wtim;  // 1.5sec offtime @ dimming and idx =0
  cntblinkmaxc=   500 div wtim;  // counter for  1Hz
  cntblinkAmaxc=  250 div wtim;  // counter for  2Hz
  cntGetVREF=   60000 div wtim;  // get VREF every 60sec

  red=1;         green=2;
  LEDoff=0;      LEDon=1;      LEDdim=2;
  rpiWait4OFF=0; rpiWait4ON=1; rpiWait4Nodef=2; // awaiting next stat

//rpiSTATEs:
  rpiINV=0;      rpiOFF=1;     rpiSwitchOff=2; rpiSwitchOn=3;
  rpiDOboot=4;   rpiON=5;      rpiDOreboot=6;  rpiDOshutdown=7;// rpiSTATE

  VinMax=3;
  pat_init=$5aa5;

var
  rpiVOK,PWMup,RelaisStat,PWREnab,BUTpres,blink,ovrVOLT,tog,VCCerr: boolean;
  rpiSTATE,LED,PWMidx,PWMmax: byte;
  cntPwrENA,gcnt,rcnt,pcnt,prcnt,ocnt,dcnt,blinkcnt,blinkmax,pat: word;
  Vref,Vin: real;
  
procedure SetRelais(ein:boolean);
begin
  RelaisStat:=ein;
  if ein then
  begin
    pat:=word(pat_init);
    PORTB:=(PORTB or (1 shl PORTB5));
  end
  else
  begin
    PORTB:=(PORTB and (not (1 shl PORTB5)));
    pat:=word(not pat_init);
  end;
end;

procedure WritePWM(mode:byte);
begin
  case mode of
    LEDoff: begin PWMidx:=0;      PWMup:=true;  dcnt:=0; end; // steady OFF
    LEDon:  begin PWMidx:=PWMmax; PWMup:=false; dcnt:=0; end  // steady ON
    else    begin // dimming
              if (dcnt>=pwmdelaymax) or (PWMidx=0) then
              begin
                dcnt:=0;
                if PWMup then
                begin // up
                  if (PWMidx>=PWMmax) then
                  begin
                    PWMup:=false; PWMidx:=PWMmax-1;
                  end else inc(PWMidx);
                end
                else
                begin // down
                  if (PWMidx<=0) then
                  begin
                    if RelaisStat or (ocnt>cntLEDoffmax) then
                    begin
                      PWMup:=true; PWMidx:=1; ocnt:=0;
                    end else inc(ocnt);
                  end else dec(PWMidx);
                end;
              end else inc(dcnt);
           end;
  end; // case

  if (dcnt=0) then
  begin
  case LED of        //set by SetLED
    red:   OCR1B:=PWMtab[PWMidx];
    green: OCR0A:=PWMtab[PWMidx];
  end; // case
  end;
end;

procedure InitLED(lednr:byte; PWMmode:boolean);
// http://matt16060936.blogspot.com/2012/04/attiny-pwm.html
begin
  case lednr of
    green: begin
             if PWMmode then
             begin // LED green
               TCCR0A:=(2 shl COM0A0) or  // Clear OC0A on Compare Match,
                       (3 shl WGM00);     // set OC0A at BOTTOM (non inv),
               TCCR0B:=(0 shl WGM02)  or  // Fast PWM, Mode:3
                       (1 shl CS00);      // Start Timer with no prescaler
             end
             else
             begin // just regular output
               TCCR0A:=0; 
               TCCR0B:=0; 
             end;
           end;
    red:   begin //LED red
             if PWMmode then
             begin
               TCCR1:=(1 shl CS10);       // no prescaler
               GTCCR:=(1 shl PWM1B) or    // LED red use OCR1B // PB4:PWM, red LED
                      (2 shl COM1B0);     // Set the OC1B output line (red LED), Start Timer
             end
             else 
             begin // just regular output
               TCCR1:=0;
               GTCCR:=0; 
             end;
           end;
  end; // case
end;

procedure SetLED(lednr,PWMmaxidx:byte);
begin
  LED:=lednr; PWMmax:=PWMmaxidx;
  case lednr of
    green: begin // green LED in PWM mode, red switch off
             InitLED(green,true);
             InitLED(red,  false);
             PORTB:=(PORTB and (not (1 shl PORTB4)));
           end;
    red:   begin // red LED in PWM mode, green switch off
             InitLED(red,  true);
             InitLED(green,false);
             PORTB:=(PORTB and (not (1 shl PORTB0)));
           end;
  end; // case
end;

procedure StartBlink(alarm:boolean);
begin
  blink:=true;
  if alarm 
    then blinkmax:=cntblinkAmaxc 
    else blinkmax:=cntblinkmaxc;
end;

procedure StopBlink; 
begin 
  blinkmax:=cntblinkmaxc; 
  blink:=false; 
end;

procedure InitPWM;
begin
  SetLED(red,pwmlow);
  StopBlink;
  PWMidx:= 0;
  PWMup:=  true;
end;

function  AD_Read(mode:byte):boolean;
const _loops=500;
var _ret:boolean; _cnt,_adc:word;
begin
  ADCSRA:=ADCSRA or (1 shl ADSC); // Start Analog conversion
  _cnt:=0;
  
  repeat
    delay_us(5); // wait for completion or timeout
    inc(_cnt);
  until (_cnt>=_loops) or ((ADCSRA and (1 shl ADSC))=0);

  _adc:= ADCL;
  _adc:=(ADCH shl 8) or _adc; // Read 10 Bit ADC data

  _ret:=(_cnt<_loops);

  case mode of
    110: begin // ADC is connected to internal 1.10V -> determine VCC/Vref
           Vref:=VCCok; VCCerr:=true; // real(1.10*1024.0) = 1126.4 ?? or 1.1*1023 = 1125.3
           if _ret and (_adc<>0) then Vref:=1126.4/real(_adc);
           
           if ((Vref<VCCmin) or (Vref>VCCmax)) 
             then Vref:=VCCok 
             else VCCerr:=false;
             
           _ret:=(not VCCerr);
         end
    else begin
           if _ret 
             then Vin:=real(_adc)*real(Vref)/1024.0
             else Vin:=OK33Volt; // simulate ok value
                                          
(*           if (rpiSTATE<>rpiOFF) then Vin:=OK33Volt else Vin:=0; // simul, if Relais Out not used *)
         end;
  end; // case
  AD_Read:=_ret;
end;

procedure InitADC(mode:byte);
var b:byte;
begin
  b:=0;
  case mode of
    110: b:=(12 shl MUX0)  // use VBG (MUX[3:0]: 1100)
    else b:=(1  shl MUX0); // use single ended Input ADC1 (PB2 Pin7)
                           // set ref. voltage (Vref) to VCC
  end; // case
  ADMUX:=  b;
  ADCSRA:=(7 shl ADPS0) or // set prescaler (64 6:110 / 128 7:110)
          (1 shl ADEN);    // enable ADC
  delay_ms(2);             // wait for Vref to settle
  AD_Read(mode);           // drop 1. reading
end;

procedure InitADC110V;
// Determine VCC
// connect bandgap internal 1.1V to ADC
// https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
//var n:word; ret:boolean;
begin
  InitADC(110);
  AD_Read(110);
end;

procedure InitADCandGetVref;
// meassure VCC and prepare for further ADC readings with (prepare for AD_Read(0))
begin
  Vref:=VCCok;
  InitADC110V;  // determine VCC (Value in Vref)
  InitADC(0);   // connect Pin7 (PB2) to DAC, determine Vin
end;

procedure InitACOMP;
begin // switch on, 1.1V to internal POSITIVE Comparator Input
  ACSR:=(1 shl ACBG);
end;

procedure InitVARs(StartVolt:real);
begin
//for idx:=0 to VinMax do VinArr[idx]:=StartVolt;
  ocnt:=0;      dcnt:=0;        gcnt:=0;       rcnt:=0;
  pcnt:=0;      prcnt:=0;       cntPwrENA:=0;  blinkcnt:=0;
  rpiVOK:=true; ovrVOLT:=false; tog:=false;    VCCerr:=false;
end;

procedure Input_Read;
var b:byte;
begin
  AD_Read(ADReadMode);                // read Vin

  b:=PINB;
  BUTpres:=((b and (1 shl PINB3))= 0); // TRUE @ active low

//compare analog input voltage level on Pin PB1 with internal 1.1V
  PWREnab:=((ACSR and (1 shl ACO))=0); // NEGATIVE Comparator Input TRUE @ active high
end;

procedure TinyInit;
begin
  DDRB:=  (1 shl DDB5) or    // Relais:       PB5 Pin1 OUT
          (1 shl DDB4) or    // LED Red:      PB4 Pin3 OUT
          (1 shl DDB0);      // LED Green:    PB0 Pin5 OUT

  PORTB:= PORTB or           // no PullUP     PB2 ADC
          (1 shl PORTB3) or  // PullUp Enable PB3 Button
          (1 shl PORTB1);    // PullUp Enable PB1 PowerEnable

  InitACOMP;
  Vref:=VCCok;               // VCC default on ATtiny as Vref
  if (pat<>pat_init) then
  begin                      // regular start
    SetRelais (false);
    rpiSTATE:=rpiOFF;
    InitVARs  (0.0);
    InitADCandGetVref;
  end
  else
  begin // fast restart due to BrownOut, VCC problems....
    SetRelais (true);
    rpiSTATE:=rpiDOboot;
    InitVARs  (OK33Volt);
    InitADC   (0);
  end;
  InitPWM;
end;

procedure Blink5sec(alarm:boolean);
const _5sec=5000 div wtim;
var n,j:word; ein:boolean;
begin
  n:=0; j:=0; ein:=false;
  StartBlink(alarm);
  while (n < _5sec) do
  begin
    if (j>blinkmax) then 
    begin 
      j:=0; 
      ein:=(not ein);
    end else inc(j);
        
    SetLED(green,pwmsteady);
    if ein then WritePWM(LEDon) else WritePWM(LEDoff);
    delay_ms(wtim);
    inc(n);
  end;
  StopBlink;
end;

procedure ResetErrors;
begin
//if VCCerr then begin Blink5sec(green,true); VCCerr:=false; end;
  if (not rpiVOK) then
  begin
    inc(rcnt);
    if (rcnt>=cntresetVfail) then
    begin
      blinkcnt:=0;
      ovrVOLT:= false;
      rpiVOK:=  true; // reset voltage fail signaling after 20sec
    end;
  end else rcnt:=0;
end;

procedure SetLED4Voltage;
begin
  if (Vin<OVR33Volt) then
  begin
    StopBlink;
    if RelaisStat then
    begin // Relais ON
      if (rpiSTATE=rpiDOshutdown) or
         (rpiSTATE=rpiDOreboot) then
      begin
        if (rpiSTATE=rpiDOshutdown)
          then SetLED(red,  pwmsteady)
          else SetLED(green,pwmsteady);
        StartBlink(false);
      end
      else
      begin
        if ((Vin>=OK33VoltL) and (Vin<=OK33VoltH)) then
        begin
          rpiVOK:=true;
          SetLED(green,pwmsteady);        // Voltage outside
        end
        else
        begin
          rpiVOK:=false;                 // set only if Relais is on
          SetLED(red, pwmsteady);        // Voltage in OK range
        end;
        
        if (rpiSTATE=rpiDOboot) then StartBlink(false);
      end;
    end
    else
    begin // Relais OFF
      if ((Vin<OK33VoltL) or (Vin>OK33VoltH))
        then SetLED(red,  pwmlow)         // Voltage outside
        else SetLED(green,pwmlow);        // Voltage in OK range
    end;
  end
  else
  begin // OverVoltage
    ovrVOLT:=true;
    rpiVOK:=false;
    SetLED(red,pwmsteady); 
    StartBlink(true);
  end;

  if blink then
  begin
    if (blinkcnt<=(blinkmax shr 1))
      then WritePWM(LEDon)
      else WritePWM(LEDoff);
    
    if (blinkcnt>=blinkmax) 
      then blinkcnt:=0 
      else inc(blinkcnt);
  end
  else
  begin
    if RelaisStat        
      then WritePWM(LEDon)        // steady ON
      else WritePWM(LEDdim);      // RPI in OFF Status (LED PWM mode)
  end;
end;

procedure DoRelais(stat:boolean);
begin
  SetLED(red, pwmsteady);
  WritePWM(LEDon);
  if stat then
  begin // switch Relais OFF
    rpiSTATE:=rpiDOshutdown;
    
    gcnt:=0;
    repeat // RPI shutdown process (20sec)
      Input_Read;
      SetLED4Voltage;
      delay_ms( wtim);
      inc(gcnt);
    until (gcnt>=cntoffmax);
    gcnt:=0;
    
    SetRelais(false);
    rpiSTATE:=rpiOFF;
    InitVARs (0.0);
  end
  else
  begin // switch Relais ON
//  if not ovrVOLT then // prevent start again
    begin
      SetRelais(true);
      rpiSTATE:=rpiDOboot;
      InitVARs(OK33Volt);
    end;
  end;
  delay_ms(500);
  InitADCandGetVref;
end;

function  rpiNextSTATE(state,nextstate:byte; setOncount:word; var counter:word):boolean;
var bool:boolean;
begin
  if (rpiSTATE=state) then
  begin
    bool:=true;
    
    if (counter>=setOncount) then
    begin
      counter:=0;
      rpiSTATE:=nextstate;
    end else inc(counter);
    
  end else bool:=false;
  rpiNextSTATE:=bool;
end;

procedure STATEmachine;
begin
  if BUTpres then
  begin
    rpiNextSTATE(rpiOFF,rpiSwitchOn,  cntBootUp, cntPwrENA);
    rpiNextSTATE(rpiON, rpiDOshutdown,cntShutDwn,cntPwrENA);
  end
  else
  begin
    rpiNextSTATE(rpiDOboot,  rpiON,    cntPwrENAmax,cntPwrENA);
    rpiNextSTATE(rpiDOreboot,rpiDOboot,cntPwrENAmax,cntPwrENA);  // 3sec -> reset

    if (rpiSTATE=rpiOFF) or (rpiSTATE=rpiON) then cntPwrENA:=0;
  end;

  if PWREnab then
  begin // active LOW on PB1 (IN  RPI PowerEnable from RPI e.g. Pin#35 GPIO#19 OUT)

    if tog then
    begin
      if rpiNextSTATE(rpiOFF,rpiSwitchOn,  4,pcnt) then tog:=false;
      if rpiNextSTATE(rpiON, rpiDOshutdown,4,pcnt) then tog:=false;
    end;

  end else tog:=true;

  if (pcnt>0) then
  begin // reset after 3sec
    inc(prcnt);
    if (prcnt>(cntPwrENAmax)) then 
    begin 
      prcnt:=0; 
      pcnt:=0; 
      tog:=false; 
    end;
  end else prcnt:=0;

  if (rpiSTATE=rpiSwitchOn) or
//   (ovrVOLT and RelaisStat) or // switch OFF
     (rpiSTATE=rpiDOshutdown) then DoRelais(RelaisStat);
end;

begin
  TinyInit;
  repeat
    Input_Read;
    STATEmachine;
    SetLED4Voltage;
    ResetErrors;
    if (gcnt>cntGetVREF) then
    begin // get Vref every 60sec
      InitADCandGetVref;
      gcnt:=0;
    end;
    delay_ms(wtim);
    inc(gcnt);
  until false;
end.
