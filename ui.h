/**********************************************************************
 BrewManiac 
 created by Vito Tai
 Copyright (C) 2015 Vito Tai
 
 This soft ware is provided as-is. Use at your own risks.
 You are free to modify and distribute this software without removing 
 this statement.
 BrewManiac by Vito Tai is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
***********************************************************************/

#ifndef UI_H
#define UI_H

#define LCD_COLUMN_NUM 20

#include <avr/pgmspace.h>

#if SupportManualModeCountDown == true
#define  SupportRunningTimeBlink true
#endif

//const char* SpaceInRow="                    ";
//const char* SpaceInRow18="                  ";


byte _uiTpDisplayRow;
byte _uiTpDisplayCol;
boolean _uiDisplayTemperature;

//boolean _uiShowCountingTime;
#define COUNTING_PAUSE 0
#define COUNTING_UP 1
#define COUNTING_DOWN 2

byte _countingTimeDirection;
unsigned long _countingTimeRef;
unsigned long _countingTimeDisplay;

const byte BluetoothSymbol[8] PROGMEM ={

	B00110,
	B10101,
	B01101,
	B00110,
	B01110,
	B10101,
	B00101,
	B00110};
	
const byte RevBluetoothSymbol[8] PROGMEM ={
	B00010,
	B00100,
	B01000,
	B11111,
	B00010,
	B00100,
	B01000,
	B00000};

const byte CelsiusSymbol[8]  PROGMEM  = {B01000, B10100, B01000, B00111, B01000, B01000, B01000, B00111};  // [0] degree c sybmol 
const byte FahrenheitSymbol[8] PROGMEM = {B01000, B10100, B01000, B00111, B00100, B00110, B00100, B00100};  // [0] degree f symbol

const byte SetpointSymbol[8]  PROGMEM  = {B11100, B10000, B11100, B00111, B11101, B00111, B00100, B00100};  // [2] SP Symbol

const byte PumpSymbol[8]  PROGMEM  = {B00000, B01110, B01010, B01110, B01000, B01000, B01000, B00000};  // [3] Pump Symbol 
const byte RevPumpSymbol[8] PROGMEM = {B11111, B10001, B10101, B10001, B10111, B10111, B10111, B11111};  // [4] Reverse PUMP Symbol
const byte HeatingSymbol[8] PROGMEM   = {	
	B00000, 
	B01010, 
	B01010, 
	B01110, 
	B01110, 
	B01010, 
	B01010, 
	B00000};  // [5] HEAT symbol
const byte RevHeatingSymbol[8] PROGMEM = {B11111, 
	B10101, 
	B10101, 
	B10001, 
	B10001, 
	B10101, 
	B10101, 
	B11111};  // [6] reverse HEAT symbol

//byte Language[8]  PROGMEM   = {B00000, B10111, B10101, B11101, B00000, B10001, B10101, B11111};  // [7] ESP symbol


#define LcdCharDegree ((byte)0)

#define LcdCharSetpoint 1
#define LcdCharPump 2
#define LcdCharRevPump 3
#define LcdCharHeating 4
#define LcdCharRevHeating 5

#define LcdCharBluetooth 6

#define LcdCharReserved 7
#define CreatecCustomChar(buff,idx,bm) uiGetBitmap(buff,bm); lcd.createChar(idx,buff)

void uiGetBitmap(byte *dst,const byte *addr);

byte _uibuffer[21];

#if BluetoothSupported == true
boolean _btConnectedIconLoaded;
#endif

void uiInitialize(void)
{
	//byte bitmapBuffer[8];
	
   	_uiTpDisplayRow=false;
   	//_uiShowCountingTime=false;
   	_countingTimeDirection=COUNTING_PAUSE;
   
	lcd.begin(20,4);
    if(gIsUseFahrenheit)
    {
       	CreatecCustomChar(_uibuffer,LcdCharDegree,FahrenheitSymbol);
    }
   	else 
   	{
   		CreatecCustomChar(_uibuffer,LcdCharDegree,CelsiusSymbol);
   	}
   	
   	CreatecCustomChar(_uibuffer,LcdCharSetpoint,SetpointSymbol);

   	CreatecCustomChar(_uibuffer,LcdCharPump,PumpSymbol);
   	CreatecCustomChar(_uibuffer,LcdCharRevPump,RevPumpSymbol);
   	CreatecCustomChar(_uibuffer,LcdCharHeating,HeatingSymbol);
   	CreatecCustomChar(_uibuffer,LcdCharRevHeating,RevHeatingSymbol);
#if BluetoothSupported == true
   	CreatecCustomChar(_uibuffer,LcdCharBluetooth,BluetoothSymbol);
   	_btConnectedIconLoaded=false;
#endif
}
//********************************************************
//* helper functions
//********************************************************


void uiGetBitmap(byte *dst,const byte *addr)
{	
	for (int i=0; i<8; i++)
    {
      dst[i] =	pgm_read_byte_near(addr + i);
    }
}
#if NoPrint != true
int numberOfDigitFloat(float number,int precision)
{

  int digits=(precision==0)? 0:(precision +1); // .0 or .00 
  if(number <0)
  {
     digits+=1;
     number = 0- number;
  }
  
  if(number >= 1000.0) digits+=4;
  else if(number >= 100.0) digits+=3;
  else if(number >= 10.0)  digits+=2;
  else /* 1.0 */digits+=1;
  return digits;
}

#endif
//********************************************************
//* BT
//********************************************************

#if BluetoothSupported == true

#define BtStateNoModule 0
#define BtStateNotConnected 1
#define BtStateConnected 2
byte _btStatus;

void uiDisplayBtIcon(void)
{
	lcd.setCursor(19,0);
	if(_btStatus == 0)
		lcd.write(' ');
	else if(_btStatus == 1)
	{
		if(_btConnectedIconLoaded)
		{
	   		CreatecCustomChar(_uibuffer,LcdCharBluetooth,BluetoothSymbol);
   			_btConnectedIconLoaded=false;
   		}
		lcd.write(LcdCharBluetooth);
	}
	else // ==2
	{
		if(!_btConnectedIconLoaded)
		{
	   		CreatecCustomChar(_uibuffer,LcdCharBluetooth,RevBluetoothSymbol);
   			_btConnectedIconLoaded=true;
   		}
		lcd.write(LcdCharBluetooth);
	}
}


void uiSetBtStatus(byte status)
{
	_btStatus=status;
	uiDisplayBtIcon();
}
#endif


#define LeftAligned 0
#define RightAligned 1
#define CenterAligned 2

//********************************************************
//* LCD helper function
//********************************************************


void uiClearRow(byte row)
{	
	byte i=0;
	byte num=20;
	if(row ==0) num=19;
	else if(row ==2) {i=1; num=19;}
	
	lcd.setCursor(i,row);
	for(;i<num;i++)
		lcd.write(' ');
}

byte LCD_print_P(const char* p)
{
	byte i=0;
	char ch;
	
	while((ch=(char)pgm_read_byte_near(p + i))!=0)
	{
  		lcd.write(ch);
  		i++;
  	}
	return i;
}

void uiShowTextAtRow(byte row,char *text,byte alignment,int indent)
{
	byte length=strlen(text);
	if(alignment ==LeftAligned)
	{
		lcd.setCursor(indent,row);
	}
	else
	{
		
		if(alignment ==RightAligned)
		{
			lcd.setCursor(LCD_COLUMN_NUM - indent - length,row);
		}
		else
		{
			// center, indent is useless
			lcd.setCursor( (LCD_COLUMN_NUM - length)/2,row);
		}
	}
	for(byte i=0;i<length;i++)
	{
  		lcd.write(text[i]);
  	}
}


void uiShowTextAtRow_P(byte row,str_t text,byte alignment,int indent)
{
	byte length=strlen_P(text);
	if(alignment ==LeftAligned)
	{
		lcd.setCursor(indent,row);
	}
	else
	{
		
		if(alignment ==RightAligned)
		{
			lcd.setCursor(LCD_COLUMN_NUM - indent - length,row);
		}
		else
		{
			// center, indent is useless
			lcd.setCursor( (LCD_COLUMN_NUM - length)/2,row);
		}
	}
	for(byte i=0;i<length;i++)
	{
	 	char ch=(char)pgm_read_byte_near(text + i);
  		lcd.write(ch);
  	}
}
//********************************************************
//* LCD interface
//********************************************************
//********************************************************
//* for update temperature and time on screen
//********************************************************
void uiChangeTemperatureUnit(bool useF)
{
    if(useF)
    {
       	CreatecCustomChar(_uibuffer,LcdCharDegree,FahrenheitSymbol);
    }
   	else 
   	{
   		CreatecCustomChar(_uibuffer,LcdCharDegree,CelsiusSymbol);
   	}
}
void uiTempDisplayHide(void)
{
	_uiDisplayTemperature=false;
}

const byte TemperatureDisplayPos[] PROGMEM=
{
	6,1,
	1,1,
	7,0
};

#define TemperatureMainScreenPosition 0
#define TemperatureManualModePosition 1
#define TemperaturePausePosition 2
#define TemperatureAutoModePosition TemperatureManualModePosition

void uiTempDisplaySetPosition(byte index)
{
   _uiDisplayTemperature=true;

   _uiTpDisplayRow=pgm_read_byte_near(TemperatureDisplayPos + index*2 +1);
   _uiTpDisplayCol=pgm_read_byte_near(TemperatureDisplayPos + index*2);   
}

//****************************
// running time

void uiRunningTimeStop(void)
{
	//redaundant _uiShowCountingTime=false;
	_countingTimeDirection = COUNTING_PAUSE;
}

const byte RunningTimeDisplayPosition[] PROGMEM=
{
	11,2,
	6,2,
};

#define RunningTimeNormalPosition 0
#define RunningTimePausePosition 1

#define RunningTimeDelayInputPosition 1

byte _runningTimeRow;
byte _runningTimeCol;

boolean _runningTimeHidden;


void uiRunningTimePrint(unsigned long timeInSeconds)
{

	unsigned long hour = timeInSeconds / (60*60);
	_uibuffer[0]= (char)((hour/10) + '0');
	_uibuffer[1]= (char)((hour%10) + '0');
	_uibuffer[2]=':';

	unsigned long minute =(timeInSeconds - hour * 60*60)/60;

	_uibuffer[3]= (char)((minute/10) + '0');
	_uibuffer[4]= (char)((minute%10) + '0');
	_uibuffer[5]=':';

	unsigned long seconds=timeInSeconds - hour*60*60 - minute*60;

	_uibuffer[6]=(char)((seconds/10) + '0');
	_uibuffer[7]=(char)((seconds%10) + '0');
	//_uibuffer[8]=0;
	
	lcd.setCursor(_runningTimeCol,_runningTimeRow);
	for(byte i=0;i<8;i++)
		lcd.write(_uibuffer[i]);
}

void uiRunningTimeHide(boolean hide)
{
	_runningTimeHidden = hide;
	if(! hide)
	{
		uiRunningTimePrint(_countingTimeDisplay);
	}
	
}


void uiRunningTimeSetPosition(byte pos)
{
   _runningTimeRow=pgm_read_byte_near(RunningTimeDisplayPosition + pos*2 +1);
   _runningTimeCol=pgm_read_byte_near(RunningTimeDisplayPosition + pos*2);   
}

void uiRunningTimeShowInitial(unsigned long initialValue)
{
	_countingTimeDirection = COUNTING_PAUSE;
	_countingTimeDisplay = initialValue;
	_runningTimeHidden= false;
	uiRunningTimePrint(initialValue);
}

void uiRunningTimeStart(void)
{
	// use reference to note time.
	_countingTimeRef=gCurrentTimeInSec;
	_countingTimeDirection = COUNTING_UP;
}

void uiRunningTimeStartCountDown(unsigned long seconds)
{
	_countingTimeRef=gCurrentTimeInSec + seconds;
	_countingTimeDirection = COUNTING_DOWN;
}


#if SupportRunningTimeBlink == true
boolean _runningTimeBlinking;
boolean _runningTimeBlinkShow;

#define RunningTimeShowCycle 700
#define RunningTimeHideCycle 300

unsigned long _runningTimeBlinkTime;

void uiRunningTimeBlank(void)
{
	lcd.setCursor(_runningTimeCol,_runningTimeRow);
	for(byte i=0;i<8;i++)
		lcd.write(' ');
}

void uiRunningTimeBlink(boolean blink)
{
	if(blink && !_runningTimeBlinking)
	{
		_runningTimeBlinkTime = gCurrentTimeInMS;
	}
	_runningTimeBlinking = blink;
	
	if(! _runningTimeBlinking && ! _runningTimeBlinkShow)
		uiRunningTimePrint(_countingTimeDisplay);
}

#endif

void uiDisplayTemperatureAndRunningTime(void)
{
    if(_uiDisplayTemperature)
    {
    	float displayTemp=gCurrentTemperature;
    	
    	if(gIsUseFahrenheit) 
    	{
    		displayTemp = ConvertC2F(gCurrentTemperature);
    	}
        
        #if NoPrint == true
        byte digitNum=sprintFloat((char*)_uibuffer,displayTemp,2);
         
        lcd.setCursor(_uiTpDisplayCol,_uiTpDisplayRow);
        for(byte i=0;i< 6 - digitNum;i++) lcd.write(' ');
        for(byte i=0;i< digitNum;i++) lcd.write(_uibuffer[i]);

        #else
        int digitNum=numberOfDigitFloat(displayTemp,2);
         
        lcd.setCursor(_uiTpDisplayCol,_uiTpDisplayRow);
        for(int i=0;i< 6 - digitNum;i++) lcd.write(' ');
        lcd.print(displayTemp,2);
        #endif
        
        lcd.write(LcdCharDegree);
    }
#if SupportRunningTimeBlink == true
			if(_runningTimeBlinking)
			{
				if(_runningTimeBlinkShow)
				{
					if((gCurrentTimeInMS - _runningTimeBlinkTime) > RunningTimeShowCycle)
					{
						_runningTimeBlinkTime=gCurrentTimeInMS;
						_runningTimeBlinkShow = false;
						//hide
						uiRunningTimeBlank();
					}
				}
				else
				{ 
					if((gCurrentTimeInMS - _runningTimeBlinkTime) > RunningTimeHideCycle)
					{
						_runningTimeBlinkTime=gCurrentTimeInMS;
						_runningTimeBlinkShow = true;
						// shown
    					uiRunningTimePrint(_countingTimeDisplay);
					}
				}

			}
#endif

    if(_countingTimeDirection != COUNTING_PAUSE)
    {
    	unsigned long count;
    	if (_countingTimeDirection == COUNTING_UP)
    	{
    		count=gCurrentTimeInSec - _countingTimeRef;
    	}
    	else
    	{
    		// counting down
    		count=_countingTimeRef - gCurrentTimeInSec;
    		if(count > 24*60*60*1000)
    		{
    			count =0;
    			_countingTimeDirection = COUNTING_PAUSE;
    		}
    	}

	 	if(! _runningTimeHidden)
	 	{
	 		if(count != _countingTimeDisplay)
    		{
    			_countingTimeDisplay=count;
    			uiRunningTimePrint(_countingTimeDisplay);
    		}

    	}
    }
}

#define  uiClearTitle() uiClearRow(0)
#define  uiClearSettingRow() uiClearRow(2)
#define  uiClearSubTitleRow() uiClearRow(1)
#define  uiNoMenu() uiClearRow(3)
#define uiClearPrompt() uiClearRow(2)

//******************************************************
// General interface
//******************************************************

#define HeatingStatus_On 1
#define HeatingStatus_On_PROGRAM_OFF 2
#define HeatingStatus_Off 0

#define HeatingSymbolRow 2
#define HeatingSymbolCol 0

#if BluetoothSupported == true
void btReportHeater(byte value);
void btReportPump(byte value);
#endif


void uiHeatingStatus(byte status)
{
#if BluetoothSupported == true
	btReportHeater(status);
#endif
	lcd.setCursor(HeatingSymbolCol,HeatingSymbolRow);
	if(status==HeatingStatus_On)
		lcd.write(LcdCharRevHeating);
	else if(status==HeatingStatus_On_PROGRAM_OFF)
		lcd.write(LcdCharHeating);
	else 
		lcd.write(' ');
}

#define PumpStatus_On 1
#define PumpStatus_On_PROGRAM_OFF 2
#define PumpStatus_Off 0

#define PumpSymbolRow 2
#define PumpSymbolCol 19

void uiPumpStatus(byte status)
{
#if BluetoothSupported == true
	btReportPump(status);
#endif

	lcd.setCursor(PumpSymbolCol,PumpSymbolRow);
	if(status==PumpStatus_On)
		lcd.write(LcdCharRevPump);
	else if(status==PumpStatus_On_PROGRAM_OFF)
		lcd.write(LcdCharPump);
	else 
		lcd.write(' ');
}

void uiTitle(str_t text)
{
	uiClearTitle();
	uiShowTextAtRow_P(0,text,CenterAligned,0);
}

void uiSubTitle(str_t text)
{
	uiClearRow(1);
  	uiShowTextAtRow_P(1,text,CenterAligned,0);
}
//****************************************
// For setting
// we have to remember the length of

byte _labelLegth;

//{ for edit text(number)
byte _editingCol;
void uiEditTextStart(char *buf)
{
	byte length=strlen(buf);
/*	lcd.setCursor(LCD_COLUMN_NUM - 1 - length,2);
	for(byte i=0;i<length;i++)
	{
  		lcd.write(buf[i]);
  	}
*/
	_editingCol =LCD_COLUMN_NUM - 1 - length;
	lcd.setCursor(_editingCol,2);
	lcd.blink();
}

void uiEditTextNext(void)
{
	lcd.setCursor(++_editingCol,2);
}

void uiEditTextSetCharAtCursor(char ch)
{
	lcd.write(ch);
	lcd.setCursor(_editingCol,2);
}

void uiEditTextEnd(void)
{
	lcd.noBlink();
}
//}

void uiSettingFieldClear(void)
{
	lcd.setCursor(_labelLegth+1,2);	
	// one off at right most 
	for(byte i=_labelLegth;i<(LCD_COLUMN_NUM-1);i++)
		lcd.write(' ');
}

void uiSettingTitle(str_t text)
{
	// need to remember the length of text
	// so that when showing parameters, 
	// we know how to clear the LCD line
	_labelLegth = strlen_P(text);
	uiShowTextAtRow_P(2,text,LeftAligned,1);
	uiSettingFieldClear();
}


void uiSettingTitleAppendNumber(byte number) // specially for HOP
{
	lcd.setCursor(_labelLegth+1,2);
	lcd.write('0' + (number/10));
	lcd.write('0' + (number%10));
	
	_labelLegth += 2;
}

// the following code is for show Setting Value
void uiSettingDisplayText(str_t text)
{
//	uiSettingFieldClear();
	uiShowTextAtRow_P(2,text,RightAligned,1);
}

void uiSettingDisplayTextDynamic(char* text)
{
//	uiSettingFieldClear();
	uiShowTextAtRow(2,text,RightAligned,1);
}

// for degree symbol

void uiSettingDegreeSymbol(byte value)
{
//	uiSettingFieldClear();

	if(value ==0)
	{
		CreatecCustomChar(_uibuffer,LcdCharReserved,CelsiusSymbol);
	}
	else
	{
		CreatecCustomChar(_uibuffer,LcdCharReserved,FahrenheitSymbol);	
	}

	lcd.setCursor(18,2);
	
	lcd.write(LcdCharReserved);
}


void uiSettingDisplayField(float number,byte precision,char unit)
{
//	uiSettingFieldClear();
	
#if NoPrint == true
	int digitNum=sprintFloat((char*)_uibuffer,number,precision);
	// +1 for one space at right end
    lcd.setCursor(LCD_COLUMN_NUM -1 -1 - digitNum ,2); // indent left 1
    for(byte i=0;i<digitNum;i++)
    	lcd.write(_uibuffer[i]);
#else
	int digitNum=numberOfDigitFloat(number,precision);
	// +1 for one space at right end
    lcd.setCursor(LCD_COLUMN_NUM -1 -1 - digitNum ,2); // indent left 1
    lcd.print(number,precision);
#endif
	lcd.write(unit);
}


void uiSettingTimeInMinutes(byte minutes)
{
	uiSettingDisplayField((float)minutes,0,'m');
}

void uiSettingShowTemperature(float temp,byte precision)
{
	float displayTemp=temp;
	if(gIsUseFahrenheit){ displayTemp=ConvertC2F(temp); }
    uiSettingDisplayField((float)displayTemp,2,LcdCharDegree);
}

void uiDisplaySettingPercentage(int number)
{
	uiSettingDisplayField((float)number,0,'%');
}

void uiSettingDisplayNumber(float number,byte precision)
{
	uiSettingDisplayField((float)number,0,' ');
}


void uiClearScreen(void)
{
	lcd.clear();
#if BluetoothSupported == true
	uiDisplayBtIcon();
#endif
	
}

void uiMenu(str_t text)
{
	uiClearRow(3);
	uiShowTextAtRow_P(3,text,LeftAligned,1);
}

void uiDisplaySettingTemperature(float settemp)
{
	float displayTemp;
	if(gIsUseFahrenheit){ displayTemp=ConvertC2F(settemp); }
	else displayTemp = settemp;
#if NoPrint == true
    int digitNum=sprintFloat((char*)_uibuffer,displayTemp,2);
         
    lcd.setCursor(12,1);
    
    for(int i=0;i< 6 - digitNum;i++) lcd.write(' ');
    for(int i=0;i<digitNum;i++) 
    	lcd.write(_uibuffer[i]);

#else
    int digitNum=numberOfDigitFloat(displayTemp,2);
         
    lcd.setCursor(12,1);
    
    for(int i=0;i< 6 - digitNum;i++) lcd.write(' ');
    lcd.print(displayTemp,2);
#endif    
    lcd.write(LcdCharSetpoint);
}

void uiClearPwmDisplay(void)
{
	lcd.setCursor(1,2);
	for(int i=0;i<8;i++)
		lcd.write(' ');
}

void uiShowPwmLabel(void)
{
	lcd.setCursor(1,2);
	LCD_print_P(STR(PWM_Is));
}

void uiShowPwmValue(byte pwm)
{
	// make it simple, should optimize later
	lcd.setCursor(5,2);
	
	
	lcd.write((pwm==100)? '1':' ');
	if(pwm==100) lcd.write('0');
	else lcd.write((pwm/10)? ('0' +(pwm/10)):' ');//100 is maximum, no way it can over 10
	lcd.write('0' + (pwm%10));
}

 
void  uiPrompt(str_t prompt)
{
	uiShowTextAtRow_P(2,prompt,CenterAligned,0);
}

void uiAutoModeTitle(void)
{
	lcd.setCursor(1,0);
	LCD_print_P(STR(Auto_Label));
}

const char * const  AutoStepNames[] PROGMEM =
{
STR( Mash_In),
STR( Phytase),
STR( Glucanase),
STR( Protease), 
STR( bAmylase),
STR( aAmylase1),
STR( aAmylase2),
STR( Mash_out), 
STR( Boil), 
STR( Cooling),
STR( Whirlpool)
};

#define BoilingStage 8
#define CoolingStage 9
#define WhirlpoolStage 10

void uiAutoModeStage(byte idx)
{
	lcd.setCursor(10,0);
	
	str_t display = (str_t)pgm_read_word(&(AutoStepNames[idx]));
	
	byte len=LCD_print_P(display);
	byte i=len+10;
#if BluetoothSupported == true
	for(;i<19;i++) lcd.write(' ');
#else
	for(;i<20;i++) lcd.write(' ');
#endif
}

void uiAutoModeShowHopNumber(byte number)
{
	lcd.setCursor(10,0);
	LCD_print_P(STR(Hops_Number_x));
	lcd.write('0' + (number/10));
	lcd.write('0' + (number%10));
}


void uiPreparePasueScreen(str_t message)
{
	uiClearScreen();	

	byte i;
	lcd.setCursor(0,0);
	for(i=0;i<5;i++) lcd.write('-');
	lcd.setCursor(15,0);
	for(i=0;i<5;i++) lcd.write('-');
	lcd.setCursor(1,2);
	for(i=0;i<4;i++) lcd.write('-');
	lcd.setCursor(15,2);
	for(i=0;i<4;i++) lcd.write('-');

	uiShowTextAtRow_P(1,message,CenterAligned,0);
	uiTempDisplaySetPosition(TemperaturePausePosition);
	uiRunningTimeSetPosition(RunningTimePausePosition);
}

void uiAutoModeFinishScreen(void)
{
	uiShowTextAtRow_P(1,STR(Brewing_Process),CenterAligned,0);
	uiShowTextAtRow_P(2,STR(Finished),CenterAligned,0);
}
#endif

