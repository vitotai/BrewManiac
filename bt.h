/**********************************************************************
 BrewManiac 
 created by Vito Tai
 Copyright (C) 2015 Vito Tai
 
 This soft ware is provided as-is. Use at your own risks.
 You are free to modify and distribute this software without removing 
 this statement.
 BrewManiac by Vito Tai is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
***********************************************************************/

#ifndef BT_H
#define BT_H


#define wiReportCurrentStage btReportCurrentStage
#define wiReportEvent btReportEvent
#define wiReportPwm btReportPwm
#define wiReportSettingTemperature btReportSettingTemperature
#define wiInitialize btInitialize
#define wiThread btThread

#define btReportHeater wiReportHeater
#define btReportPump wiReportPump

boolean gIsBtModulePresent;
boolean gIsConnected;

//default baudrate is for non auto baud rate
#if CHANG_BAUDRATE == true
#define BT_DefaultBaudRate 9600
#endif

#define TrialNumber 2

#define BT_PreferredBaudRateIndex 2 //57600

//0---------9600 
//1---------19200 
//2---------38400 
//3---------57600 
//4---------115200

#define BT_DEBUGTEMP false

//*******************************************************
#if BT_MODULE_INITIALIZATION == true

byte btSecurityType;
char btPinCode[20];
byte  baudIndex;

char btNetworkName[20];

#endif


#if UseSoftwareSerial == true
SoftwareSerial btSerial(SoftwareSerialRx,SoftwareSerialTx);
#else

#define btSerial Serial

#endif


#if NoPrint == true

char btBuff[64];
byte btIndex;
//bool btUseChecksum;
// sentense always starts with $V[c1][c2],
void btSentenceStart(char c1, char c2)
{
//	btUseChecksum = checksum;
	// the start of sentence
    btBuff[0]='$';
    btBuff[1]='V';
    btBuff[2]=c1;
    btBuff[3]=c2;
    btBuff[4]=',';    
    btIndex=5;
}

void btSentenceComma(void)
{
	btBuff[btIndex++]=',';
}

void btSentenceInt(int number)
{
	btIndex+=sprintInt(btBuff+btIndex,number);

}

void btSentenceFloat(float number)
{
	btIndex+=sprintFloat(btBuff+btIndex,number,2);
}
char toHex(byte v)
{
	if(v >= 10) return ('A' + (v -10));
	return '0' + v;
}
void btSentenceSend(void)
{
	//if(btUseChecksum){
		byte checksum='V';
		for(byte i=2;i< btIndex ; i++)
			checksum = checksum ^ btBuff[i];
			
		btBuff[btIndex++]='*';
		btBuff[btIndex++]=toHex(checksum >> 4);
		btBuff[btIndex++]=toHex(checksum & 0xF);
	//}
	btBuff[btIndex++]='\r';
	btBuff[btIndex++]='\n';
	for(byte i=0;i<btIndex;i++)
		btSerial.write(btBuff[i]);
}
void btSentenceInt(char c1, char c2,int p1)
{
	btSentenceStart(c1,c2);
	btSentenceInt(p1);
	btSentenceSend();
}
#endif 


#if BT_AutoBaudRate == true
long baudrateFromIndex(byte index)
{
	if(index ==0) return 9600;
	if(index ==1) return 19200;
	if(index == 2) return 38400;
	if(index == 3) return 57600;
	//if(index == 4) 
		return 115200;
}

//const int _supportedBauds[] PROGMEM ={9600,38400,57600,19200,115200};
const byte _trialSequence[] PROGMEM ={0,2,3,1,4};

void btTestAvailable(void)
{
	gIsBtModulePresent = false;
	
    for(byte i=0;i< 5;i++)
    {
    	byte tidx=(byte)pgm_read_byte(&(_trialSequence[i]));
      	btSerial.begin(baudrateFromIndex(tidx));
      
      	for(byte t=0;t<TrialNumber;t++)
      	{
#if NoPrint == true
			btSerial.write('A');
			btSerial.write('T');
#else
        	btSerial.print(F("AT"));
#endif
          	delay(100);
          	char ch;
          	
          	if(btSerial.available()) ch=btSerial.read();
          	if(ch == 'O')
          	{
              	if(btSerial.available()) ch=btSerial.read();
              	if(ch == 'K')
              	{
                	gIsBtModulePresent=true;
#if                SerialDebug == true
                Serial.print("Bt buad:");
                Serial.println(baudrateFromIndex(tidx));
#endif

#if BT_MODULE_INITIALIZATION == true
					baudIndex = tidx;
#endif
                	return;
              	} // if K
         	}// if O
      	}//for(byte t=0;t<TrialNumber;t++)
    } // for(byte i=0;i<sizeof(bauds)/sizeof(int);i++)
}
#else
void btTestAvailable(void)
{
	gIsBtModulePresent = false;
	
    btSerial.begin(BT_DefaultBaudRate);
    for(byte t=0;t<TrialNumber;t++)
    {
#if NoPrint == true
			btSerial.write('A');
			btSerial.write('T');
#else
    	btSerial.print(F("AT"));
#endif
        delay(100);
        char ch;
          	
        if(btSerial.available()) ch=btSerial.read();
        if(ch == 'O')
        {
        	if(btSerial.available()) ch=btSerial.read();
            if(ch == 'K')
            {
            	gIsBtModulePresent=true;
                return;
            } // if K
        }// if O
    }//for(byte t=0;t<TrialNumber;t++)
}
#endif //BT_AutoBaudRate == true

#if BT_MODULE_INITIALIZATION == true

void atSendAtCmd(const char* cmd,const char* parameter)
{
  #if SerialDebug == true
    Serial.println(">>>atSendAtCmd:");
  #endif

  	btBuff[0]='A';
	btBuff[1]='T';
	btBuff[2]='+';
	byte idx=3;
	byte i=0;
	char ch;
	while((ch=(char)pgm_read_byte_near(cmd + i)) != 0){
        i++;
        btBuff[idx++]=ch;
    }

	i=0;	
	if(parameter!=0)
	{
//		while( (ch=(char)pgm_read_byte_near(parameter + i)) != 0){
		while( (ch= *(parameter + i)) != 0){
		 	i++;
		 	btBuff[idx++]=ch;
		}
	}
	
	#if SerialDebug == true
	btBuff[idx]=0;
    Serial.println(btBuff);
  	#endif

	for(i=0;i<idx;i++)
	{
		btSerial.write(btBuff[i]);
	}

  #if SerialDebug == true
  Serial.println("<<<atSendAtCmd");
  #endif
}

boolean atCommand(const char* cmd,const char* parameter)
{
#if NoPrint == true
	atSendAtCmd(cmd,parameter);
#else
    btSerial.print(F("AT+"));
    btSerial.print(cmd);
    if(parameter)  btSerial.print(parameter);
#endif
    
    delay(100);
    // check OK
    byte i=0;
    boolean ret=true;
    while(btSerial.available())
    {
       char ch=btSerial.read();

#if SerialDebug == true
	Serial.print(ch);
#endif       
       
       if(i==0){ 
       		if (ch!='O') ret=false; 
       }
       else if(i==1){
        	if (ch!='K') ret=false; 
       }
#if BT_STRICT == true
       // read through all remending data
#endif
		i++;
    }
    return ret;
}

byte atQuery(const char* cmd, char *addr,byte maxLen,boolean useGet)
{
  // clear buffer
  while(btSerial.available()) btSerial.read();
#if NoPrint == true  
	atSendAtCmd(cmd,"?");
#else
  btSerial.print(F("AT+"));
  btSerial.print(cmd);
  btSerial.print('?');
#endif

  delay(100);

  byte endCmd;
 
  if(useGet)
    endCmd = 3 + 3; //Get:
  else
     endCmd = strlen_P(cmd) + 3;

     #if SerialDebug == true
	     Serial.print(endCmd);
        Serial.print(F("RSP:\""));
	#endif
  
  byte i=0;
  byte len=0;
  while(btSerial.available())
  {
     char ch=btSerial.read();
     #if SerialDebug == true
        Serial.print(ch);
	#endif

     if(i==0){ if (ch!='O') return 0; }
     else if(i==1){ if (ch!='K') return 0; }
     else if(i==2){ if (ch!='+') return 0; }
     else if(i>2 && i < endCmd)
     { 
#if 0 //BT_STRICT == true
       // we can just ignored this.
        if(useGet)
        {
        }
        else
        {
          if (*(cmd + i - 3) != ch) return 0; 
        }
#endif
     }
     else if(i == endCmd ) { if (ch != ':') return 0; }
     else if(len < maxLen) 
     {
     	addr[len] = ch;
     	len++;
     }
     i++;
  }
	#if SerialDebug == true
        Serial.println(F("\""));
   #endif

  addr[len]='\0';
  return len;
}



const char btDefaultName[] PROGMEM ="BrewManiac";
const char NAME[] PROGMEM ="NAME";
const char TYPE[] PROGMEM ="TYPE";
const char PASS[] PROGMEM ="PASS";
const char BAUD[] PROGMEM ="BAUD";
// setup the bt module if it has not been done 
// NAME, PASS, connect type(pin code required?)
#if CHANG_BAUDRATE == true
byte btChangeBaudrate(byte index)
{
	//change mult
  #if SerialDebug == true
  Serial.println("btChangeBaudrate:");
  Serial.println(baudrateFromIndex(index));
  #endif
  	char idx[2];
  	idx[0]='0' + index;
  	idx[1]='\0';
	atCommand(BAUD,idx);
	// change baudrate
	delay(100);
	
	btTestAvailable();
	
	return index;
}
#endif

byte btSetUsePinCode(bool usePin)
{
	if(usePin)
	{
		if(atCommand(TYPE,"2"))
		{
			btSecurityType=2;
		}
	}
	else
	{
		if(atCommand(TYPE,"0"))
		{
			btSecurityType=0;
		}
	}
	return btSecurityType;
}

void btSetPinCode(char *pin)
{
		if(atCommand(PASS,pin))
		{
			strcpy(btPinCode,pin);
		}	
}

void btSetupBtModule(void)
{
  #if SerialDebug == true
  Serial.println("btSetupBtModule");
  #endif
#if CHANG_BAUDRATE == true
	if(baudIndex != BT_PreferredBaudRateIndex)
	{
		// change baud rate
		baudIndex=btChangeBaudrate(BT_PreferredBaudRateIndex);
	}
#endif

    byte ret=atQuery(NAME,btNetworkName,10,false);
    
    if(ret>0)
    {
  #if SerialDebug == true
  Serial.print("BT Name:");
  Serial.println(btNetworkName);
  #endif

        if(strcmp_P(btNetworkName,btDefaultName) !=0)
        {
  #if SerialDebug == true
  Serial.println("Change name");
  #endif
  		  char name[20];
		  strcpy_P(name, btDefaultName);
          // not equal, set the default name
          boolean success= atCommand(NAME,name);
          if(success)
          {
          	strcpy(btNetworkName,name);
          }
        }
    }
    else
    {
    	btNetworkName[0]='\0';
    }

    char *buff= btPinCode; // user pin code as temperaturally buffer

    ret=atQuery(TYPE,buff,20,true);
    if(ret>0)
    {
        // there should only be one
        btSecurityType = buff[0] - '0';
  #if SerialDebug == true
  Serial.print("connection type:");
  Serial.println(btSecurityType);
  #endif
        
    }
      // get PIN code & connect type
    ret=atQuery(PASS,btPinCode,20,true);
    if(ret > 0)
    {
        if(ret >19) ret=19;
        btPinCode[ret]=0;
  #if SerialDebug == true
  Serial.print("PASS Code:");
  Serial.println(btPinCode);
  #endif

    }
}
#endif

//********************************************
// remote command handling

byte _currentState;
int _duration;

//$V[CMD],[p1],[p2],[p3],[…],[pn]*[cksum]\r\n

byte _btRecevingState;

#define BR_Start 0
#define BR_ID    1
#define BR_Data 2
#define BR_Return 3

#define BR_StateIs(s) (_btRecevingState == s)
#define BR_StateSet(s)  _btRecevingState =s
#define BR_NextState _btRecevingState++
#define BR_StateReset _btRecevingState=BR_Start
#define BluetoothCommadBufferSize 64
#define BluetoothMaximumCommandLength 48
#define BtCommandCompress(a,b) (((int)(a) << 8) | (int)(b))

#define BtCmdButtonPressed BtCommandCompress('K','P')
#define BtCmdBrewSet       BtCommandCompress('B','S')
#define BtCmdBoilHops          BtCommandCompress('B','H')
#define BtCmdPersistenceRead  BtCommandCompress('P','R')
#define BtCmdPersistenceSet   BtCommandCompress('P','S')
#define BtCmdHello   BtCommandCompress('B','M')

#define BtCmdPersistenceBlockRead  BtCommandCompress('P','B')
//#define BtCmdPersistenceBlockData   BtCommandCompress('P','D')
#define BtCmdDisconnect  BtCommandCompress('D','C')


//#define BtCmdQueryStatus BtCommandCompress('Q','S')
#define BtCmdQueryBrewInfo BtCommandCompress('Q','B')
#define BtCmdQueryBoilHopTime BtCommandCompress('Q','H')


#if BT_DEBUGTEMP == true
#define BtCmdSetTemperature   BtCommandCompress('T','S')
#endif


void btProcessSentence(void);

char _cmd[BluetoothCommadBufferSize];
byte _bufferIndex;
byte _processIndex;

void btReceive(void)
{
  while(btSerial.available())
  {
    char ch=btSerial.read();

#if SerialDebug == true
	Serial.print(ch);
#endif

    if(BR_StateIs(BR_Start))
    {
      if(ch == '$') BR_NextState;
    }
    else if(BR_StateIs(BR_ID))
    {
      if(ch != 'V') BR_StateReset;
      else
      {
        BR_NextState;
        _bufferIndex=0;
      } 
    }
    else if(BR_StateIs(BR_Data))
    {
      if(ch == '\r') 
        BR_NextState;
      else
      {
        _cmd[_bufferIndex]=ch;
        _bufferIndex++;
        if(_bufferIndex > BluetoothMaximumCommandLength)
        {
            BR_StateReset;
        }
      }
    }
    else if(BR_StateIs(BR_Return))
    {
      if(ch == '\n')
      {
        btProcessSentence();
      }
      BR_StateReset;
    }
#if 0
    Serial.print(F("ch="));
    Serial.print(ch);
    Serial.print(F(", state="));
    Serial.println(_btRecevingState);
#endif
  }  // while(has data)
} //void btReceive(void)

float readNextFloat(void)
{
	float ret=0.0;
  	float decimal=0.0;  
  	
  	if(_cmd[_processIndex] == ',') _processIndex++;

  	while(_cmd[_processIndex] != ','
       	&& _processIndex < _bufferIndex)
  	{
    	if(_cmd[_processIndex] == '.')
      		decimal=10.0;
    	else
    	{
      		if (decimal > 1.0)
      		{ 
         		ret += (_cmd[_processIndex] - '0')/decimal;
         		decimal = decimal * 10.0;
     		}
      		else
      		{
        		ret = ret * 10.0 + (_cmd[_processIndex] - '0');
      		}
  		}
      	_processIndex++;
	}
	return ret;
}

int readNextInt(void)
{
  int ret=0;

#if 0  
  Serial.print("readNextInt-");
  Serial.print(_processIndex);
  Serial.print(",");
  Serial.println(_bufferIndex);
#endif
  
  if(_cmd[_processIndex] == ',') _processIndex++;
  
  while(_cmd[_processIndex] != ','
       && _processIndex < _bufferIndex)
  {
      ret = ret * 10 + (_cmd[_processIndex] - '0');
      _processIndex++;
  }  
  return ret;
}

byte toHexValue(char hex)
{
  if(hex >='0' && hex <='9') return hex - '0';
  // else 
  return (hex - 'A' +10);
}
// $VKP,[mask],[0/1]*cksum
void   btProcessButtonPressed(void)
{
  int mask=readNextInt();
  boolean longpressed=(boolean)readNextInt();
  // call
#if 0
  Serial.print("btProcessButtonPressed");
  Serial.print(mask);
  Serial.print(" ,longpressed=");
  Serial.println(longpressed);
#endif
  
  virtualButtonPress(mask,longpressed);
}

void btReportBrewStage(byte stage)
{
	byte time=readSetting(PS_StageTimeAddr(stage));
	float temp;
	if(time ==0) temp =0;
	else temp=TempFromStorage(readSettingWord(PS_StageTemperatureAddr(stage)));

#if NoPrint == true
	btSentenceStart('B','I');
	btSentenceInt(stage);
	btSentenceComma();
	btSentenceFloat(temp);
	btSentenceComma();
	btSentenceInt(time);
	btSentenceSend();
#else
	btSerial.print(F("$VBI,"));
	btSerial.print(stage);			
	btSerial.print(",");			
	btSerial.print(temp);			
	btSerial.print(",");
	btSerial.println(time);		
#endif
}

// $VBS,[stage],[temp/float],[time]
void   btProcessBrewSet(void)
{
	byte stage;
	float temp;
	byte time;

	while(_processIndex < _bufferIndex)
	{
		
  		stage=readNextInt();
  		temp=readNextFloat();
  		time=readNextInt();
  		
  		if(stage <=7)
  		{
			if(stage ==0) time=1; // always use 1 as MashIn time
			else if(stage >= 6)  // stage 6 & 7 are mandatory
			{	
				if (time ==0)	time =1;
			}
  			updateSetting(PS_StageTimeAddr(stage),time);
  			if(time > 0 || stage ==0)
  				updateSettingWord(PS_StageTemperatureAddr(stage),ToTempInStorage(temp));

		}
	}
	btReportBrewStage(stage);
}
void btProcessQueryBoilHopTime();
//$VBH,60,40,20,10*cksum
void   btProcessBoilHops(void)
{
	// Boil time is
	//#define PS_BoilTime     73    //   Boil Time 
	//#define PS_HopTimeBase 74

  byte num=0;
  while(_processIndex < _bufferIndex)
  {
    int hopTime=readNextInt();
    // write hop time
    updateSetting(PS_BoilTime+num,hopTime);
    num++;
  }
  // write number of hop
  updateSetting(PS_NumberOfHops,num-1);
  btProcessQueryBoilHopTime();
}


// int, int
void  btConfirmPersistence(int addr, int size,int value)
{
#if 0
  Serial.print("btConfirmPersistence addr=");
  Serial.print(addr);
  Serial.print(" ,size=");
  Serial.print(dataSize);
  Serial.print(" ,value=");
  Serial.println(value);
#endif  

	#if NoPrint == true
	btSentenceStart('P','V');
	btSentenceInt(addr);
	btSentenceComma();
	btSentenceInt(value);
	btSentenceSend();
	#else
  	btSerial.print(F("$VPV,"));
  	btSerial.print(addr);
  	btSerial.print(F(","));
  	btSerial.println(value);
	#endif
}


//$VPB,[addr],[number]*cksum
void   btProcessPersistenceBlockRead(void)
{
  	int addr=readNextInt();
  	int number= readNextInt();
  	int value;
  
	btSentenceStart('P','D');
	btSentenceInt(addr);
  
  	byte i;
  	for(i=0;i< number;i++)
  	{
  		value=(int)readSetting(addr+i);
			
		btSentenceComma();
		btSentenceInt(value);
  	}
	btSentenceSend();

}


//$VPR,[addr],[size]*cksum
void   btProcessPersistenceRead(void)
{
  int addr=readNextInt();
  int dataSize= readNextInt();
  int value;
  
  
  if(dataSize == 1)
  {
  	value=(int)readSetting(addr);
  }
  else if(dataSize == 2)
  {
  	value=readSettingWord(addr);
  }
	btConfirmPersistence(addr,dataSize,value);
}

//$VPR,[addr],[size],[value]*cksum
void   btProcessPersistenceSet(void)
{
  int addr=readNextInt();
  int dataSize= readNextInt();
  int value = readNextInt();
  if(dataSize == 1)
  {
  	updateSetting(addr,(byte)value);
  }
  else if(dataSize == 2)
  {
  	updateSettingWord(addr,value);
  }

	btConfirmPersistence(addr,dataSize,value); 
}
void btProcessDisconnect(void)
{
	gIsConnected=false;
	uiSetWirelessStatus(WiStateNotConnected);
}
//$VHB,[software name]
// int,int, string
void   btProcessHello(void)
{
	#if NoPrint == true  
	btSentenceStart('B','M');
	btSentenceInt(_currentState);
	btSentenceComma();
	int left=(int)(getTimeLeft()/1000);	
	btSentenceInt(left);
	btSentenceComma();
	btSentenceFloat(0.4);
	btSentenceSend();
	#else
  	btSerial.print(F("$VBM,"));
  	btSerial.print(_currentState);
	btSerial.print(",");				
	int left=(int)(getTimeLeft()/1000);	
	btSerial.print(left);	
	btSerial.println(",0.4"); //version
	#endif
  	gIsConnected = true;
  	uiSetWirelessStatus(WiStateConnected);
}

// int,float,int
void btProcessQueryBrewInfo(void)
{
	byte stage=readNextInt();
	
	btReportBrewStage(stage);
}

// int,int,...
void btProcessQueryBoilHopTime(void)
{
	//#define PS_BoilTime     73    //   Boil Time 
	//#define PS_HopTimeBase 74

#if NoPrint == true
	btSentenceStart('B','H');
    byte time=readSetting(PS_BoilTime);
	btSentenceInt(time);
#else
	btSerial.print(F("$VBH"));
#endif
	byte num=readSetting(PS_NumberOfHops);
	byte i=0;
	while(i < num)
  	{
#if NoPrint == true 
    	byte time=readSetting(PS_HopTimeBase+i);
		btSentenceComma();
		btSentenceInt(time);
#else
    	byte time=readSetting(PS_BoilTime+i);
    	btSerial.print(",");
    	btSerial.print(time);    	
#endif
    	i++;
  	}
#if NoPrint == true 
	btSentenceSend();
#else
  	btSerial.println("");
#endif
}

 
 
#if BT_DEBUGTEMP == true
void btProcessSetTemperature(void)
{
	float temp=readNextFloat();
	gCurrentTemperature=temp;
}
#endif
 
void btProcessSentence(void)
{
  boolean hasChecksum=false;
  byte chksum;
  // checksum
  if( _bufferIndex > 3
    && _cmd[_bufferIndex - 3] == '*')
  {
     // there is checksum
     chksum= (toHexValue(_cmd[_bufferIndex - 2]) << 4) + toHexValue(_cmd[_bufferIndex - 1] );
     
     // the checksum calulate V
     chksum ^='V';
     for(byte i=0;i < (_bufferIndex - 3);i++)
     {
       chksum ^= _cmd[i];
     }
     if(chksum != 0) return;
     _bufferIndex -= 3;
     hasChecksum=true;
  }

#if 1 // always checksum BT_STRICT == true
  if(_cmd[2] != ',') return;
  if(!hasChecksum || chksum !=0) return;
#endif


  // process command.
  int commad=BtCommandCompress(_cmd[0],_cmd[1]);


  _processIndex = 3;
  
  if(commad == BtCmdButtonPressed)
  {
    btProcessButtonPressed();
  }
  else if(commad == BtCmdBrewSet)
  {
    btProcessBrewSet();
  }
  else if(commad == BtCmdBoilHops)
  {
    btProcessBoilHops();
  }
  else if(commad == BtCmdPersistenceRead)
  {
    btProcessPersistenceRead();
  }
  else if(commad == BtCmdPersistenceSet)
  {
    btProcessPersistenceSet();
  }
  else if(commad == BtCmdHello)
  {
   	btProcessHello();
  }
  else if(commad == BtCmdQueryBrewInfo)
  {
  	btProcessQueryBrewInfo();
  }
  else if(commad == BtCmdQueryBoilHopTime)
  {
    btProcessQueryBoilHopTime();
  }
  else if(commad == BtCmdPersistenceBlockRead)
  {
  	btProcessPersistenceBlockRead();
  }
  else if(commad == BtCmdDisconnect)
  {
  	btProcessDisconnect();
  }
#if BT_DEBUGTEMP == true
  else if(commad == BtCmdSetTemperature)
  {
   	btProcessSetTemperature();
  }	
#endif
}

void btInitialize(void)
{

   btTestAvailable();
   BR_StateReset;
   
   // if connected, check setting or issue command to set 
   if(gIsBtModulePresent)
   {
   		uiSetWirelessStatus(WiStateNotConnected);

	#if  BT_MODULE_INITIALIZATION == true
		btSetupBtModule();
	#endif
   }
   else
   {
	   	uiSetWirelessStatus(WiStateNoModule);
   }
}

unsigned long _btLastReport;

// float

void btReportTemperature(void)
{
	if((gCurrentTimeInMS - _btLastReport) >BT_TemperatureReportPeriod)
	{
	
#if NoPrint == true
		btSentenceStart('S','T');
		btSentenceFloat(gCurrentTemperature);
		btSentenceSend();
#else
		btSerial.print(F("$VST,"));
		btSerial.println(gCurrentTemperature);
#endif

		_btLastReport = gCurrentTimeInMS;

	}
}

void btThread(void)
{
	if(gIsBtModulePresent)
	{
		btReceive();
		
		if(gIsConnected) btReportTemperature();
	}
}
// reporting function

void btReportHeater(byte value)
{
	if(!gIsConnected) return;

	#if NoPrint == true
	btSentenceInt('S','H',value);
	#else
	btSerial.print(F("$VSH,"));
	btSerial.println(value);
	#endif
}

void btReportPump(byte value)
{
	if(!gIsConnected) return;
	
	#if NoPrint == true
	btSentenceInt('S','P',value);
	#else

	btSerial.print(F("$VSP,"));
	btSerial.println(value);
	#endif
}


void btReportCurrentStage(byte stage)
{
	_currentState=stage;

	if(gIsConnected)
	{
		#if NoPrint == true
		btSentenceInt('B','S',_currentState);
		#else

		btSerial.print(F("$VBS,"));
		btSerial.println(_currentState);
		#endif
	}
}

void btReportEvent(byte event)
{
	if(!gIsConnected) return;
	
	#if NoPrint == true
	btSentenceInt('A','L',event);
	#else

	btSerial.print(F("$VAL,"));
	btSerial.println(event);	
	#endif
}

void btReportSettingTemperature(void)
{
	if(!gIsConnected) return;

#if NoPrint == true
	btSentenceStart('S','S');
	btSentenceFloat(gSettingTemperature);
	btSentenceSend();
#else

	btSerial.print(F("$VSS,"));
	btSerial.println(gSettingTemperature);	
#endif

}

void btReportPwm(void)
{
	if(!gIsConnected) return;

	#if NoPrint == true
	btSentenceInt('S','W',gBoilHeatOutput);
	#else
	btSerial.print(F("$VSW,"));
	btSerial.println(gBoilHeatOutput);	
	#endif
}

#if PerformanceProfiling == true

void btReportDebugInfo(int info)
{
	if(!gIsConnected) return;

	btSentenceInt('D','B',info);
}

#endif

#endif  