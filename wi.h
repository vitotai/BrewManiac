#ifndef WI_H
#define WI_H

#define RetransmissionTimerValue 1500

#define VERSION 0x01
#define MAX_RETRY 5



#if SerialDebug != true
#define DEBUG_WIFI  false
#else
#define DEBUG_WIFI  true
#endif

#if DEBUG_WIFI == true
#define DPRINT_WIFI(s) Serial.print(s)
#define DPRINT_WIFI_H(s) Serial.print(s,HEX)
#define DPRINTLN_WIFI(s) Serial.println(s)
#else
#define DPRINT_WIFI(s) 
#define DPRINT_WIFI_H(s)
#define DPRINTLN_WIFI(s)
#endif


//#define WifiReportPeriod 5000
// packet format
#define StartOfFrame 0x81
#define CammandMask 0x01F


// message code
#define BT(c) ((c)|0x20)
#define WiControllerAck BT(1)
#define WiConnectRequest BT(2)
#define WiButtonPressed BT(3)
#define WiSetMashSchedule BT(4)
#define WiSetBoilHopsTime  BT(5)
#define WiPersistenceRead BT(6)
#define WiPersistenceBlockRead BT(7)
#define WiPersistenceSet BT(8)
#define WiDisconnect BT(9)
#define WiDeviceAddress BT(10)
#define WiSetRecipe BT(11)
#define WiQueryRecipe BT(12)

#define BO(c) (c)
#define WiBMAck BO(1)
#define WiConectionConfirm BO(2)
#define WiStatus BO(3)
#define WiEventNotification BO(4)
#define WiSetTemperature BO(5)
#define WiSetPwm BO(6)
#define WiPersistenceValue BO(7)
#define WiPersistenceBlockData BO(8)
#define WiRecipeInformation BO(9)
#define WiButtonLabel BO(10)

///
#define RequestIndicationSequence(n) (((n)&0xF))
#define ResponseSequence(n) (((n)&0xF)|0x10)
#define RoundSequenceNumber(n)  ((n) & 0xF)
#define IsAckResponse(n) (((n)&0xF0) == 0x10)
#define PacketSequence(n) ((n)&0xF)
//PDU format
#define WiMessageCode(c) ((c)&0x3F)
#define NullPDU(c) (c)
#define OnePDU(c)  ((c) | (0x1<<6))
#define VarPDU(c) ((c) |  (0x2<<6))

#define IsNullPDU(c) (((c)&0xC0) ==0)
#define IsOnePDU(c) (((c)&0xC0) ==0x40)
#define IsVarPDU(c) (((c)&0xC0) ==0x80)


byte _currentState;
bool gIsWifiConnected;
byte _pumpStatus;
byte _heaterStatus;
unsigned long _wiLastReport;
//wifi

#if UseSoftwareSerial == true

SoftwareSerial swSerial(SoftwareSerialRx,SoftwareSerialTx);

#define WifiWrite swSerial.write
#define WifiAvailable swSerial.available
#define WifiRead swSerial.read
#define WifiPortBegin	swSerial.begin

#else

#define WifiWrite WirelessHardwarePort.write
#define WifiAvailable WirelessHardwarePort.available
#define WifiRead WirelessHardwarePort.read
#define WifiPortBegin	WirelessHardwarePort.begin

#endif

/*******************************************/
/*  for sending buffer
/*******************************************/
unsigned long _retransTimer;
#define IsRetransTimerRunning (_retransTimer!=0)
#define StartRetransTimer(a) _retransTimer=millis() + a
#define StopRetransTimer _retransTimer=0
#define IsRetransTimerExpiry ((_retransTimer!=0)&& ( millis() > _retransTimer))

#define WiStart(r,v) if(r) wiResponseStart(v); else wiIndicationStart(v);
#define WiPut(r,v) if(r) wiResponsePut(v); else wiIndicationPut(v);
#define WiEnd(r) if(r) wiResponseEnd(); else wiIndicationEnd();
#define WiPutInt(r,v) if(r) wiResponsePutInt(v); else wiIndicationPutInt(v);

byte _txChecksum;
byte _sendNumber;
byte _rcvNumber;
byte _retryCount;
#define QueueBufferSize 128

byte _l2QueueBuffer[QueueBufferSize];
byte _l2WritePtr;
byte _l2CurrentPduPtr;

void wiL2Init(void)
{
	_retransTimer =0;
	_retryCount=0;

	_sendNumber=0;
	_rcvNumber=0;
	_l2WritePtr=0;
	_l2CurrentPduPtr=0;
}

void wiSerialDisconnected(void)
{
	gIsWifiConnected=false;
	uiSetWirelessStatus(WiStateSerialDisconnected);
	wiL2Init();
	if(_currentState == StageIdleScreen)
		uiClearIpAddress();
	DPRINTLN_WIFI("DISC");
}



void writeTxBuffer(byte ch)
{
	_l2QueueBuffer[_l2WritePtr] = ch;
		
	_l2WritePtr++;
	if(_l2WritePtr == QueueBufferSize) _l2WritePtr=0;
	if(_l2WritePtr == _l2CurrentPduPtr){
			// overlap. problem here
			// either NO response from peer
			// or the queue buffer is too small
		DPRINTLN_WIFI("Fatal:buf overrun");
		// disconnect
		wiSerialDisconnected();
	}
}

void wiSendFromQueueBuffer(void)
{
	if(_l2CurrentPduPtr == _l2WritePtr) return; // empty
	// check current pdu type
	byte ptr = _l2CurrentPduPtr;
	
	byte checksum=0;
	// start sending
	WifiWrite(StartOfFrame);

	// sequence number
	byte sqn=RequestIndicationSequence(_sendNumber);
	WifiWrite(sqn);
	checksum ^= sqn;

	// message header
	byte ch = _l2QueueBuffer[ptr++];
	DPRINT_WIFI("Send Msg:");
	DPRINTLN_WIFI(ch);
	if(ptr == QueueBufferSize) ptr =0;
	checksum ^= ch;
	WifiWrite(ch);
	// length, if any
	byte len;
	if(IsNullPDU(ch)){
		len=0;
	}else if(IsOnePDU(ch)){
		len = 1;
	}else if(IsVarPDU(ch)){
		len = _l2QueueBuffer[ptr++];
		if(ptr == QueueBufferSize) ptr =0;
		WifiWrite(len);
		
		checksum ^= len;
	}


	for(int i=0;i<len;i++)
	{
		byte data=_l2QueueBuffer[ptr];
		ptr ++; 
		if(ptr == QueueBufferSize) ptr =0;
		
		checksum ^= data;
		WifiWrite(data);
	}
	WifiWrite(checksum);	
	StartRetransTimer(RetransmissionTimerValue);
}
void wiDiscardCurrentPdu(void)
{
	// check current pdu type
	byte ptr = _l2CurrentPduPtr;
	byte ch = _l2QueueBuffer[ptr++];
	if(ptr == QueueBufferSize) ptr =0;
	
	byte size;

	if(IsNullPDU(ch)){
		size=0;
	}else if(IsOnePDU(ch)){
		size = 1;
	}else if(IsVarPDU(ch)){
		size = _l2QueueBuffer[ptr++];
		// let it be done in the following code: if(ptr == QueueBufferSize) ptr =0;
	}
	ptr += size;
	if(ptr >= QueueBufferSize) ptr -=QueueBufferSize;
	_l2CurrentPduPtr=ptr;
}

void wiIndicationPut(byte d)
{
	writeTxBuffer(d);
}

void wiIndicationStart(byte header)
{
	writeTxBuffer(header);
}


void wiIndicationPutInt(int value)
{
	writeTxBuffer((byte)( value >> 8));
	writeTxBuffer((byte)( value & 0xFF));
}

void wiIndicationEnd(void)
{
	// end from buffer
	if(! IsRetransTimerRunning)
	{
		wiSendFromQueueBuffer();
	}
}

void wiResponseStart(byte header)
{
	WifiWrite(StartOfFrame);

	_txChecksum =0;	
	byte s=ResponseSequence(_rcvNumber);
	_txChecksum ^= s;
	WifiWrite(s);
	_txChecksum ^=header;
	WifiWrite(header);
}
void wiResponsePut(byte d)
{
	_txChecksum ^= d;
	WifiWrite(d);
}

void wiResponsePutInt(int value)
{
	wiResponsePut((byte)( value >> 8));
	wiResponsePut((byte)( value & 0xFF));
}

void wiResponseEnd(void)
{
	WifiWrite(_txChecksum);
}
/*******************************************/


void wiSendAck(void)
{
	wiResponseStart(NullPDU(WiBMAck));
	wiResponseEnd();
}

#define StatusLen 6
void wiSendStatus(void)
{
	_wiLastReport = gCurrentTimeInMS;

	// code
	wiIndicationStart(VarPDU(WiStatus));
	// len
	wiIndicationPut(StatusLen);

	byte counting;
	int countingTime=getDisplayTime(&counting);

	bool paused=(counting == COUNTING_PAUSE);
	byte countUp=(counting == COUNTING_UP)? 1:0;
	
	byte trBit=(gIsTemperatureReached)? 1:0;
	byte pwmBit=(_isEnterPwm)? 1:0;
	byte pauseBit=(paused)? 1:0;
	// status
	byte status= (_pumpStatus & 0x3)
				| ((_heaterStatus & 0x3)<<2)
 				| ((pwmBit & 0x01)<<4)
				| ((trBit & 0x1)<<5)
			 	| ((pauseBit & 0x1)<<6)
			 	| ((countUp & 0x1)<<7);
	wiIndicationPut(status);
	//stage
	wiIndicationPut(_currentState);
	// current temp.
	wiIndicationPutInt(ToTempInStorage(gCurrentTemperature));
	// current running timer
	wiIndicationPutInt(countingTime);	
	wiIndicationEnd();
}

byte _latestLabelId;
void wiSendButtonLabel(const byte labelId)
{
	_latestLabelId=labelId;
	if(!gIsWifiConnected) return;

	wiIndicationStart(OnePDU(WiButtonLabel));
	wiIndicationPut(labelId);
	wiIndicationEnd();	
}


void wiReportCurrentStage(byte stage)
{
	_currentState=stage;
	if(!gIsWifiConnected) return;

	wiSendStatus();
}
void wiReportHeater(byte value)
{
	_heaterStatus=value;
	if(!gIsWifiConnected) return;

	wiSendStatus();
}

void wiReportPump(byte value)
{
	_pumpStatus=value;
	if(!gIsWifiConnected) return;

	wiSendStatus();
}

void wiReportEvent(byte event)
{
	if(!gIsWifiConnected) return;

	wiIndicationStart(OnePDU(WiEventNotification));
	wiIndicationPut(event);
	wiIndicationEnd();
}

void wiReportPwm(void)
{
	if(!gIsWifiConnected) return;

	wiIndicationStart(OnePDU(WiSetPwm));
	wiIndicationPut(gBoilHeatOutput);
	wiIndicationEnd();
}

void wiReportSettingTemperature()
{
	if(!gIsWifiConnected) return;
	wiIndicationStart(VarPDU(WiSetTemperature));
	wiIndicationPut(2);// length

	wiIndicationPutInt(ToTempInStorage(gSettingTemperature));
	wiIndicationEnd();
}

void wiTogglePwm(void)
{
	if(!gIsWifiConnected) return;
	wiSendStatus();
}
unsigned long _wifiReportPeriod;

void wiStatusReport(void)
{
	if((gCurrentTimeInMS - _wiLastReport) > _wifiReportPeriod)
	{
		wiSendStatus();
	}
}
#define BufferSize 64
byte _wiBuffer[BufferSize];
byte * _wiPtr;
byte _wiState;
byte _wiLength;
byte _sizeToRead;
byte _messageCode;
byte _checksum;

byte _frameSequence;
byte _isResponse;
#define WSWaitSOF 0
#define WSWaitSequence 1
#define WSMessageCode 2
#define WSLength 3
#define WSData 4
#define WSChecksum 5


void wiControllerAck(void)
{
	DPRINT_WIFI("C_AcK:");
	DPRINTLN_WIFI(_frameSequence);
}

void wiButtonPressed(bool process)
{
	if(process)
	{
		byte msg=_wiBuffer[0];
		virtualButtonPress(msg & 0xF, (msg & 0x10)!=0 );
	}
	wiSendAck();
}

void wiConnectRequest(void)
{
	gIsWifiConnected=true;
	uiSetWirelessStatus(WiStateConnected);
	_rcvNumber=0;
	_sendNumber=0;
	_wifiReportPeriod=(unsigned long)_wiBuffer[0] *1000;
	// response
	wiResponseStart(VarPDU(WiConectionConfirm));
	wiResponsePut(2); // length
	wiResponsePut(VERSION);
	wiResponsePut(_latestLabelId);
	wiResponseEnd();
}

void wiSendRecipe(bool response)
{
	if(!gIsWifiConnected) return;
	// first step is to know the length
	byte len=4; // always send mash-in
	byte stage;

	byte tNum=readSetting(PS_NumberOfHops) +1; // plus Boil Time
	len += tNum;

#if SimpleMashStep == true
	bool mashend=false;
	for(stage=1;stage<8;stage++)
	{
		byte time=readSetting(PS_StageTimeAddr(stage));
		if(mashend && stage < 7){
			len ++;
		}else{
			len += (time==0)? 1:4;
			if(time ==0){
				mashend=true;
			}
		}
	}
	
	// start response
	WiStart(response,VarPDU(WiRecipeInformation))
	WiPut(response,len)

	mashend=false;
	for(stage=0;stage<8;stage++)
	{
		byte time=readSetting(PS_StageTimeAddr(stage));
		if(stage ==0) time =1;

		if((mashend && stage < 7) ||(time==0))
		{
			byte stageByte= 0xF0 | stage;
			WiPut(response,stageByte)
			mashend = true;
		}
		else
		{
			int temp=readSettingWord(PS_StageTemperatureAddr(stage));

			WiPut(response,stage)
			WiPut(response,time)
			WiPutInt(response,temp)
		}
	}
#else //#if SimpleMashStep == true
	for(stage=1;stage<8;stage++)
	{
		byte time=readSetting(PS_StageTimeAddr(stage));
		len += (time==0)? 1:4;
	}
	
	// start response
	WiStart(response,VarPDU(WiRecipeInformation))

	WiPut(response,len)
	
	for(stage=0;stage<8;stage++)
	{
		byte time=readSetting(PS_StageTimeAddr(stage));
		if(stage ==0) time =1;
		if(time==0)
		{
			byte stageByte= 0xF0 | stage;
			WiPut(response,stageByte)
		}
		else
		{
			int temp=readSettingWord(PS_StageTemperatureAddr(stage));

			WiPut(response,stage)
			WiPut(response,time)
			WiPutInt(response,temp)
		}
	}
#endif //#if SimpleMashStep == true

	// finish mash schedule. now boil & hop
	
	byte i=0;
	while(i < tNum)
  	{
    	byte time=readSetting(PS_BoilTime+i);
    	WiPut(response,time)
    	i++;
  	}
  	WiEnd(response)
}

void wiQueryRecipe(void)
{
	wiSendRecipe(true);
}

void wiSetRecipe(bool process)
{
	if(process)
	{
	byte idx=0;	
	byte stage;
	
	do{
		byte stageByte= _wiBuffer[idx];
		stage= stageByte & 0x0F;
		
		if((_wiBuffer[idx] & 0xF0) == 0)
		{
			byte time=_wiBuffer[idx+1];
			if(stage ==0) time=1;
			else if(stage >=6 && time==0) time=1;
			
			int temperature = (((int)_wiBuffer[idx+2]) << 8) 
							| (int)_wiBuffer[idx+3];
  			updateSettingWord(PS_StageTemperatureAddr(stage),temperature);
  			updateSetting(PS_StageTimeAddr(stage),time);
  			
  			idx +=4;
		}
		else
		{
			// empty one
			if(stage != 0)
	  			updateSetting(PS_StageTimeAddr(stage),0);
  			idx ++;
		}
	}while(idx < _wiLength && stage < 7);
	
	// boil & hop
	byte tNum=0;
  	while(idx < _wiLength)
  	{
    	byte hopTime=_wiBuffer[idx];
    	// write hop time
    	updateSetting(PS_BoilTime+tNum,hopTime);
    	tNum++;
    	idx++;
  	}
  	// write number of hop
  	updateSetting(PS_NumberOfHops,tNum-1);
  	// response
 	}
 	wiSendRecipe(true);
}


void wiSendPersistenceValue(bool response,int address,byte value)
{
	if(!gIsWifiConnected) return;

	byte len;
	if(address > 0x7F) len=3;
	else len=2;

	WiStart(response,VarPDU(WiPersistenceValue))
	WiPut(response,len)

	byte addr1=address & 0x7F;

	if(address > 0x7F) 
	{
		addr1 != 0x80;
		
		WiPut(response,addr1)

		byte addr2 =(byte) (address > 7);
		WiPut(response,addr2)
	}
	else
	{
		WiPut(response,addr1)
	}
	
	WiPut(response,value)

	WiEnd(response)
}

byte readAddress(byte *buf,int *address)
{
	byte v=buf[0];
	*address = (int)(v & 0x7F);
	if( (v & 0x80))
	{
		*address = *address + ((buf[1] & 0x7F) << 7);
		return 2;
	}
	else
	{
		return 1;
	}
}


void wiPersistenceRead(void)
{
	int address;
	readAddress(_wiBuffer,&address);
	byte value=readSetting(address);
	wiSendPersistenceValue(true,address,value);
}

void wiPersistenceSet(bool process)
{
	int address;
	byte value;
	int idx=0;
	if(process)
	{
		while(idx < _wiLength)
		{
			idx +=readAddress(_wiBuffer+idx,&address);
			value=_wiBuffer[idx++];
			updateSetting(address,value);
			// temperature setting must be handled.
			// either another "command" set, or
			// handled here.
			if(address ==PS_TempUnit)
			{
				if(gIsUseFahrenheit != (boolean)value)
				{
					uiChangeTemperatureUnit((boolean)value);
					gIsUseFahrenheit = (boolean)value;
				}
			}
		}
	}
	//response
	
	// echo!
	// code
	wiResponseStart(VarPDU(WiPersistenceValue));
	// lenth
	wiResponsePut(_wiLength);
	for(byte i=0;i<_wiLength;i++)
	{
		wiResponsePut(_wiBuffer[i]);
	}
	wiResponseEnd();
}

void wiSettingChanged(int address,byte value)
{
	wiSendPersistenceValue(false,address,value);
}

void wiPersistenceBlockRead(void)
{
	int address;
	byte idx=readAddress(_wiBuffer,&address);
	byte size=_wiBuffer[idx];
	byte len = idx + size;
	
	// code
	wiResponseStart(VarPDU(WiPersistenceBlockData));

	wiResponsePut(len);
	//address, may be one or two bytes
	// whatever it is, copy the original data
	wiResponsePut(_wiBuffer[0]);
	if(idx ==2){ // if two byte address. copy
		wiResponsePut(_wiBuffer[1]);
	}
	
	for(byte i=0;i<size;i++)
	{
		byte value=readSetting(address+i);
		wiResponsePut(value);
	}
	
	wiResponseEnd();
}



void wiDeviceAddress(void)
{
	wiSendAck();
	// support only IPv4 for now
	if(	_wiBuffer[0] == 1 &&  _wiLength ==5){
		uiSetIp(_wiBuffer +1);
		if(_currentState == StageIdleScreen)
			uiPrintIpAddress();

	}else if (	_wiBuffer[0] == 0){ // clear
		uiSetWirelessStatus(WiStateNotConnected);
		byte zIp[4]={0,0,0,0};
		uiSetIp(zIp);
		if(_currentState == StageIdleScreen)		
			uiClearIpAddress();
	}
}

void wiProcessMsg(void){
	bool inSequence;

	if(_isResponse){
		// response// ack
		wiDiscardCurrentPdu();
		StopRetransTimer;
		_retryCount=0;
		_sendNumber = RoundSequenceNumber(_sendNumber+1);
		wiSendFromQueueBuffer();
	}else{

		if(_frameSequence == _rcvNumber){
			inSequence=true;
			DPRINTLN_WIFI("in sequence");
			_rcvNumber =RoundSequenceNumber(_rcvNumber+1);
		}else{
			inSequence =false;
			DPRINT_WIFI("out of sequence:");
			DPRINT_WIFI(_frameSequence);
			DPRINT_WIFI(" Expected:");
			DPRINT_WIFI(_rcvNumber);
		}
	}
	
	if(_messageCode == WiConnectRequest){
		DPRINTLN_WIFI("ConnectRequest");
		// response only
		wiConnectRequest();
	}else if(_messageCode == WiButtonPressed){
		DPRINTLN_WIFI("ButtonPressed");
		wiButtonPressed(inSequence);
	}else if(_messageCode == WiPersistenceRead){
		DPRINTLN_WIFI("PersistenceRead");
		//always response
		wiPersistenceRead();
	}else if(_messageCode == WiPersistenceBlockRead){
		DPRINTLN_WIFI("PersistenceBlockRead");
		// response only
		wiPersistenceBlockRead();
	}else if(_messageCode == WiPersistenceSet){
		DPRINTLN_WIFI("PersistenceSet");
		// always response, but only set when in sequence
		wiPersistenceSet(inSequence);
		
	}else if(_messageCode == WiQueryRecipe){
		DPRINTLN_WIFI("QueryRecipe");
		// always response
		wiQueryRecipe();
	}else if(_messageCode == WiSetRecipe){
		DPRINTLN_WIFI("SetRecipe");
		// only response when in sequence
		wiSetRecipe(inSequence);	
	}else if(_messageCode == WiDeviceAddress){
		DPRINTLN_WIFI("DeviceAddress");
		wiDeviceAddress();
	}else if(_messageCode == WiControllerAck){
		DPRINTLN_WIFI("ControllerAck");
		wiControllerAck();
	}
}

void wiReceive(void)
{
	while(WifiAvailable())
	{
		byte ch=WifiRead();
		
/*		DPRINT_WIFI("rcv:");
		DPRINT_WIFI_H(ch);
		DPRINT_WIFI(" _wiState:");
		DPRINTLN_WIFI(_wiState);
*/		
		if(_wiState == WSWaitSOF){
			if(ch == StartOfFrame){
				_wiState = WSWaitSequence;
			}
		}else if(_wiState == WSWaitSequence){
			_checksum = ch;
			_frameSequence = PacketSequence(ch);
			_isResponse = IsAckResponse(ch);
			_wiState = WSMessageCode;
		}else if(_wiState == WSMessageCode){
			_checksum ^= ch;
			_messageCode = WiMessageCode(ch);
			if(IsNullPDU(ch)){
				_wiState=WSChecksum;
			}else if(IsOnePDU(ch)){
				_sizeToRead=_wiLength = 1;
				_wiState = WSData;
			}else if(IsVarPDU(ch)){
				_wiState = WSLength;
			}
			_wiPtr=_wiBuffer;
			
		}else if(_wiState == WSLength){
			_checksum ^= ch;
			_sizeToRead=_wiLength = ch;
			if(_wiLength > BufferSize) _wiState=WSWaitSOF;
			else _wiState=WSData;
		}else if(_wiState == WSData){
			_checksum ^= ch;
			*_wiPtr = ch;
			_wiPtr ++;
			_sizeToRead --;
			if(_sizeToRead ==0){
				_wiState=WSChecksum;
			}
		}else /*if(_wiState == WSChecksum)*/{
			if(ch == _checksum){
				wiProcessMsg();
				_wiState = WSWaitSOF;
			}else{
				DPRINTLN_WIFI("Checksum Error!");
				_wiState = WSWaitSOF;
			}
			
		}
	}
}

void wiInitialize(){
	_currentState = StageIdleScreen;
	//wifi
	_pumpStatus=_heaterStatus=0;
	//wifi
	gIsWifiConnected=false;
	_wiState = WSWaitSOF;
	wiL2Init();
	
	WifiPortBegin(WiFiSerialBaudRate);
}

void wiThread()
{
	wiReceive();
	if(gIsWifiConnected) wiStatusReport();
	if(IsRetransTimerExpiry)
	{
		StopRetransTimer;
		if(_retryCount < MAX_RETRY){
			//retransmit
			wiSendFromQueueBuffer();
			_retryCount++;
			DPRINT_WIFI("Re-send:");
			DPRINTLN_WIFI(_retryCount);
		}else{
			wiSerialDisconnected();
		}
	}
}


#endif
