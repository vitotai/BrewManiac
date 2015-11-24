/**********************************************************************
 BrewManiac 
 created by Vito Tai
 Copyright (C) 2015 Vito Tai
 
 This soft ware is provided as-is. Use at your own risks.
 You are free to modify and distribute this software without removing 
 this statement.
 BrewManiac by Vito Tai is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
***********************************************************************/

#ifndef SERIALDEBUG_H
#define SERIALDEBUG_H
//*********************************************************************
// Serial recive
//*********************************************************************
#define RemoteBufferSize 8

#define RemoteParseWaitHead 0
#define RemoteParseWaitCommand 2
#define RemoteParseWaitData 3

#define RM_BEGIN 's'
#define RM_TEMP_COMMAND 't'
#define RM_END '*'

//byte _rmBuffer[RemoteBufferSize];
//byte _rmIndex=0;
int rmTemp;

byte _parseState=RemoteParseWaitHead;

void rmReceive(void)
{
    while(Serial.available())
    {
    	byte ch=Serial.read();
    	if(_parseState == RemoteParseWaitHead)
    	{
    		if(ch == RM_BEGIN)
    		{
    			_parseState=RemoteParseWaitCommand;
    		}
    		else
    		{
    			_parseState=RemoteParseWaitHead;
    		}
    	}
    	else if(_parseState == RemoteParseWaitCommand)
    	{
    		if(ch == RM_TEMP_COMMAND)
    		{
	    		_parseState=RemoteParseWaitData;
	    		rmTemp=0;
    		}
    		else
    		{
    			_parseState=RemoteParseWaitHead;
    		}
    	}
    	else if(_parseState == RemoteParseWaitData)
    	{
    		if(ch == RM_END)
    		{
    			//finish command
    			_parseState=RemoteParseWaitHead;
    			// issue command
    			gCurrentTemperature=rmTemp;
    		}
    		else
    		{
    			// collect data
    			rmTemp = rmTemp*10 + (ch - '0');
    		}
    	}
    }
}   
#endif
