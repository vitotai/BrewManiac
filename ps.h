/**********************************************************************
 BrewManiac 
 created by Vito Tai
 Copyright (C) 2015 Vito Tai
 
 This soft ware is provided as-is. Use at your own risks.
 You are free to modify and distribute this software without removing 
 this statement.
 BrewManiac by Vito Tai is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
***********************************************************************/

#ifndef PS_H
#define PS_H
// Persistence Storage
//  PID MENU

#define PS_UseGas   0  //	Use Gas
#define PS_PidBase PS_UseGas

#define PS_AddrOfPidSetting(i) (PS_PidBase+i)

#define PS_kP      1  // 	kP
#define PS_kI      2  //	kI
#define PS_kD      3  //     kD
#define PS_SampleTime      4  //     SampleTime
#define PS_WindowSize      5  //     WindowSize
#define PS_BoilHeat      6    //   Boil Heat %
#define PS_Offset     7      // Offset
#define PS_Hysteresi     8   //    Hysteresi 
//      9       [ SPACE ]

//  UNIT MENU  
#define PS_TempUnit   10     //  Scale Temp
#define PS_UnitBase PS_TempUnit

#define PS_AddrOfUnitSetting(i) (PS_UnitBase+i)

#define PS_SensorType     11      // Sensor Type
#define PS_BoilTemp     12       //Temp Boil °C
//     13       Temp Boil °F
#define PS_PumpCycle     14  //     Time Pump Cycle
#define PS_PumpRest     15   //    Time Pump Rest
#define PS_PumpPreMash     16  //     Pump PreMash
#define PS_PumpOnMash     17   //    Pump on Mash
#define PS_PumpOnMashOut     18  //     Pump on MashOut
#define PS_PumpOnBoil      19     //  Pump on Boil
#define PS_TempPumpRest     20    //   Temp Pump Rest °C
//     21       Temp Pump Rest °F
#define PS_PidPipe     22     //  PID Pipe
#define PS_SkipAddMalt     23  //     Skip Add Malt
#define PS_SkipRemoveMalt     24  //     Skip Remove Malt
#define PS_SkipIodineTest     25    //   Skip Iodine Test
#define PS_IodineTime     26   //    Iodine Time
#define PS_Whirlpool     27     //  Whirlpool
//     28 -  31 [ SPACE ]

//  RUN  (HTemp °C - LTemp °C - HTemp °F - LTemp °F - Time)
#define PS_RunBase 32
#define PS_StageTemperatureAddr(i) ((PS_RunBase)+(i)* 5)
#define PS_StageTimeAddr(i) ((PS_RunBase)+(i)*5+4)
#define ToTempInStorage(t) ((int)((t)*16))
#define TempFromStorage(t)  ((float)(t)/16.0)

// 0:   32 -  36 MashIn
// 1:   37 -  41 Fitasi / phytase
// 2:   42 -  46 Glucanasi /glucanase
// 3:   47 -  51 Proteasi /protease
// 4:   52 -  55 B-Amilasi /B-amylase
// 5:   57 -  61 A-Amilasi 1 
// 6:   62 -  66 A-Amilasi 2
// 7:   67 -  71 Mash Out

#define PS_NumberOfHops    72  //      Numbers of Hops
#define PS_BoilTime     73    //   Boil Time 
#define PS_HopTimeBase 74
#define PS_TimeOfHop(i) ((PS_HopTimeBase)+i)
/*
    74       Time Hop  1
    75       Time Hop  2
    76       Time Hop  3
    77       Time Hop  4
    78       Time Hop  5
    79       Time Hop  6
    80       Time Hop  7
    81       Time Hop  8
    82       Time Hop  9
    83       Time Hop 10
*/
#define  PS_AutomodeStarted    84    //   FLAG Automode Started

//  RESUME
#define PS_StageResume    85 //      HANDLE Stage
#define PS_StageTimeLeft    86 //      HANDLE Time Rest
#define PS_HopAdd    87     //  Hop Add


#define PS_SensorUseBase    90 
#define PS_SensorUseAddressOf(i)    (PS_SensorUseBase+(i)) 

#define PS_SensorAuxBase    95 
#define PS_AuxSensorAddressOf(i)    (PS_SensorAuxBase+(i)) 

#define PS_SensorAddressBase    100 
#define SensorAddressOf(i) ((i)*8 + PS_SensorAddressBase)

#define PS_SensorCalibrationAddressBase    140
#define CalibrationAddressOf(i) ((i) + PS_SensorCalibrationAddressBase)

//    88 -  89 [ SPACE ]
/*
  RECIPE
    90 -  99 Index 1-10
   100 - 151 Recipe Data  1

   152 - 203 Recipe Data  2
   204 - 255 Recipe Data  3
   256 - 307 Recipe Data  4
   308 - 359 Recipe Data  5
   360 - 411 Recipe Data  6
   412 - 463 Recipe Data  7
   464 - 515 Recipe Data  8
   516 - 567 Recipe Data  9
   568 - 619 Recipe Data 10

   620 - 629 Recipe Name  1
   630 - 639 Recipe Name  2
   640 - 649 Recipe Name  3
   650 - 659 Recipe Name  4
   660 - 669 Recipe Name  5
   670 - 679 Recipe Name  6
   680 - 689 Recipe Name  7
   690 - 699 Recipe Name  8
   700 - 709 Recipe Name  9
   710 - 719 Recipe Name 10
*/

byte readSetting(int addr)
{
	return EEPROM.read(addr);
}

void updateSetting(int addr,byte value)
{
	//EEPROM.update(addr,value);
EEPROM.write(addr,value);
}

word readSettingWord(int addr)
{
	return word(EEPROM.read(addr),EEPROM.read(addr+1));
}

word updateSettingWord(int addr,word value)
{
  	//EEPROM.update(addr,highByte(value));
  	//EEPROM.update((addr+1),lowByte(value));
  	EEPROM.write(addr,highByte(value));
  	EEPROM.write((addr+1),lowByte(value));

}

#endif
