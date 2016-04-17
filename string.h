/**********************************************************************
 BrewManiac 
 created by Vito Tai
 Copyright (C) 2015 Vito Tai
 
 This soft ware is provided as-is. Use at your own risks.
 You are free to modify and distribute this software without removing 
 this statement.
 BrewManiac by Vito Tai is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
***********************************************************************/

#ifndef STRING_H
#define STRING_H

#define StringConst(v,c) const char C_##v[]  PROGMEM = c
#define STR(v) C_##v

//typedef const char* str_t;
#define str_t const char*

StringConst(BT_Setup,"Bluetooth");
StringConst(BT_NetworkName,"Name");
StringConst(BT_UsePin,"Use PIN");
StringConst(BT_PIN,"PIN:");

StringConst( x_x_x_Ok,          "---  ---  ---  Ok");
StringConst( x_x_Edit_Ok,       "---  ---  Edit Ok");
StringConst( Up_Down_Next_x,    "UP   DWN   >>  ---");
StringConst( Up_Down_Done_x,    "UP   DWN  Done ---");

StringConst(min,"min");

StringConst( welcome,"BrewManiac 0.01""\xE0");

#if NoDelayStart == false
StringConst(To_be_started_in,"To be started in");
StringConst( Setting_Delay,"Setting Delay");
StringConst( Delay_Start,"Delay Start?");
#endif

StringConst( Resume_Process,"Resume Process?");

StringConst( In_Pause,"In Pause");
StringConst( IODINE_TEST,"IODINE  TEST");
StringConst( AutomaticMode, "AUTOMATIC MODE");
StringConst( Pump_Prime, "Pump Prime");
StringConst( Auto_Label, "AUTO -->");
StringConst( TemperatureReached,"Temp.  Reached");
StringConst(Add_Malt, "Add Malt");
StringConst(Remove_Malt, "Remove Malt");
StringConst(Go_to_next_step,"Go to next step?");

StringConst(START_COOLING,"START  COOLING");
StringConst(WHIRLPOOL,"WHIRLPOOL");
StringConst(Timeing_Whirlpool,"Timing Whirlpool");

StringConst(Brewing_Process,"Brewing Process");
StringConst(Finished,"Finished");

//button labels
#if NoDelayStart == false
StringConst( x_x_Quit_Go,"         Quit  Go");
#endif
StringConst( No_Yes,            "            No Yes");
StringConst( Manual_Auto_Setup, "---  MAN AUTO SETUP");
StringConst( _Pump_Rest_,       " -  Pump  Rest  -  ");
StringConst( Continue_Yes_x,    "Continue: Yes ---");
StringConst( Continue_Yes_No,   "Continue: Yes  No");

StringConst( x_x_x_Pmp,         "--* *--  ---  Pmp");
StringConst( x_x_Time_Pmp,      "--* *--  Tme  Pmp");
StringConst( Up_Down_x_Pmp,     "UP* *DWN  ---  Pmp");
StringConst( Up_Down_x_x,       "UP* *DWN  ---  ---");
StringConst( x_x_Exit_x,        "---  ---  Exit ---");
StringConst( x_x_Ok_x,          "---  ---   Ok  ---");

StringConst( Up_Down_Heat_Pmp,  "UP* *DWN Heat Pmp");
StringConst( Up_Down_Pause_STP, "UP* *DWN Pause STP");

StringConst( Up_Down_PmPus_STP, "UP* *DWN PmPus STP");

StringConst( Up_Down_Pause_x,   "UP* *DWN Pause ---");
StringConst( Up_Down_Skip_Ok,   "UP* *DWN Skip   Ok");
StringConst( Up_Down_x_Ok,      "UP* *DWN  ---   Ok");

StringConst( Up_Down_Quit_Ok,   "Up   Dwn  Quit  Ok");
StringConst( x_Down_Quit_Ok,    "---  Dwn  Quit  Ok");
StringConst( Up_x_Quit_Ok,      "Up   ---  Quit  Ok");

StringConst( Up_Down_Pause_Pmp,  "UP* *DWN Pause Pmp");
StringConst( Up_Down_RUN_Pmp,     "UP* *DWN  RUN  Pmp");


// end of menu

StringConst( Manual_Mode ,"MANUAL MODE");
StringConst( Water_Added ,"Water  Added?");

StringConst( PWM_Is, "PWM=   %");

// Setting strings
StringConst( Setup, "SETUP MENU");
StringConst( PID_PWM, "PID -- PWM");

StringConst( Use, "Use");
StringConst( Electric, "Electric");
StringConst( Gas, "Gas");
StringConst( kP, "Constant kP");
StringConst( kI, "Constant kI");
StringConst( kD, "Constant kD");
StringConst( SampleTime, "SampleTime");
StringConst( WindowSet_ms, "WindowSet ms");
StringConst( Heat_in_Boil, "Heat in Boil");
StringConst( Calibration, "Calibration");
StringConst( Hysteresi, "Hysteresi");


StringConst( Unit_Parameters, "Unit Parameters");
StringConst( Set_Degree, "Set Degrees");
StringConst( Sensor, "Sensor");
StringConst( Inside, "Inside");
StringConst( Outside, "Outside");
StringConst( Temp_Boil, "Temp Boil");
StringConst( Pump_Cycle, "Pump Cycle");
StringConst( Pump_Rest, "Pump Rest");
StringConst( Pump_PreMash, "Pump PreMash");
StringConst( On, "On");
StringConst( Off, "Off");
StringConst( Pump_On_Mash, "Pmp on Mash");
StringConst( Pump_Mashout, "Pmp Mashout");
StringConst( Pump_On_Boil, "Pmp on Boil");
StringConst( Pump_Stop, "Pump Stop");
StringConst( PID_Pipe, "PID Pipe");
StringConst( Active, "Active");
StringConst( Passive, "Passive");

StringConst( Skip_Add, "Skip Add");
StringConst( Yes, "Yes");
StringConst( No, "No");
StringConst( Skip_Remove, "Skip Remove");
StringConst( Skip_Iodine, "Skip Iodine");
StringConst( IodineTime, "IodineTime");
StringConst( Whirlpool_e, "Whirlpool");
StringConst( Hot, "Hot");
StringConst( Cold, "Cold"); // and off

StringConst( Set_Automation, "Set Automation");
StringConst( Mash_In, "Mash In");
StringConst( Phytase, "Phytase");
StringConst( Glucanase, "Glucanase");
StringConst( Protease, "Protease");
StringConst( bAmylase, "\xE2""Amylase");
StringConst( aAmylase1, "\xE0""Amylase1");
StringConst( aAmylase2, "\xE0""Amylase2");
StringConst( Mash_out, "Mash Out");
StringConst( Boil, "Boil");
StringConst( Cooling, "Cooling");
StringConst( Whirlpool, "Whirlpool");
StringConst( Number_Of_Hops, "Number of Hops");
StringConst( Hops_Number_leftPara, "Hop nmbr#");
StringConst( right_Para, ")");

StringConst( Hops_Number_x, "Hop #");

StringConst( Manage_Recipes, "Manage Recipes");


#endif
