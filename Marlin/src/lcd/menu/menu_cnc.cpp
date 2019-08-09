/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


/****************************************************************************
 *   Written By Florin Muntean (C)2019                                         *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <http://www.gnu.org/licenses/>.                              *
 ****************************************************************************/


/*
  G54 ; Select workspace 0 (this is the machine physical workspace)
  G55 ; Select workspace 1 (this will be the tool change workspace) 
  G56 ; Select workspace 2 (this will be the workpiece milling workspace)



  Main process:
    1. if not using home switches and homing select CNC -> Reset all Axes
    2. Move to tool change position aka where the z probing is done using MOVE Axis menu
    3. Set or change the Z probing offset if necessary CNC -> XYZ Probing -> Z probe offset
    3. CNC -> XYZ Probing -> Z probe down 
    4. Move to work piece origin using MOVE Axis menu
    5. CNC -> Set Workpiece Origin 
    6. Start the printing

    For Change Tool script use:
    M25  ; pause 
    M801 ; use M801 macro to move to the position
    #    ; wait here for pause to take effect
    ; change tool
    ; using CNC -> XYZ Probing -> Z probe down will reset the tool Z 
    ; use CNC -> Move to Workpiece origin
    ; resume will continue priting with the new tool now


*/



#include "../../inc/MarlinConfigPre.h"

#if HAS_LCD_MENU && ENABLED(CNC)

#include "../../core/macros.h"
#include "menu.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../gcode/queue.h"
#include "../../gcode/gcode.h"
#include "../../gcode/parser.h"
#include "../../sd/cardreader.h"
#include "../../module/printcounter.h"
#include "../../module/endstops.h"

#if ENABLED(DELTA)
  #include "../../module/delta.h"
#endif


#if HAS_LEVELING
  #include "../../module/planner.h"
  #include "../../feature/bedlevel/bedlevel.h"
#endif

//extern millis_t manual_move_start_time;
//extern int8_t manual_move_axis;
//#if IS_KINEMATIC
//  extern float manual_move_offset;
//#endif


void reset_all_axes(){
  //Setting machine physical origin helps especially if not using home switches
  //G92 applies offsets only  so we can't use it here

  //enable all steppers to make sure they keep their position
  enable_all_steppers();

  //set current coordinates directly
  current_position[X_AXIS]=0;
  current_position[Y_AXIS]=0;
  current_position[Z_AXIS]=0;


  //remove the ? from the LCD
  set_axis_is_at_home(X_AXIS);
  set_axis_is_at_home(Y_AXIS);
  set_axis_is_at_home(Z_AXIS);

  sync_plan_position(); //sync the planner

  //audio feedback
  ui.completion_feedback();
  ui.return_to_status();
}

void reset_axis(AxisEnum axis){
  current_position[axis]=0;
  set_axis_is_at_home(axis);
  sync_plan_position();
  ui.completion_feedback();
  ui.return_to_status();
}


void inline reset_x_axis(){ enable_X(); reset_axis(X_AXIS);}
void inline reset_y_axis(){ enable_Y(); reset_axis(Y_AXIS);}
void inline reset_z_axis(){ enable_Z(); reset_axis(Z_AXIS);}




//all these variables are used to keep the milling information
static float millingX = DEFAULT_CNC_MILLING_X;
static float millingY = DEFAULT_CNC_MILLING_Y;
static float millingZ = DEFAULT_CNC_MILLING_Z;
static float toolDiameter  = DEFAULT_CNC_TOOL_DIAMETER;
static float millSpeed     = DEFAULT_CNC_MILLING_SPEED;
static uint8_t millOverlap   = DEFAULT_CNC_MILLING_OVERLAP;
static float zProbeOffset = 0;


static float scan[4]; //used to keep the min max for x and Y during scanning
#define MINX scan[0]
#define MINY scan[1]
#define MAXX scan[2]
#define MAXY scan[3]


void cnc_process_command_P(PGM_P const cmd){
  SERIAL_ECHOLN(cmd);
  gcode.process_subcommands_now_P(cmd);
  
}


void cnc_process_command(char* cmd){
  SERIAL_ECHOLN(cmd);
  gcode.process_subcommands_now(cmd);
}



/*
G55 ;Select workspace 1 (this will be the tool workspace) 
G91 ;relative moves

G38.2 Z-100 ;find Z surface
G92 Z0 ;set this position as zero (it is 10mm above the surface)
G0 Z10 ;move Z up
G91
G0 X35 + <TOOL_DIAMETER> ; move to the right making sure we clear the block
G0 Z-15 ;move the tool down to half block size
G38.2 X-100 ;find X
G92 X <TOOL_DIAMETER/2> ;mark the zero point acomodating for the tool 
G0 X10 ;move to the side 
G0 Y 60 + <TOOL_DIAMETER>  ;move to reach the other side making sure we clear the block
G0 X-27 ;move to the middle of the block (-10-35/2)
G38.2 Y-100; find Y
G92 Y <TOOL_DIAMETER/2> ; mark the zero point acomodating for the tool
G0 Y10 ;get the tool out of the way
G90 ;switch to absolute position
G0 X0 Y0 Z0 F200 ;lets move slowly to the zero position
;from here in this workspace we should know where everything is located if we know the precise position of the block. 
;if the block is randomly put then we can still use this position for tool changing

G56 ; Select workspace 2 (this will be the milling workspace)
G92 X0 Y0 Z0 ;after we manually moved to the desired starting position for milling
;at this point we can start milling
*/

#ifdef USE_PROBE_BLOCK
void cnc_tool_workspace(){
  char cmd[50],str_1[16];//,str_2[16];

  //G55 ;Select workspace 1 (this will be the tool workspace) 
  //G91 ;relative moves
  cnc_process_command_P(PSTR("G55\nG91"));   //relative moves

  //G38.2 Z-100 ;find Z surface
  dtostrf(HOMING_FEEDRATE_Z, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G38.2 Z-100 F%s"),str_1);  
  cnc_process_command(cmd);

  //G92 Z0 ;set this position as zero (it is 10mm [block Z size] above the surface)
  //G0 Z10 ;move Z up
  cnc_process_command_P(PSTR("G92 Z0\nG0 Z10"));

  //G0 X35 + <TOOL_DIAMETER> ; move to the right making sure we clear the block
  dtostrf(toolDiameter+PROBE_BLOCK_X, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 X%s"),str_1);  
  cnc_process_command(cmd);

  //G0 Z-15 ;move the tool down to half block size
  dtostrf(PROBE_BLOCK_Z/2+10, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 Z-%s"),str_1);  
  cnc_process_command(cmd);

  //G38.2 X-100 ;find X
  dtostrf(HOMING_FEEDRATE_XY, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G38.2 X-100 F%s"),str_1);  
  cnc_process_command(cmd);
  //cnc_process_command_P(PSTR("G38.2 X-100"));

  //G92 X <TOOL_DIAMETER/2> ;mark the zero point acomodating for the tool 
  dtostrf(toolDiameter/2, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G92 X%s"),str_1);  
  cnc_process_command(cmd);

  //G0 X10 ;move to the side 
  cnc_process_command_P(PSTR("G0 X10"));

  //G0 Y 60 + <TOOL_DIAMETER>  ;move to reach the other side making sure we clear the block
  dtostrf(PROBE_BLOCK_Y+toolDiameter, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 Y%s"),str_1);  
  cnc_process_command(cmd);


  //G0 X-27 ;move to the middle of the block (-10-35/2)
  dtostrf(10+PROBE_BLOCK_X/2, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 X-%s"),str_1);  
  cnc_process_command(cmd);

  //G38.2 Y-100; find Y
  dtostrf(HOMING_FEEDRATE_XY, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G38.2 Y-100 F%s"),str_1);  
  cnc_process_command(cmd);
  //cnc_process_command_P(PSTR("G38.2 Y-100"));

  //G92 Y <TOOL_DIAMETER/2> ; mark the zero point acomodating for the tool
  dtostrf(toolDiameter/2, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G92 Y%s"),str_1);  
  cnc_process_command(cmd);


  //G0 Y10 ;get the tool out of the way
  //G90 ;switch to absolute position
  cnc_process_command_P(PSTR("G0 Y10 Z10\nG90\nG0 X0 Y0 Z0"));

  //G0 X0 Y0 Z0 F200 ;lets move slowly to the zero position
  //sprintf_P(cmd, PSTR("G0 X0 Y0 Z0"),str_1);  
  //cnc_process_command(cmd);
}
#else
  void cnc_tool_workspace(){}; 
#endif






//the gcode generation program for milling the top
void cnc_start_milling_top_xDirection(){
  char cmd[50],str_1[16],str_2[16];

  ui.return_to_status();
  ui.set_status_P(PSTR(MSG_CNC_MILLING_TOP_SURFACE));
  

  float w = millingX -toolDiameter/2;
  float h = millingY -toolDiameter/2; 
  
  float overlap = float(ui8_to_percent(millOverlap)) / 100.0f;
  float dY=((overlap*toolDiameter));

  cnc_process_command_P(PSTR("G91")); //relative moves

  /*
  dtostrf(-w/2, 1, 3, str_1);
  dtostrf(-h/2, 1, 3, str_2);
  sprintf_P(cmd, PSTR("G0 X%s Y%s"),str_1,str_2); //move to begining
  cnc_process_command(cmd);
 */

  long ms=millis();

  for(int z=0;z<millingZ;z++){ //this is for z
    
    
    dtostrf(millSpeed, 1, 3, str_2);
    sprintf_P(cmd, PSTR("G1 Z-1 F%s"),str_2);
    cnc_process_command(cmd);

    float y=0;
    dtostrf(w, 1, 3, str_1);
    
    while(y<h ){

      dtostrf(millSpeed, 1, 3, str_2); //trying to see if it acomodates for the infoscreen feed adjustment
      sprintf_P(cmd, PSTR("G1 X%s F%s"),str_1,str_2);
      cnc_process_command(cmd);

      dtostrf(h>y+dY?dY:h-y, 1, 3, str_2);
      sprintf_P(cmd, PSTR("G1 Y%s"),str_2);
      cnc_process_command(cmd);

      y+=dY;

        if (millis()-ms>1000){ //every 1 second refresh the screen info
            ui.refresh(LCDVIEW_CALL_NO_REDRAW);
            ms= millis();
        }

      sprintf_P(cmd, PSTR("G1 X-%s"),str_1);
      cnc_process_command(cmd);

      if (y<h){
        dtostrf(h>y+dY?dY:h-y, 1, 3, str_2);
        sprintf_P(cmd, PSTR("G1 Y%s"),str_2);
        cnc_process_command(cmd);
        y+=dY;
      }

    }//while

    if (z%2==0){

      //finish the margins 
      sprintf_P(cmd, PSTR("G1 X%s"),str_1);
      cnc_process_command(cmd);

      dtostrf(h, 1, 3, str_2);
      sprintf_P(cmd, PSTR("G1 Y-%s"),str_2);
      cnc_process_command(cmd);

      //dtostrf(h, 1, 3, str_2);
      sprintf_P(cmd, PSTR("G1 X-%s"),str_1);
      cnc_process_command(cmd);
    }
    else
    {
      dtostrf(h, 1, 3, str_2);
      sprintf_P(cmd, PSTR("G1 Y-%s"),str_2);
      cnc_process_command(cmd);
    }
    


  } //for
      

  cnc_process_command_P(PSTR("G90")); //absolute moves
}

void cnc_start_milling_top_diagonaly(){
    char cmd[50],str_1[16],str_2[16];

  ui.return_to_status();
  ui.set_status_P(PSTR(MSG_CNC_MILLING_TOP_SURFACE));
  

  float w = millingX/2;
  float h = millingY/2;
  
  float oX=LOGICAL_X_POSITION(current_position[X_AXIS]);
  float oY=LOGICAL_Y_POSITION(current_position[Y_AXIS]);
  float oZ=LOGICAL_Z_POSITION(current_position[Z_AXIS])+5; //we want to end up 5 mm above the surface

  cnc_process_command_P(PSTR("G90\nG0 Z5")); //move up 5mm
  
  


  dtostrf(-w, 1, 3, str_1);
  dtostrf(-h, 1, 3, str_2);
  sprintf_P(cmd, PSTR("G0 X%s Y%s"),str_1,str_2); //move to begining
  cnc_process_command(cmd);

  

  float overlap = float(ui8_to_percent(millOverlap)) / 100.0f;
  float dX=(overlap*toolDiameter),dY=(overlap*toolDiameter);
  SERIAL_ECHO("DX="); SERIAL_ECHOLN(dX);
  SERIAL_ECHO("DY="); SERIAL_ECHOLN(dY);
  
  long ms=millis();

  for(int z=0;z<millingZ;z++){ //this is for z
    
    dtostrf(-z, 1, 3, str_1);
    dtostrf(millSpeed, 1, 3, str_2);
    sprintf_P(cmd, PSTR("G1 Z%s F%s"),str_1,str_2);
    cnc_process_command(cmd);

    float x=-w,y=-h;
    float minx=-w,miny=-h;

   while( (minx<w) & (miny<h) ){
      x=x+dX; y=y+dY;
      
      if (millis()-ms>1000){ //every 1 second refresh the screen info
          ui.refresh(LCDVIEW_CALL_NO_REDRAW);
          ms= millis();
      }

      if (y>h) {minx+=dX;y=h;}
      if (x>w) {x=w;miny+=dY;}
      
      dtostrf(x==w?w:x-dX, 1, 3, str_1);
      dtostrf(x==w?miny-dY:miny, 1, 3, str_2);
      sprintf_P(cmd, PSTR("G1 X%s Y%s"),str_1,str_2);
      cnc_process_command(cmd);

      if ( (minx>w) & (miny>h) )
      {
        dtostrf(w, 1, 3, str_1);
        dtostrf(h, 1, 3, str_2);
        sprintf_P(cmd, PSTR("G1 X%s Y%s"),str_1,str_2);
        cnc_process_command(cmd);
        continue;
      }

      dtostrf(x, 1, 3, str_1);
      dtostrf(miny, 1, 3, str_2);
      sprintf_P(cmd, PSTR("G1 X%s Y%s"),str_1,str_2);
      cnc_process_command(cmd);

      dtostrf(y==h?minx-dX:minx, 1, 3, str_1);
      dtostrf(y==h?h:y-dY, 1, 3, str_2);
      sprintf_P(cmd, PSTR("G1 X%s Y%s"),str_1,str_2);
      cnc_process_command(cmd);


      dtostrf(minx, 1, 3, str_1);
      dtostrf(y, 1, 3, str_2);
      sprintf_P(cmd, PSTR("G1 X%s Y%s"),str_1,str_2);
      cnc_process_command(cmd);

      
    }
    
    cnc_process_command_P(PSTR("G90"));

    dtostrf(oZ, 1, 3, str_1);
    sprintf_P(cmd, PSTR("G0 Z%s"),str_1);
    cnc_process_command(cmd);


    dtostrf(oX, 1, 3, str_1);
    dtostrf(oY, 1, 3, str_2);
    sprintf_P(cmd, PSTR("G0 X%s Y%s"),str_1,str_2);
    cnc_process_command(cmd);
  }
    
    ui.completion_feedback();
    ui.reset_status();
 
}

void cnc_start_milling_x_side(){
  char cmd[20],str_1[16],str_2[16];

  ui.return_to_status();
  ui.set_status_P(PSTR(MSG_CNC_MILLING_X_SIDE));

  
  cnc_process_command_P(PSTR("G91"));
 
  dtostrf(millSpeed, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G1 Z0 F%s"),str_1);
  cnc_process_command(cmd); //establish the speed with an extra command in order not to use another string variable

  dtostrf(millingX, 1, 3, str_1);
  for(int z=0;z<millingZ;z++){ //this is for z
      
    //dtostrf(-z, 1, 3, str_2);
    sprintf_P(cmd, PSTR("G1 X%s Z-1"),str_1,str_2);
    cnc_process_command(cmd);

       
    sprintf_P(cmd, PSTR("G1 X-%s"),str_1,str_2);
    cnc_process_command(cmd);
  }

  cnc_process_command_P(PSTR("G90"));

  ui.completion_feedback();
  ui.reset_status();

}


void cnc_start_milling_y_side(){
char cmd[20],str_1[16],str_2[16];

  ui.return_to_status();
  ui.set_status_P(PSTR(MSG_CNC_MILLING_Y_SIDE));

  
  cnc_process_command_P(PSTR("G91"));//relative movements
 
  dtostrf(millSpeed, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G1 Z0 F%s"),str_1);
  cnc_process_command(cmd); //establish the speed with an extra command in order not to use another string variable


  dtostrf(millingY, 1, 3, str_1);
  for(int z=0;z<millingZ;z++){ //this is for z
      
      //dtostrf(-z, 1, 3, str_2);
      sprintf_P(cmd, PSTR("G1 Y%s Z-1"),str_1,str_2);
      cnc_process_command(cmd);

       
    sprintf_P(cmd, PSTR("G1 Y-%s Z"),str_1,str_2);
    cnc_process_command(cmd);
  }

  cnc_process_command_P(PSTR("G90")); //revert to absolute

  ui.completion_feedback();
  ui.reset_status();


}


void cnc_start_drilling(){
 char cmd[20],str_1[16];

  ui.return_to_status();
  ui.set_status_P(PSTR(MSG_CNC_MILLING_Y_SIDE));

  //the drilling algorithm
  cnc_process_command_P(PSTR("G91\nG0 Z2"));//relative 
  
  dtostrf(millSpeed, 1, 3, str_1);

  for(int z=0;z<millingZ;z++){ //this is for z
    sprintf_P(cmd, PSTR("G1 Z-3 F%s"),str_1);
    cnc_process_command(cmd);
    safe_delay(100);
    cnc_process_command_P(PSTR("G0 Z2"));
  }

  dtostrf(millingZ, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 Z%s"),str_1);
  cnc_process_command(cmd); //retract the tool back 

  cnc_process_command_P(PSTR("G90")); //back to absolute positioning 
  
  ui.completion_feedback();
  ui.reset_status();
}


void set_workpiece_origin(){
  cnc_process_command_P(PSTR("G56\nG92 X0 Y0\nM300")); //we keep the same Z as before
  ui.return_to_status();
}

void probing_z_down(){
  char cmd[50],str_1[10];
  dtostrf(zProbeOffset, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G56\nG91\nG38.2 Z-10\nG90\nG92 Z%s"),str_1);
  cnc_process_command(cmd); 
  if (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING) //probe endstop hit
  {
    cnc_process_command_P(PSTR("G1 Z10"));
    cnc_process_command_P(PSTR("G55\nG90\nG92 X0 Y0 Z0"));
    cnc_process_command_P(PSTR("M810 G55|G0 Z0|G0 X0 Y0\nG56"));
    
    ui.return_to_status();
  }
}

#define busy (IS_SD_PRINTING() || print_job_timer.isRunning() || print_job_timer.isPaused())

//CNC -> PROBING submenu
void menu_cnc_probing() {
  START_MENU();
  MENU_BACK(MSG_CNC);

  //the most used command here is to set the Z axis when changing tools so we put it first
  MENU_ITEM(function, MSG_CNC_PROBE_Z, probing_z_down);

  

  if (!busy){
    MENU_ITEM_EDIT(float3, MSG_ZPROBE_ZOFFSET, &zProbeOffset , -50,50);
    MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  }
  
  //MENU_ITEM(gcode, MSG_CNC_PROBE_X_RIGHT, PSTR("G91\nG38.2 X20\nG90\nG92 Z0"));
  //MENU_ITEM(gcode, MSG_CNC_PROBE_X_LEFT, PSTR("G91\nG38.2 X-20\nG90\nG92 Z0"));
  
  //MENU_ITEM(gcode, MSG_CNC_PROBE_Y_RIGHT, PSTR("G91\nG38.2 Y20\nG90\nG92 Z0"));
  //MENU_ITEM(gcode, MSG_CNC_PROBE_Y_LEFT,  PSTR("G91\nG38.2 Y-20\nG90\nG92 Z0"));

  END_MENU();
}



void inline lcd_menu_cnc_milling_top(){
  do_select_screen_yn(
      cnc_start_milling_top_xDirection, ui.goto_previous_screen,
      PSTR(MSG_CNC_MILLING_TOP_SURFACE), MSG_CNC_START " " MSG_CNC_MILLING, PSTR("?")
    );
} 

void inline lcd_menu_cnc_milling_X_side(){
  do_select_screen_yn(
      cnc_start_milling_x_side, ui.goto_previous_screen,
      PSTR(MSG_CNC_MILLING_X_SIDE), MSG_CNC_START " " MSG_CNC_MILLING, PSTR("?")
    );
} 

void inline lcd_menu_cnc_milling_Y_side(){
  do_select_screen_yn(
      cnc_start_milling_y_side, ui.goto_previous_screen,
      PSTR(MSG_CNC_MILLING_Y_SIDE), MSG_CNC_START " " MSG_CNC_MILLING, PSTR("?")
    );
} 

void inline lcd_menu_cnc_drill(){
  do_select_screen_yn(
      cnc_start_drilling, ui.goto_previous_screen,
      PSTR(MSG_CNC_DRILL),MSG_CNC_START " " MSG_CNC_DRILLING,  PSTR("?")
    );
   
}

void menu_cnc_config(){
  START_MENU();
  MENU_BACK(MSG_CNC_MILLING);

  MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_CNC_MILLING_OVERLAP, &millOverlap, 1, 255);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 25, CNC_MAX_MILLING_SPEED);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_X, &millingX, 0, X_BED_SIZE);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_Y, &millingY, 0, Y_BED_SIZE);

  END_MENU();
}

void menu_cnc_milling_top(){
  START_MENU();
  MENU_BACK(MSG_CNC_MILLING);

  MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 10, CNC_MAX_MILLING_SPEED);
  MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_CNC_MILLING_OVERLAP, &millOverlap, 1, 255);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_X, &millingX, 0, X_BED_SIZE);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_Y, &millingY, 0, Y_BED_SIZE);

  MENU_ITEM(submenu,MSG_CNC_MILL,lcd_menu_cnc_milling_top);

  END_MENU();
}

void menu_cnc_milling_X_side(){
  START_MENU();
  MENU_BACK(MSG_CNC_MILLING);

  //MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 10, CNC_MAX_MILLING_SPEED);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_X,  &millingX, 0, X_BED_SIZE);
  //MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_Y, &millingY, 0, Y_BED_SIZE);

  MENU_ITEM(submenu,MSG_CNC_MILL,lcd_menu_cnc_milling_X_side);


  END_MENU();
}

void menu_cnc_milling_Y_side(){
  START_MENU();
  MENU_BACK(MSG_CNC_MILLING);

  //MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 10, CNC_MAX_MILLING_SPEED);
  
  //MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_X, &millingX, 0, X_BED_SIZE);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_Y, &millingY, 0, Y_BED_SIZE);

  MENU_ITEM(submenu,MSG_CNC_MILL,lcd_menu_cnc_milling_Y_side);

  END_MENU();
}

void menu_cnc_drill(){
  START_MENU();
  MENU_BACK(MSG_CNC_MILLING);

  MENU_ITEM_EDIT(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  MENU_ITEM_EDIT(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 1,CNC_MAX_DRILLING_SPEED);
  
  MENU_ITEM(submenu,MSG_CNC_DRILL,lcd_menu_cnc_drill);

  END_MENU();
}



void menu_cnc_milling(){
  START_MENU();
  MENU_BACK(MSG_CNC);

  //settings
  MENU_ITEM(submenu, MSG_CONFIGURATION,menu_cnc_config);

  //flaten top surface
  MENU_ITEM(submenu,MSG_CNC_MILLING_TOP_SURFACE,menu_cnc_milling_top);

  //mill corner X axis
  MENU_ITEM(submenu,MSG_CNC_MILLING_X_SIDE,menu_cnc_milling_X_side);

  //mill corner Y axis
  MENU_ITEM(submenu,MSG_CNC_MILLING_Y_SIDE,menu_cnc_milling_Y_side);

  MENU_ITEM(submenu,MSG_CNC_DRILL,menu_cnc_drill);

  END_MENU();
}


//these are defined inside the menu_sdcard.cpp
extern void lcd_sd_updir();
extern uint16_t sd_encoder_position; 
extern int8_t sd_top_line, sd_items;


  /**
   * Get commands from the SD Card until the command buffer is full
   * or until the end of the file is reached. The special character '#'
   * can also interrupt buffering.
   */
  void scan_sdcard_commands(){ //float *minX, float *minY, float *maxX, float *maxY) {
  
    bool stop_buffering = false,
         sd_comment_mode = false
          #if ENABLED(PAREN_COMMENTS)
            , sd_comment_paren_mode = false
          #endif
        ;
    char cmd[MAX_CMD_SIZE];
    /**
     * '#' stops reading from SD to the buffer prematurely, so procedural
     * macro calls are possible. If it occurs, stop_buffering is triggered
     * and the buffer is run dry; this character _can_ occur in serial com
     * due to checksums, however, no checksums are used in SD printing.
     */

    //if (commands_in_queue == 0) stop_buffering = false;

    SERIAL_ECHOLN("Start scanning");
    uint16_t sd_count = 0;
    bool card_eof = card.eof();
    while (!card_eof && !stop_buffering) {
      const int16_t n = card.get();
      char sd_char = (char)n;
      card_eof = card.eof();
      if (card_eof || n == -1
          || sd_char == '\n' || sd_char == '\r'
          || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode
            #if ENABLED(PAREN_COMMENTS)
              && !sd_comment_paren_mode
            #endif
          )
      ) {
        if (card_eof) {
         //we reached the end of the file

          //card.printingHasFinished();
          SERIAL_ECHOLN("Done Scanning");
          
          //return;
        }
        else if (n == -1)
          SERIAL_ERROR_MSG(MSG_SD_ERR_READ);

        if (sd_char == '#') stop_buffering = true;

        sd_comment_mode = false; // for new command
        #if ENABLED(PAREN_COMMENTS)
          sd_comment_paren_mode = false;
        #endif

        // Skip empty lines and comments
        if (!sd_count)  continue;

        cmd[sd_count] = '\0'; // terminate string
        sd_count = 0; // clear sd line buffer

        //_commit_command(false);
        //SERIAL_ECHOLN(cmd);
        
       safe_delay(1);
        parser.parse(cmd);

        

        if (parser.seen_axis()){
          if (parser.seen('X')){
            float x = parser.value_per_axis_units(X_AXIS);
            MINX = _MIN(MINX, x);
            MAXX = _MAX(MAXX, x);
            //SERIAL_ECHOPAIR_F("X=",x,2);
          }

          if (parser.seen('Y')){
          float y = parser.value_per_axis_units(Y_AXIS);
          MINY = _MIN(MINY, y);
          MAXY = _MAX(MAXY, y);
          //SERIAL_ECHOLNPAIR_F(" Y=",y,2);
          }

        // SERIAL_ECHOLNPAIR_F("range X: ", *maxX-*minX,2);
        }

      }
      else if (sd_count >= MAX_CMD_SIZE - 1) {
        /**
         * Keep fetching, but ignore normal characters beyond the max length
         * The command will be injected when EOL is reached
         */
      }
      else {
        if (sd_char == ';') sd_comment_mode = true;
        #if ENABLED(PAREN_COMMENTS)
          else if (sd_char == '(') sd_comment_paren_mode = true;
          else if (sd_char == ')') sd_comment_paren_mode = false;
        #endif
        else if (!sd_comment_mode
          #if ENABLED(PAREN_COMMENTS)
            && ! sd_comment_paren_mode
          #endif
        ) cmd[sd_count++] = sd_char;
      }
    }
  }


void inline line_to_z(const float &z) {
    current_position[Z_AXIS] = z;
    planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[Z_AXIS]), active_extruder);
  }


void cnc_scan_move(){
    char cmd[50], str_1[16], str_2[16];

    //ui.clear_lcd();
    START_SCREEN();
      STATIC_ITEM("WorkPiece");
      dtostrf(MINX, 1, 3, str_1);
      dtostrf(MAXX, 1, 3, str_2);
      sprintf_P(cmd, PSTR("[%s - %s]"),str_1,str_2);  
      STATIC_ITEM("X=",true,false, cmd);
      
      dtostrf(MINY, 1, 3, str_1);
      dtostrf(MAXY, 1, 3, str_2);
      sprintf_P(cmd, PSTR("[%s - %s]"),str_1,str_2);  
      STATIC_ITEM("Y=", true,false, cmd);
    END_SCREEN();
    

    uint8_t i=0;
    do{

    //
    // Encoder knob or keypad buttons adjust the Z position
    //
    if (ui.encoderPosition) {
      const float z = current_position[Z_AXIS] + float(int16_t(ui.encoderPosition)) * 0.5f;
      line_to_z(z);
      ui.encoderPosition = 0;
    }

    
    const bool got_click = ui.use_click();
    if (got_click){ 
        ui.return_to_status();
        cnc_process_command_P("G0 X0 Y0");
        planner.synchronize();
        return;
    }

    if (planner.has_blocks_queued())
    {
      idle();
      safe_delay(100);
      continue;
    }

    //scan array order: minx miny maxx maxy
    if (i<2){
      dtostrf(scan[i]-toolDiameter/2, 1, 3, str_1);
    }else
       dtostrf(scan[i]-toolDiameter/2, 1, 3, str_1);
   
    int j = (i+1) % 4;
    if (j<2){
      dtostrf(scan[j]-toolDiameter/2, 1, 3, str_2);
    }
    else
      dtostrf(scan[j]+toolDiameter/2, 1, 3, str_2);
    sprintf_P(cmd, PSTR("G0 X%s Y%s"),str_1,str_2);  
    cnc_process_command(cmd);

    i++;
    i = i % 4;

    }while(1);
}




//the implementation of scanning the file
void cnc_scan_file(){
//  char[50] cmd;
  MINX = X_BED_SIZE; //minX
  MINY = Y_BED_SIZE; //minY
  MAXX = -X_BED_SIZE; //maxX
  MAXY = -Y_BED_SIZE; //maxY

  ui.clear_lcd();
  ui.set_status_P(MSG_CNC_SCANNING);
  ui.draw_status_message(true);

  char cmd[4 + strlen(card.filename) + 1]; // Room for "M23 ", filename, and null
  sprintf_P(cmd, PSTR("M23 %s"), card.filename);
  for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
  cnc_process_command(cmd);

  //read line by line and command by command
  scan_sdcard_commands();//&scan[0],&scan[1],&scan[2],&scan[3]);
  
  sprintf_P(cmd, PSTR("[%s,%s] [%s,%s]"),ftostr51sign(MINX),ftostr51sign(MINY),ftostr51sign(MAXX),ftostr51sign(MAXY));
  //display the box area on the screen
  SERIAL_ECHOPGM("Found Range: ");SERIAL_ECHOLN(cmd); 

  do_select_screen(PSTR(MSG_CNC_MOVE), PSTR(MSG_BUTTON_CANCEL),
      cnc_scan_move, ui.return_to_status,
      PSTR(MSG_CNC_SCAN_BOX),MSG_CNC_START 
    );
}

void inline lcd_menu_cnc_scan(){
  do_select_screen_yn(
      cnc_scan_file, ui.goto_previous_screen,
      PSTR(MSG_CNC_SCAN),MSG_CNC_START ,  PSTR("?")
    );
}

class MenuItem_sdfile {
  public:
    static void action(CardReader &theCard) {
      #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
        // Save menu state for the selected file
        sd_encoder_position = ui.encoderPosition;
        sd_top_line = encoderTopLine;
        sd_items = screen_items;
      #endif
      #if ENABLED(SD_MENU_CONFIRM_START)
        MenuItem_submenu::action(lcd_menu_cnc_scan);
      #else
        cnc_scan_file();
      #endif
    }
};

class MenuItem_sdfolder {
  public:
    static void action(CardReader &theCard) {
      card.chdir(theCard.filename);
      encoderTopLine = 0;
      ui.encoderPosition = 2 * (ENCODER_STEPS_PER_MENU_ITEM);
      screen_changed = true;
      #if HAS_GRAPHICAL_LCD
        ui.drawing_screen = false;
      #endif
      ui.refresh();
    }
};


void menu_cnc_scan() {
  ui.encoder_direction_menus();

  const uint16_t fileCnt = card.get_num_Files();

  START_MENU();
  MENU_BACK(MSG_CNC);
  card.getWorkDirName();
  if (card.filename[0] == '/') {
    #if !PIN_EXISTS(SD_DETECT)
      MENU_ITEM(function, LCD_STR_REFRESH MSG_REFRESH, lcd_sd_refresh);
    #endif
  }
  else if (card.isDetected())
    MENU_ITEM(function, LCD_STR_FOLDER "..", lcd_sd_updir);

  if (ui.should_draw()) for (uint16_t i = 0; i < fileCnt; i++) {
    if (_menuLineNr == _thisItemNr) {
      const uint16_t nr =
        #if ENABLED(SDCARD_RATHERRECENTFIRST) && DISABLED(SDCARD_SORT_ALPHA)
          fileCnt - 1 -
        #endif
      i;

      card.getfilename_sorted(nr);

      if (card.flag.filenameIsDir)
        MENU_ITEM(sdfolder, MSG_CARD_MENU, card);
      else
        MENU_ITEM(sdfile, MSG_CARD_MENU, card);
    }
    else {
      MENU_ITEM_DUMMY();
    }
  }
  END_MENU();
}

void menu_cnc() {
  START_MENU();

  //
  // ^ Main
  //
  MENU_BACK(MSG_MAIN);

  

  //Resetting coordinates
  if (!busy)
    MENU_ITEM(function, MSG_CNC_RESET_ALL, reset_all_axes);
  
  
  
  //
  // Move Axis
  //
  #if ENABLED(DELTA)
    if (all_axes_homed())
  #endif
  

  if (busy)
  {
    //the most used command here is to set the Z axis when changing tools so we put it first
    MENU_ITEM(function,MSG_CNC_PROBE " " MSG_CNC_PROBE_Z, probing_z_down);
  }else
    MENU_ITEM(submenu, MSG_CNC_PROBE,   menu_cnc_probing);

  //establish the cnc tool workspace
  if (!busy)
    MENU_ITEM(function, MSG_CNC_TOOL_WORKSPACE,cnc_tool_workspace);

  //this creates and executes a macro to move to specific location and can be used during the tool change procedure 
  // so M810 is set to move to the Tool Change position
  //MENU_ITEM(gcode, MSG_CNC_MOVE_TOOL_CHANGE, PSTR("M810 G55|G0 Z" STRINGIFY(TOOL_CHANGE_Z) "|G0 X" STRINGIFY(TOOL_CHANGE_X) " Y" STRINGIFY(TOOL_CHANGE_Y) "\nM810" ));
  MENU_ITEM(gcode, MSG_CNC_MOVE_TOOL_CHANGE, PSTR("M810" ));

  if (!busy)
    MENU_ITEM(function, MSG_CNC_SET_WORKPIECE_ORIGIN, set_workpiece_origin);

  MENU_ITEM(gcode, MSG_CNC_MOVE_WORKPIECE, PSTR("G56\nG0 X0 Y0"));


  if (!busy)
  {
    MENU_ITEM(submenu, MSG_CNC_MILLING, menu_cnc_milling);
    MENU_ITEM(submenu, MSG_CNC_SCAN,menu_cnc_scan);
  }
  //settings
  MENU_ITEM(submenu, MSG_CONFIGURATION,menu_cnc_config);

  MENU_ITEM(function, MSG_CNC_RESET_X, reset_x_axis);
  MENU_ITEM(function, MSG_CNC_RESET_Y, reset_y_axis);
  MENU_ITEM(function, MSG_CNC_RESET_Z, reset_z_axis);

/*
  //
  // Auto Home
  //
  MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28 O"));
  #if ENABLED(INDIVIDUAL_AXIS_HOMING_MENU)
    MENU_ITEM(gcode, MSG_AUTO_HOME_X, PSTR("G28 X"));
    MENU_ITEM(gcode, MSG_AUTO_HOME_Y, PSTR("G28 Y"));
    MENU_ITEM(gcode, MSG_AUTO_HOME_Z, PSTR("G28 Z"));
  #endif

  */

 

   
  //
  // Disable Steppers
  //
  MENU_ITEM(gcode, MSG_DISABLE_STEPPERS, PSTR("M84"));
  MENU_ITEM(gcode, MSG_CNC_DISABLE_Z, PSTR("M84 Z"));

  END_MENU();
}

#endif // HAS_LCD_MENU
