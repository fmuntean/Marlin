
/****************************************************************************
 *   Written By Florin Muntean (C)2019                                      *
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
    6. Start the milling

    For Change Tool script use:
    M25  ; pause
    M801 ; use M801 macro to move to the change tool position
    #    ; wait here for pause to take effect
    ; change tool
    ; using CNC -> XYZ Probing -> Z probe down will reset the tool Z
    ; use CNC -> Move to Workpiece origin
    ; resume will continue  with the new tool now
*/



#include "../../inc/MarlinConfigPre.h"

#if HAS_DISPLAY && ENABLED(CNC)

#include "menu.h"
#include "menu_item.h"

#include "../../core/macros.h"
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
  stepper.enable_all_steppers();

  //remove the ? from the LCD
  set_axis_is_at_home(X_AXIS);
  set_axis_is_at_home(Y_AXIS);
  set_axis_is_at_home(Z_AXIS);

  //set current coordinates directly as the Y otherwise will be set to max position
  current_position[X_AXIS]=0;
  current_position[Y_AXIS]=0;
  current_position[Z_AXIS]=0;
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


void inline reset_x_axis(){ ENABLE_AXIS_X(); reset_axis(X_AXIS);}
void inline reset_y_axis(){ ENABLE_AXIS_Y(); reset_axis(Y_AXIS);}
void inline reset_z_axis(){ ENABLE_AXIS_Z(); reset_axis(Z_AXIS);}




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


void cnc_process_command_P(FSTR_P cmd){
  SERIAL_ECHOLN(cmd);
  gcode.process_subcommands_now(cmd);
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
G92 X <TOOL_DIAMETER/2> ;mark the zero point accommodating for the tool
G0 X10 ;move to the side
G0 Y 60 + <TOOL_DIAMETER>  ;move to reach the other side making sure we clear the block
G0 X-27 ;move to the middle of the block (-10-35/2)
G38.2 Y-100; find Y
G92 Y <TOOL_DIAMETER/2> ; mark the zero point accommodating for the tool
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
  cnc_process_command_P(FSTR_P("G55\nG91"));   //relative moves

  constexpr xyz_feedrate_t homing_feedrate_mm_m = HOMING_FEEDRATE_MM_M;
  //G38.2 Z-100 ;find Z surface
  dtostrf(homing_feedrate_mm_m.Z, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G38.2 Z-100 F%s"),str_1);
  cnc_process_command(cmd);

  //G92 Z0 ;set this position as zero (it is 10mm [block Z size] above the surface)
  //G0 Z10 ;move Z up
  cnc_process_command_P(F("G92 Z0\nG0 Z10"));

  //G0 X35 + <TOOL_DIAMETER> ; move to the right making sure we clear the block
  dtostrf(toolDiameter+PROBE_BLOCK_X, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 X%s"),str_1);
  cnc_process_command(cmd);

  //G0 Z-15 ;move the tool down to half block size
  dtostrf(PROBE_BLOCK_Z/2+10, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 Z-%s"),str_1);
  cnc_process_command(cmd);

  //G38.2 X-100 ;find X
  dtostrf(homing_feedrate_mm_m.X, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G38.2 X-100 F%s"),str_1);
  cnc_process_command(cmd);
  //cnc_process_command_P(PSTR("G38.2 X-100"));

  //G92 X <TOOL_DIAMETER/2> ;mark the zero point accommodating for the tool
  dtostrf(toolDiameter/2, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G92 X%s"),str_1);
  cnc_process_command(cmd);

  //G0 X10 ;move to the side
  cnc_process_command_P(F("G0 X10"));

  //G0 Y 60 + <TOOL_DIAMETER>  ;move to reach the other side making sure we clear the block
  dtostrf(PROBE_BLOCK_Y+toolDiameter, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 Y%s"),str_1);
  cnc_process_command(cmd);


  //G0 X-27 ;move to the middle of the block (-10-35/2)
  dtostrf(10+PROBE_BLOCK_X/2, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 X-%s"),str_1);
  cnc_process_command(cmd);

  //G38.2 Y-100; find Y
  dtostrf(homing_feedrate_mm_m.Y, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G38.2 Y-100 F%s"),str_1);
  cnc_process_command(cmd);
  //cnc_process_command_P(PSTR("G38.2 Y-100"));

  //G92 Y <TOOL_DIAMETER/2> ; mark the zero point accommodating for the tool
  dtostrf(toolDiameter/2, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G92 Y%s"),str_1);
  cnc_process_command(cmd);


  //G0 Y10 ;get the tool out of the way
  //G90 ;switch to absolute position
  cnc_process_command_P(F("G0 Y10 Z10\nG90\nG0 X0 Y0 Z0"));

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
  ui.set_status_P(GET_TEXT(MSG_CNC_MILLING_TOP_SURFACE));


  float w = millingX -toolDiameter/2;
  float h = millingY -toolDiameter/2;

  float overlap = float(ui8_to_percent(millOverlap)) / 100.0f;
  float dY=((overlap*toolDiameter));

  cnc_process_command_P(F("G91")); //relative moves

  long ms=millis();

  for(int z=0;z<millingZ;z++){ //this is for z
    dtostrf(millSpeed, 1, 3, str_2);
    sprintf_P(cmd, PSTR("G1 Z-1 F%s"),str_2);
    cnc_process_command(cmd);

    float y=0;
    dtostrf(w, 1, 3, str_1);

    while(y<h ){

      dtostrf(millSpeed, 1, 3, str_2); //trying to see if it accommodates for the info screen feed adjustment
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


  cnc_process_command_P(F("G90")); //absolute moves
}

void cnc_start_milling_top_diagonally(){
    char cmd[50],str_1[16],str_2[16];

  ui.return_to_status();
  ui.set_status_P(GET_TEXT(MSG_CNC_MILLING_TOP_SURFACE));


  float w = millingX/2;
  float h = millingY/2;

  float oX=LOGICAL_X_POSITION(current_position[X_AXIS]);
  float oY=LOGICAL_Y_POSITION(current_position[Y_AXIS]);
  float oZ=LOGICAL_Z_POSITION(current_position[Z_AXIS])+5; //we want to end up 5 mm above the surface

  cnc_process_command_P(FPSTR("G90\nG0 Z5")); //move up 5mm

  dtostrf(-w, 1, 3, str_1);
  dtostrf(-h, 1, 3, str_2);
  sprintf_P(cmd, PSTR("G0 X%s Y%s"),str_1,str_2); //move to beginning
  cnc_process_command(cmd);

  float overlap = float(ui8_to_percent(millOverlap)) / 100.0f;
  float dX=(overlap*toolDiameter),dY=(overlap*toolDiameter);
  SERIAL_ECHOPGM("DX="); SERIAL_ECHOLN(dX);
  SERIAL_ECHOPGM("DY="); SERIAL_ECHOLN(dY);

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

    cnc_process_command_P(FPSTR("G90"));

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
  ui.set_status_P(GET_TEXT(MSG_CNC_MILLING_X_SIDE));


  cnc_process_command_P(FPSTR("G91"));

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

  cnc_process_command_P(FPSTR("G90"));

  ui.completion_feedback();
  ui.reset_status();

}


void cnc_start_milling_y_side(){
char cmd[20],str_1[16],str_2[16];

  ui.return_to_status();
  ui.set_status_P(GET_TEXT(MSG_CNC_MILLING_Y_SIDE));


  cnc_process_command_P(FPSTR("G91"));//relative movements

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

  cnc_process_command_P(FPSTR("G90")); //revert to absolute

  ui.completion_feedback();
  ui.reset_status();


}


void cnc_start_drilling(){
 char cmd[20],str_1[16];

  ui.return_to_status();
  ui.set_status_P(GET_TEXT(MSG_CNC_MILLING_Y_SIDE));

  //the drilling algorithm
  cnc_process_command_P(FPSTR("G91\nG0 Z2"));//relative

  dtostrf(millSpeed, 1, 3, str_1);

  for(int z=0;z<millingZ;z++){ //this is for z
    sprintf_P(cmd, PSTR("G1 Z-3 F%s"),str_1);
    cnc_process_command(cmd);
    safe_delay(100);
    cnc_process_command_P(FPSTR("G0 Z2"));
  }

  dtostrf(millingZ, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G0 Z%s"),str_1);
  cnc_process_command(cmd); //retract the tool back

  cnc_process_command_P(FPSTR("G90")); //back to absolute positioning

  ui.completion_feedback();
  ui.reset_status();
}


void set_workpiece_origin(){
  cnc_process_command_P(FPSTR("G56\nG92 X0 Y0\nM300")); //we keep the same Z as before
  ui.return_to_status();
}

void probing_z_down(){
  char cmd[50],str_1[10];
  dtostrf(zProbeOffset, 1, 3, str_1);
  sprintf_P(cmd, PSTR("G56\nG91\nG38.2 Z-10\nG90\nG92 Z%s"),str_1);
  cnc_process_command(cmd);
  if (READ(Z_MIN_PROBE_PIN) == Z_MIN_PROBE_ENDSTOP_HIT_STATE) //probe end-stop hit
  {
    cnc_process_command_P(FPSTR("G1 Z10\nM400")); //we move 10mm up
    cnc_process_command_P(FPSTR("G55\nG90\nG92 X0 Y0 Z0")); //we set this as the tool workspace origin
    cnc_process_command_P(FPSTR("M810 G55|G0 Z0|G0 X0 Y0\nG56")); //we create the macro to move to the tool change position

    ui.return_to_status();
  }
}

#define busy (IS_SD_PRINTING() || print_job_timer.isRunning() || print_job_timer.isPaused())

//CNC -> PROBING submenu
void menu_cnc_probing() {
  START_MENU();
  BACK_ITEM(MSG_CNC);

  //the most used command here is to set the Z axis when changing tools so we put it first
  ACTION_ITEM(MSG_CNC_PROBE_Z, probing_z_down);


  if (!busy){
    EDIT_ITEM(float3, MSG_ZPROBE_ZOFFSET, &zProbeOffset , -50,50);
    EDIT_ITEM(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  }

  //MENU_ITEM(gcode, MSG_CNC_PROBE_X_RIGHT, PSTR("G91\nG38.2 X20\nG90\nG92 Z0"));
  //MENU_ITEM(gcode, MSG_CNC_PROBE_X_LEFT, PSTR("G91\nG38.2 X-20\nG90\nG92 Z0"));

  //MENU_ITEM(gcode, MSG_CNC_PROBE_Y_RIGHT, PSTR("G91\nG38.2 Y20\nG90\nG92 Z0"));
  //MENU_ITEM(gcode, MSG_CNC_PROBE_Y_LEFT,  PSTR("G91\nG38.2 Y-20\nG90\nG92 Z0"));

  END_MENU();
}

void menu_cnc_config(){
  START_MENU();
  BACK_ITEM(MSG_CNC_MILLING);

  EDIT_ITEM(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  EDIT_ITEM(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  EDIT_ITEM(percent, MSG_CNC_MILLING_OVERLAP, &millOverlap, 1, 255);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 25, CNC_MAX_MILLING_SPEED);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_X, &millingX, 0, X_BED_SIZE);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_Y, &millingY, 0, Y_BED_SIZE);

  END_MENU();
}

void menu_cnc_milling_top(){
  START_MENU();
  BACK_ITEM(MSG_CNC_MILLING);

  EDIT_ITEM(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  EDIT_ITEM(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 10, CNC_MAX_MILLING_SPEED);
  EDIT_ITEM(percent, MSG_CNC_MILLING_OVERLAP, &millOverlap, 1, 255);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_X, &millingX, 0, X_BED_SIZE);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_Y, &millingY, 0, Y_BED_SIZE);

  //MENU_ITEM(submenu,MSG_CNC_MILL,lcd_menu_cnc_milling_top);
  CONFIRM_ITEM(MSG_CNC_MILLING_TOP_SURFACE,
      MSG_BUTTON_PROCEED, MSG_BUTTON_CANCEL,
      cnc_start_milling_top_xDirection, ui.goto_previous_screen,
      GET_TEXT_F(MSG_CNC_MILLING_TOP_SURFACE), GET_TEXT_F(MSG_CNC_MILLING), F("?")
  );

  END_MENU();
}

void menu_cnc_milling_X_side(){
  START_MENU();
  BACK_ITEM(MSG_CNC_MILLING);

  //EDIT_ITEM(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  EDIT_ITEM(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 10, CNC_MAX_MILLING_SPEED);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_X,  &millingX, 0, X_BED_SIZE);
  //EDIT_ITEM(float5_25, MSG_CNC_MILLING_Y, &millingY, 0, Y_BED_SIZE);

  //MENU_ITEM(submenu,MSG_CNC_MILL,lcd_menu_cnc_milling_X_side);
  CONFIRM_ITEM(MSG_CNC_MILLING_X_SIDE,
      MSG_BUTTON_PROCEED, MSG_BUTTON_CANCEL,
      cnc_start_milling_x_side, ui.goto_previous_screen,
      GET_TEXT_F(MSG_CNC_MILLING_X_SIDE), GET_TEXT_F(MSG_CNC_MILLING), F("?")
    );


  END_MENU();
}

void menu_cnc_milling_Y_side(){
  START_MENU();
  BACK_ITEM(MSG_CNC_MILLING);

  //EDIT_ITEM(float3, MSG_CNC_MILLING_TOOL, &toolDiameter, 0,CNC_MAX_TOOL_DIAMETER);
  EDIT_ITEM(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 10, CNC_MAX_MILLING_SPEED);

  //EDIT_ITEM(float5_25, MSG_CNC_MILLING_X, &millingX, 0, X_BED_SIZE);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_Y, &millingY, 0, Y_BED_SIZE);

  //MENU_ITEM(submenu,MSG_CNC_MILL,lcd_menu_cnc_milling_Y_side);
  CONFIRM_ITEM(MSG_CNC_MILLING_Y_SIDE,
      MSG_BUTTON_PROCEED, MSG_BUTTON_CANCEL,
      cnc_start_milling_y_side, ui.goto_previous_screen,
      GET_TEXT_F(MSG_CNC_MILLING_Y_SIDE), GET_TEXT_F(MSG_CNC_MILLING), F("?")
    );

  END_MENU();
}

void menu_cnc_drill(){
  START_MENU();
  BACK_ITEM(MSG_CNC_MILLING);

  EDIT_ITEM(float3, MSG_CNC_MILLING_DEPTH, &millingZ, 0, 30);
  EDIT_ITEM(float5_25, MSG_CNC_MILLING_SPEED, &millSpeed, 1,CNC_MAX_DRILLING_SPEED);

  //MENU_ITEM(submenu,MSG_CNC_DRILL,lcd_menu_cnc_drill);
  CONFIRM_ITEM(MSG_CNC_DRILL,
      MSG_BUTTON_PROCEED, MSG_BUTTON_CANCEL,
      cnc_start_drilling, ui.goto_previous_screen,
      GET_TEXT_F(MSG_CNC_DRILL),GET_TEXT_F(MSG_CNC_DRILLING),  F("?")
    );

  END_MENU();
}



void menu_cnc_milling(){
  START_MENU();
  BACK_ITEM(MSG_CNC);

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

    SERIAL_ECHOLNPGM("Start scanning");
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
          SERIAL_ECHOLNPGM("Done Scanning");

          //return;
        }
        else if (n == -1)
          SERIAL_ERROR_MSG("Error reading SD Card");

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
    ui.manual_move.soon(Z_AXIS);
  }


void cnc_scan_move(){
    char cmd[50], str_1[16], str_2[16];

    //ui.clear_lcd();
    START_SCREEN();
      STATIC_ITEM(MSG_CNC_WORKPIECE, SS_CENTER|SS_INVERT);
      dtostrf(MINX, 1, 3, str_1);
      dtostrf(MAXX, 1, 3, str_2);
      sprintf_P(cmd, PSTR("[%s - %s]"),str_1,str_2);
      STATIC_ITEM(MSG_CNC_X,SS_LEFT, cmd);

      dtostrf(MINY, 1, 3, str_1);
      dtostrf(MAXY, 1, 3, str_2);
      sprintf_P(cmd, PSTR("[%s - %s]"),str_1,str_2);
      STATIC_ITEM(MSG_CNC_Y, SS_LEFT, cmd);
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
        cnc_process_command_P(F("G0 X0 Y0"));
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
  ui.set_status_P(GET_TEXT(MSG_CNC_SCANNING));
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
  START_SCREEN();
  CONFIRM_ITEM(MSG_CNC_MOVE,
      MSG_BUTTON_PROCEED, MSG_BUTTON_CANCEL,
      cnc_scan_move, ui.return_to_status,
      GET_TEXT_F(MSG_CNC_SCAN_BOX),GET_TEXT_F(MSG_CNC_START),  F("?")
    );
  END_SCREEN();
}

class MenuItem_sdfilecnc : public MenuItem_sdbase {
  public:
    static inline void draw(const bool sel, const uint8_t row, FSTR_P pstr, CardReader &theCard) {
        MenuItem_sdbase::draw(sel, row, pstr, theCard, false);
      }

    static void action(FSTR_P pstr, CardReader &)  {
      #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
        // Save menu state for the selected file
        sd_encoder_position = ui.encoderPosition;
        sd_top_line = encoderTopLine;
        sd_items = screen_items;
      #endif
      #if ENABLED(SD_MENU_CONFIRM_START)
        MenuItem_submenu::action(pstr, []{
          char * const longest = card.longest_filename();
          char buffer[strlen(longest) + 2];
          buffer[0] = ' ';
          strcpy(buffer + 1, longest);
          MenuItem_confirm::select_screen(
            GET_TEXT_F(MSG_BUTTON_PRINT), GET_TEXT_F(MSG_BUTTON_CANCEL),
            cnc_scan_file, ui.goto_previous_screen,
            GET_EN_TEXT_F(MSG_CNC_START), buffer, F("?")
          );
        });
      #else
        cnc_scan_file();
      #endif
    }
};

class MenuItem_sdfoldercnc : public MenuItem_sdbase {
  public:
    static inline void draw(const bool sel, const uint8_t row, FSTR_P pstr, CardReader &theCard) {
      MenuItem_sdbase::draw(sel, row, pstr, theCard, true);
    }
    static void action(FSTR_P const, CardReader &theCard) {
      card.cd(theCard.filename);
      encoderTopLine = 0;
      ui.encoderPosition = 2 * (ENCODER_STEPS_PER_MENU_ITEM);
      ui.screen_changed = true;
      TERN_(HAS_GRAPHICAL_LCD, ui.drawing_screen = false);
      ui.refresh();
    }
};


void menu_cnc_scan() {
  ui.encoder_direction_menus();

  #if HAS_MARLINUI_U8GLIB
    static int16_t fileCnt;
    if (ui.first_page) fileCnt = card.get_num_items();
  #else
    const int16_t fileCnt = card.get_num_items();
  #endif

  START_MENU();
    BACK_ITEM(MSG_CNC);

    card.getWorkDirName();
    if (card.filename[0] == '/') {
      #if !PIN_EXISTS(SD_DETECT)
        ACTION_ITEM(LCD_STR_REFRESH MSG_REFRESH, lcd_sd_refresh);
      #endif
    }
    else if (card.isMounted())
      ACTION_ITEM_F(F(LCD_STR_FOLDER " .."), lcd_sd_updir);
    
    if (ui.should_draw()) {
      for (int16_t i = 0; i < fileCnt; i++) {
        if (_menuLineNr != _thisItemNr)
          SKIP_ITEM();
        else {
          card.selectFileByIndexSorted(i);
          if (card.flag.filenameIsDir)
            MENU_ITEM(sdfoldercnc, MSG_MEDIA_MENU, card);
          else
            MENU_ITEM(sdfilecnc, MSG_MEDIA_MENU, card);
        }
      }
    }
  END_MENU();
}


/// This is the main menu for CNC commands and settings
void menu_cnc() {
  START_MENU();
  BACK_ITEM(MSG_MAIN_MENU); // back to ^ Main

  if (busy)
  {
    //the most used command here is to set the Z axis when changing tools so we put it first
    ACTION_ITEM(MSG_CNC_PROBE_Z, probing_z_down);
    MENU_ITEM(gcode, MSG_CNC_MOVE_WORKPIECE, F("G56\nG0 X0 Y0"));

  }else
  {
    ACTION_ITEM(MSG_CNC_RESET_ALL, reset_all_axes); //usually this is the first command to run as we do not home the CNC
    MENU_ITEM(submenu, MSG_CNC_PROBE,   menu_cnc_probing);
    ACTION_ITEM(MSG_CNC_TOOL_WORKSPACE,cnc_tool_workspace); //establish the cnc tool workspace

    //this creates and executes a macro to move to specific location and can be used during the tool change procedure
    // so M810 is set to move to the Tool Change position
    //MENU_ITEM(gcode, MSG_CNC_MOVE_TOOL_CHANGE, PSTR("M810 G55|G0 Z" STRINGIFY(TOOL_CHANGE_Z) "|G0 X" STRINGIFY(TOOL_CHANGE_X) " Y" STRINGIFY(TOOL_CHANGE_Y) "\nM810" ));
    MENU_ITEM(gcode, MSG_CNC_MOVE_TOOL_CHANGE, F("M810" ));

    ACTION_ITEM(MSG_CNC_SET_WORKPIECE_ORIGIN, set_workpiece_origin);

    MENU_ITEM(gcode, MSG_CNC_MOVE_WORKPIECE, F("G56\nG0 X0 Y0"));


    MENU_ITEM(submenu, MSG_CNC_MILLING, menu_cnc_milling);
    MENU_ITEM(submenu, MSG_CNC_SCAN,menu_cnc_scan);

    //settings
    MENU_ITEM(submenu, MSG_CONFIGURATION,menu_cnc_config);

    ACTION_ITEM(MSG_CNC_RESET_X, reset_x_axis);
    ACTION_ITEM(MSG_CNC_RESET_Y, reset_y_axis);
    ACTION_ITEM(MSG_CNC_RESET_Z, reset_z_axis);

    //
    // Disable Steppers
    //
    MENU_ITEM(gcode, MSG_DISABLE_STEPPERS, F("M84"));
    MENU_ITEM(gcode, MSG_CNC_DISABLE_Z, F("M84 Z"));
  }

  END_MENU();
}

#endif // HAS_DISPLAY
