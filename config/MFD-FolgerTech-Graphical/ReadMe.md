
This configuration is based on the https://github.com/MarlinFirmware/Configurations/tree/bugfix-2.1.x/config/examples/FolgerTech/i3-2020

The configuration has been modified to match my printer.
 I am also removing some sections that are disabled and do not affect my build.

## My printer configuration:
 - replaced the text LCD with a graphic one.
 - Replaced the stepper drivers to DRV8825 using 1/32 micro-stepping
 - Moved the X Stop Sensor on the right side (X_MAX) while still keeping the origin on the left


Some of the configuration changes based on my needs:

### Extruder:

PID parameters: Measured on 2019-04-07 using "M303 E0 C8 S200" to run autotune on the hotend 0 at 200 degreesC for 8 cycles.
	#define DEFAULT_Kp 22.05 //MFD: was 11.50
	#define DEFAULT_Ki 1.33  //MFD: was  0.50
	#define DEFAULT_Kd 91.24 //MFD: was 60.00

  #define HOTEND_IDLE_TIMEOUT  //MFD: enabled this

  * Linear Pressure Control v1.5
  See https://marlinfw.org/docs/features/lin_advance.html for full instructions.
  #define LIN_ADVANCE   //MFD: enabled this


### Bed:
PID Parameters: Measured on 2019-04-07 using "M303 E-1 C8 S60" to run autotune on the bed at 60 degreesC for 8 cycles.
  #define DEFAULT_bedKp 151.42 //MFD: was 250.0
  #define DEFAULT_bedKi 9.88   //MFD: was  18.0
  #define DEFAULT_bedKd 579.91 //MFD: was 950.0 

### X-Axis:
Moved X Endstop on the right side X_MAX:
  #define X_MIN_ENDSTOP_HIT_STATE HIGH  //MFD: reversed the position of the endstop
  #define X_MAX_ENDSTOP_HIT_STATE LOW   //MFD: reversed the position of the endstop
  #define ENDSTOP_INTERRUPTS_FEATURE //MFD: enabled endstop interrupt
  #define INVERT_X_DIR true //MFD: was false
  #define X_HOME_DIR 1 //MFD: was -1

### Motion:
DRV8825 1/32 microsteps:
  #define DEFAULT_AXIS_STEPS_PER_UNIT  {160,160,8000,191.4} steps/mm using the 1/32 driver DRV8825 (E was measured on 2019_5_25)

Travel limits (linear=mm, rotational=°) after homing, corresponding to endstop positions.
  #define X_MIN_POS 0 //MFD was 6
  #define Y_MIN_POS 0 //MFD was 3
  #define Z_MIN_POS 0
  #define X_MAX_POS 210 //MFD was 207
  #define Y_MAX_POS 220 //MFD was 182
  #define Z_MAX_POS 175

#define QUICK_HOME               //MFD: enabled this           // If G28 contains XY do a diagonal move first

#define INDIVIDUAL_AXIS_HOMING_MENU //MFD: enabled this

  * G5 Bézier Curve Support with XYZE destination and IJPQ offsets
  #define BEZIER_CURVE_SUPPORT  //MFD: enabled this

  * Add G10 / G11 commands for automatic firmware-based retract / recover.
  Use M207 and M208 to define parameters for retract / recover.
  Use M209 to enable or disable auto-retract.
  #define FWRETRACT //MFD enabled this
  // #define FWRETRACT_AUTORETRACT   //MFD disabled this as the printer stops after first move
  #define RETRACT_LENGTH                1    //MFD was 3   // (mm) Default retract length (positive value)
  

  * S-Curve Acceleration:
  See https://github.com/synthetos/TinyG/wiki/Jerk-Controlled-Motion-Explained
  #define S_CURVE_ACCELERATION //MFD: enabled this

Bed Leveling with Manual Probe:
  #define PROBE_MANUALLY //MFD: enabled this
  #define LCD_BED_LEVELING //MFD: enabled this
  #define LCD_BED_TRAMMING  //MFD: enabled this



### ENCODER:
  
  #define ENCODER_PULSES_PER_STEP 3 //MFD: was  4 and enabled this

  #define ENCODER_STEPS_PER_MENU_ITEM 4 //MFD: was 1 and enabled this



### SPEAKER/BUZZER:
  #define SPEAKER //MFD enabled this

  #define BEEP_ON_FEEDRATE_CHANGE  //MFD: enabled this

### FAN:
  #define FAN_MIN_PWM 50  //MFD: enabled this
  
  * Turn off print-cooling fans while the machine is paused
  #define ADVANCED_PAUSE_FANS_PAUSE  //MFD: enabled this           

### UI Menus:
  
  #define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER  //MFD: enabled this one
  
  * Scroll a longer status message into view
  #define STATUS_MESSAGE_SCROLLING //MFD: enabled this

  * Show the E position (filament used) during printing
  #define LCD_SHOW_E_TOTAL //MFD: enabled this

  * Force the media menu to be listed on the top of the main menu
  #define MEDIA_MENU_AT_TOP    //MFD: enabled this  

  * Sort SD file listings in alphabetical order
  #define SDCARD_SORT_ALPHA //MFD: enabled this

  * Get the long filename of a file/folder with 'M33 <dosname>' and list long filenames with 'M20 L'
  #define LONG_FILENAME_HOST_SUPPORT   //MFD: enabled this 

  * Scroll long filenames in the SD card menu
  #define SCROLL_LONG_FILENAMES       //MFD: enabled this  

  * Allow babystepping at all times (not just during movement)
  #define BABYSTEP_ALWAYS_AVAILABLE    //MFD: enabled this    

  * Display total babysteps since last G28
  #define BABYSTEP_DISPLAY_TOTAL   //MFD: enabled this       

### G-CODE:
  * Add 'M73' to set print job progress, overrides Marlin's built-in estimate
  #define SET_PROGRESS_MANUALLY  //MFD: enabled this

  * Enable G-code M808 to set repeat markers and do looping
  #define GCODE_REPEAT_MARKERS    //MFD: enabled this    

  * Add M701/M702 Load/Unload G-codes, plus Load/Unload in the LCD Prepare menu.
  #define FILAMENT_LOAD_UNLOAD_GCODES       //MFD: enabled this    

  * Add the M240 G-code to take a photo.
  #define PHOTO_GCODE   //MFD: enabled this
  #define PHOTO_DELAY_MS   100        //MFD: enabled this                    // (ms) Duration to pause before moving back (M240 P)
  #define PHOTO_RETRACT_MM   6.5      //MFD: enable this                    // (mm) E retract/recover for the photo move (M240 R S)
  #define PHOTOGRAPH_PIN 23           //MFD: enabled this

  * Define a variable from 100-115 with G-code like '#101=19.6'.
  A variable can then be used in a G-code expression like 'G0 X[#101+3]'.
  See https://gcodetutor.com/cnc-macro-programming/cnc-variables.html
  #define GCODE_VARIABLES   //MFD: enabled this

  * Add M360 commands originally from Repetier FW
  #define REPETIER_GCODE_M360  //MFD: enabled this   

  * Add M42 - Set pin states
  #define DIRECT_PIN_CONTROL  //MFD: enabled this

  * Add M43 - display pin status, toggle pins, watch pins, watch endstops & toggle LED, test servo probe
  #define PINS_DEBUGGING     //MFD: enabled this

### Statistics:
  * View the current statistics with M78.
  #define PRINTCOUNTER //MFD: enabled this

  * Include a page of printer information in the LCD Main Menu
  #define LCD_INFO_MENU  //MFD: enabled this


### MISC:
  * Shrink the build for smaller boards by sacrificing some serial feedback
  #define MARLIN_SMALL_BUILD  //MFD: enabled this





  16:42:55.456 > start
16:42:55.457 > echo: External Reset
16:42:55.457 > Marlin bugfix-2.0.x
16:42:55.461 >
16:42:55.461 > echo: Last Updated: 2019-05-29 | Author: (none, default config)
16:42:55.461 > echo:Compiled: May 29 2019
16:42:55.461 > echo: Free Memory: 2010  PlannerBufferBytes: 1632
16:42:55.482 > Unified Bed Leveling System v1.01 inactive.
16:42:55.530 > 
16:42:55.530 > Unified Bed Levelingecho:  G21    ; Units in mm (mm)
16:42:55.534 > echo:  M149 C ; Units in Celsius
16:42:55.534 >
16:42:55.534 > echo:Filament settings: Disabled
16:42:55.535 > echo:  M200 D1.75
16:42:55.538 > echo:  M200 D0
16:42:55.538 > echo:Steps per unit:
16:42:55.538 > echo: M92 X160.00 Y160.00 Z8000.00 E191.40
16:42:55.542 > echo:Maximum feedrates (units/s):
16:42:55.542 > echo:  M203 X500.00 Y500.00 Z2.00 E120.00
16:42:55.542 > echo:Maximum Acceleration (units/s2):
16:42:55.547 > echo:  M201 X4000.00 Y4000.00 Z4.00 E10000.00
16:42:55.547 > echo:Acceleration (units/s2): P<print_accel> R<retract_accel> T<travel_accel>
16:42:55.551 > echo:  M204 P1500.00 R1500.00 T1500.00
16:42:55.551 > echo:Advanced: B<min_segment_time_us> S<min_feedrate> T<min_travel_feedrate> J<junc_dev> 
16:42:55.555 > echo:  M205 B20000.00 S0.00 T0.00 J0.02
16:42:55.559 > echo:Unified Bed Leveling:
16:42:55.559 > echo:  M420 S0 Z10.00
16:42:55.559 >
16:42:55.559 > Unified Bed Leveling System v1.01 inactive.
16:42:55.613 > 
16:42:55.613 > Active Mesh Slot: -1
16:42:55.613 > EEPROM can hold 8 meshes.
16:42:55.613 >
16:42:55.613 > echo:Material heatup parameters:
16:42:55.617 > echo:  M145 S0 H180 B70 F0
16:42:55.617 > echo:  M145 S1 H240 B110 F0
16:42:55.617 > echo:PID settings:
16:42:55.617 > echo:  M301 P22.05 I1.33 D91.24
16:42:55.621 > echo:  M304 P151.42 I9.88 D579.91
16:42:55.621 > echo:Retract: S<length> F<units/m> Z<lift>
16:42:55.625 > echo:  M207 S3.00 W13.00 F1500.00 Z0.00
16:42:55.625 > echo:Recover: S<length> F<units/m>
16:42:55.625 > echo:  M208 S0.00 W0.00 F480.00
16:42:55.629 > echo:Linear Advance:
16:42:55.629 > echo:  M900 K0.04
16:42:55.629 > echo:Filament load/unload lengths:
16:42:55.629 > echo:  M603 L0.00 U100.00

M501
16:45:36.843 > Unified Bed Leveling System v1.01 inactive.
16:45:36.896 > 
16:45:36.896 > Unified Bed Levelingecho:  G21    ; Units in mm (mm)
16:45:36.896 > echo:  M149 C ; Units in Celsius
16:45:36.896 >
16:45:36.896 > echo:Filament settings: Disabled
16:45:36.900 > echo:  M200 D1.75
16:45:36.900 > echo:  M200 D0
16:45:36.900 > echo:Steps per unit:
16:45:36.900 > echo: M92 X160.00 Y160.00 Z8000.00 E191.40
16:45:36.904 > echo:Maximum feedrates (units/s):
16:45:36.904 > echo:  M203 X500.00 Y500.00 Z2.00 E120.00
16:45:36.909 > echo:Maximum Acceleration (units/s2):
16:45:36.909 > echo:  M201 X4000.00 Y4000.00 Z4.00 E10000.00
16:45:36.913 > echo:Acceleration (units/s2): P<print_accel> R<retract_accel> T<travel_accel>
16:45:36.913 > echo:  M204 P1500.00 R1500.00 T1500.00
16:45:36.917 > echo:Advanced: B<min_segment_time_us> S<min_feedrate> T<min_travel_feedrate> J<junc_dev> 
16:45:36.921 > echo:  M205 B20000.00 S0.00 T0.00 J0.02
16:45:36.921 > echo:Unified Bed Leveling:
16:45:36.921 > echo:  M420 S0 Z10.00
16:45:36.921 >
16:45:36.921 > Unified Bed Leveling System v1.01 inactive.
16:45:36.973 > 
16:45:36.974 > Active Mesh Slot: -1
16:45:36.978 > EEPROM can hold 8 meshes.
16:45:36.978 >
16:45:36.978 > echo:Material heatup parameters:
16:45:36.978 > echo:  M145 S0 H180 B70 F0
16:45:36.978 > echo:  M145 S1 H240 B110 F0
16:45:36.982 > echo:PID settings:
16:45:36.982 > echo:  M301 P22.05 I1.33 D91.24
16:45:36.982 > echo:  M304 P151.42 I9.88 D579.91
16:45:36.986 > echo:Retract: S<length> F<units/m> Z<lift>
16:45:36.986 > echo:  M207 S3.00 W13.00 F1500.00 Z0.00
16:45:36.986 > echo:Recover: S<length> F<units/m>
16:45:36.990 > echo:  M208 S0.00 W0.00 F480.00
16:45:36.990 > echo:Linear Advance:
16:45:36.990 > echo:  M900 K0.04
16:45:36.990 > echo:Filament load/unload lengths:
16:45:36.994 > echo:  M603 L0.00 U100.00
16:45:36.994 > ok