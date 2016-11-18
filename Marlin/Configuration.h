//固件修改更新QQ群：224981313
//固件配套的3D打印機學習套件購買地址：http://item.taobao.com/item.htm?id=42916612391
//網盤資料：http://yunpan.taobao.com/s/19pI3jQStOL
//視頻教程 - v1,v2版機型軟件設置，自動調平
//優酷地址：http://www.youku.com/playlist_show/id_23218776.html
//視頻教程 - v3版機型組裝部分，即初八以後發貨的組裝視頻
//優酷地址：http://www.youku.com/playlist_show/id_23522533.html

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//===========================================================================
//==========================MICROMAKE 3D打印機配套固件 ======================
//===========================================================================
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ 
#define STRING_CONFIG_H_AUTHOR "(MICROMAKE KOSSEL)" 

#define SERIAL_PORT 0

//  串口通訊速率
#define BAUDRATE 250000

//// 驅動板類型
// 10 = Gen7 custom (Alfons3 Version) "https://github.com/Alfons3/Generation_7_Electronics"
// 11 = Gen7 v1.1, v1.2 = 11
// 12 = Gen7 v1.3
// 13 = Gen7 v1.4
// 2  = Cheaptronic v1.0
// 20 = Sethi 3D_1
// 3  = MEGA/RAMPS up to 1.2 = 3
// 33 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Bed)
// 34 = RAMPS 1.3 / 1.4 (Power outputs: Extruder0, Extruder1, Bed)
// 35 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Fan)
// 4  = Duemilanove w/ ATMega328P pin assignment
// 5  = Gen6
// 51 = Gen6 deluxe
// 6  = Sanguinololu < 1.2
// 62 = Sanguinololu 1.2 and above
// 63 = Melzi
// 64 = STB V1.1
// 65 = Azteeg X1
// 66 = Melzi with ATmega1284 (MaKr3d version)
// 67 = Azteeg X3
// 68 = Azteeg X3 Pro
// 7  = Ultimaker
// 71 = Ultimaker (Older electronics. Pre 1.5.4. This is rare)
// 72 = Ultimainboard 2.x (Uses TEMP_SENSOR 20)
// 77 = 3Drag Controller
// 8  = Teensylu
// 80 = Rumba
// 81 = Printrboard (AT90USB1286)
// 82 = Brainwave (AT90USB646)
// 83 = SAV Mk-I (AT90USB1286)
// 9  = Gen3+
// 70 = Megatronics
// 701= Megatronics v2.0
// 702= Minitronics v1.0
// 90 = Alpha OMCA board
// 91 = Final OMCA board
// 301= Rambo
// 21 = Elefu Ra Board (v3)

#ifndef MOTHERBOARD
#define MOTHERBOARD 33 //此處33為RAMPS 1.4擴展板類型
#endif

//液晶屏顯示的名字，不支持中文,此處作為新版本的版本號顯示
#define CUSTOM_MENDEL_NAME "UM v2.4"

// 定義擠出頭的數量
#define EXTRUDERS 1

//// 電源電壓電流
#define POWER_SUPPLY 1

//===========================================================================
//====================MICROMAKE 3D打印機 三角洲結構配置======================
//===========================================================================
#define DELTA

// //減小這個數值，來緩解卡頓現象，如修改為120進行測試。
#define DELTA_SEGMENTS_PER_SECOND 160

// 碳桿長度，從一端球中心到另一端球中心的距離 大小調整此參數
#define DELTA_DIAGONAL_ROD 217.3// mm

// 打印頭到滑桿水平距離 凹凸調整此參數
#define DELTA_SMOOTH_ROD_OFFSET 151// mm

// 效應器球中心和打印頭的水平距離
#define DELTA_EFFECTOR_OFFSET 24.0 // mm

// 滑車球中心到滑桿水平距離
#define DELTA_CARRIAGE_OFFSET 22.0 // mm

// 三角洲半徑.（打印頭到滑桿水平距離-效應器球中心和打印頭的水平距離-滑車球中心到滑桿水平距離）
#define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET)

// 打印半徑
#define DELTA_PRINTABLE_RADIUS 100.0

#define SIN_60 0.8660254037844386
#define COS_60 0.5
#define DELTA_TOWER1_X -SIN_60*DELTA_RADIUS
#define DELTA_TOWER1_Y -COS_60*DELTA_RADIUS
#define DELTA_TOWER2_X SIN_60*DELTA_RADIUS
#define DELTA_TOWER2_Y -COS_60*DELTA_RADIUS
#define DELTA_TOWER3_X 0.0
#define DELTA_TOWER3_Y DELTA_RADIUS

#define DELTA_DIAGONAL_ROD_2 pow(DELTA_DIAGONAL_ROD,2)

//===========================================================================
//========================MICROMAKE 3D打印機 傳感器設置======================
//===========================================================================

//配置傳感器，根據擠出機個數來配置連接傳感器數量。如果只有1個擠出機，則只需要開啟傳感器0接口和熱床接口即可。
#define TEMP_SENSOR_0 5 //設置傳感器0接口連接的傳感器類型編號，類型根據上面說明設置相應的編號
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0  //設置傳感器2接口連接的傳感器編號，0表示關閉該端口
#define TEMP_SENSOR_BED 0 //設置熱床傳感器端口連接的傳感器類型。該項如果設置錯誤將影響加熱床溫度控制
//添加熱床支持只需將#define TEMP_SENSOR_BED 處0設置為5即可

//這裡用傳感器1來做傳感器0的冗余。如果兩個傳感器溫度差較大，將停止打印。
//#define TEMP_SENSOR_1_AS_REDUNDANT  //設置傳感器1作為冗余傳感器。
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10, //設置溫度最大差值

// 打印之前通過M109檢查當前溫度已經接近設置溫度，並等待N秒作為緩衝。
#define TEMP_RESIDENCY_TIME 10  // 設置達到設置溫度後等待時間，單位秒
#define TEMP_HYSTERESIS 3       //設置離設置溫度的浮動範圍
#define TEMP_WINDOW     1      

//最低溫度低於N時，加熱頭將不會工作。該功能確保溫度傳感器連接或配置錯誤時不會燒燬設備。
//檢查熱敏電阻是否正常。
//如果熱門電阻工作不正常，將使加熱頭電源一直工作。這是非常危險的。

#define HEATER_0_MINTEMP 5 //設置加熱頭0的最小溫度，一般設置成室內最低溫度比較好。因為開機時應該測量到的是室溫。
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

//當溫度超過最大設置值，加熱頭會自動關閉。
//該項配置是為了保護你的設備，避免加熱溫度過高產生以外。但不能防止溫度傳感器非正常工作的情況。
//你應該使用MINTEMP選項來保證溫度傳感器短路或損壞時的設備安全。
#define HEATER_0_MAXTEMP 275 //擠出頭0 最大保護溫度
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define BED_MAXTEMP 120 //熱床最大保護溫度

//如果你的熱床電流較大，你可以通過設置占空比的方式降低電流，這個值應該是個整數，數字越大，電流越小。
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

//如果你想用M105命令來顯示加熱器的功耗，需要設置下面兩個參數
//#define EXTRUDER_WATTS (12.0*12.0/6.7) //  P=I^2/R
//#define BED_WATTS (12.0*12.0/1.1)      // P=I^2/R

//PID設置
#define PIDTEMP
#define BANG_MAX 255 
#define PID_MAX 255
#ifdef PIDTEMP
  #define PID_FUNCTIONAL_RANGE 10
  #define PID_INTEGRAL_DRIVE_MAX 255
  #define K1 0.95
  #define PID_dT ((OVERSAMPLENR * 8.0)/(F_CPU / 64.0 / 256.0)) 
    #define  DEFAULT_Kp 22.2
    #define  DEFAULT_Ki 1.08
    #define  DEFAULT_Kd 114
#endif 

#define MAX_BED_POWER 255 //通過占空比方式限制熱床的最大功率，255表示不限制
#ifdef PIDTEMPBED
    #define  DEFAULT_bedKp 10.00
    #define  DEFAULT_bedKi .023
    #define  DEFAULT_bedKd 305.4
#endif

//為了防止加熱頭未開啟時的冷擠出，這裡設置當加熱頭溫度未達到N時不允許擠出操作執行。（M302指令可以解除冷擠出限制）
#define PREVENT_DANGEROUS_EXTRUDE
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 175//設置擠出頭運行的最低溫度
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) //避免非常長的擠出操作

//===========================================================================
//============================= 機械方面設置 ================================
//===========================================================================

// #define COREXY //取消前面的註釋可以期待用corexy運動系統

// 限位開關設置
#define ENDSTOPPULLUPS  //將上面參數用「//」註釋掉，將禁用限位開關的上拉電阻。該配置是全局配置，不用該參數可以用下面單獨設置是否開啟上拉電阻

#ifndef ENDSTOPPULLUPS
 //分別對限位開關單獨設置上拉電阻。如果ENDSTOPPULLUPS被定義，該配置將被忽略
  // #define ENDSTOPPULLUP_XMAX
  // #define ENDSTOPPULLUP_YMAX
  // #define ENDSTOPPULLUP_ZMAX
  // #define ENDSTOPPULLUP_XMIN
  // #define ENDSTOPPULLUP_YMIN
  // #define ENDSTOPPULLUP_ZMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

//如果你使用機械式的限位開關，並且接到了信號和GND兩個接口，那麼上面的上拉配置需要打開
//配置3個軸的限位開關類型的，配置為true，限位開關應該接常開端子。如果你接常閉端子，則將true改為false
//設置為true來顛倒限位開關邏輯值。如果設置為true時，限位開關實際的開/合與檢測相反，則將該參數配置為false
const bool X_MIN_ENDSTOP_INVERTING = false; 
const bool Y_MIN_ENDSTOP_INVERTING = false; 
const bool Z_MIN_ENDSTOP_INVERTING = false;
const bool X_MAX_ENDSTOP_INVERTING = false; 
const bool Y_MAX_ENDSTOP_INVERTING = false;
const bool Z_MAX_ENDSTOP_INVERTING = false; 
//#define DISABLE_MAX_ENDSTOPS
//#define DISABLE_MIN_ENDSTOPS

//為了擋塊檢查程序的兼容性禁用最大終點擋塊
#if defined(COREXY) && !defined(DISABLE_MAX_ENDSTOPS)
  #define DISABLE_MAX_ENDSTOPS
#endif

//設置步進電機使能引腳的電平。（4988模塊保持0即可）
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // 針對所有擠出機有效

//當哪個軸不運動時是否關閉電機。（注意：如果這裡打開將會使電機在不使用時被鎖止，而導致電機溫度急劇上升）
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false //針對所有擠出機有效

//電機運動方向控制。由於電機連線不同，電機的運動方向也不同，但打印機的0點位置在左下角，如果電機的運動方向
//與控制方向不同，則可以將下面參數值true和false對調，也可以將步進電機的4根線反過來插。
#define INVERT_X_DIR true    // (X軸配置）
#define INVERT_Y_DIR true    // (Y軸配置）
#define INVERT_Z_DIR true    // (Z軸配置）
#define INVERT_E0_DIR false   //  (擠出機0配置）
#define INVERT_E1_DIR false   // (擠出機1配置）
#define INVERT_E2_DIR false   // (擠出機2配置）

//停止開關設置
//設置回0時，電機的運動方向。1最大限位方向，-1最小限位方向。一般都是設置為-1
#define X_HOME_DIR 1
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1
//軟限位開關設置
#define min_software_endstops false //最小值設置，如果設置為true，則移動距離<HOME_POS值
#define max_software_endstops true  //最大值設置，如果設置為true，軸不會移動到坐標大於下面定義的長度。

//各軸的軟件限位值
#define X_MAX_POS DELTA_PRINTABLE_RADIUS
#define X_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Y_MAX_POS DELTA_PRINTABLE_RADIUS
#define Y_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS
#define Z_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)
//================= MICROMAKE 3D打印機 自動調平配置 ====================

#define ENABLE_AUTO_BED_LEVELING // 是否開啟自動調平功能 

#ifdef ENABLE_AUTO_BED_LEVELING


  #define DELTA_PROBABLE_RADIUS (DELTA_PRINTABLE_RADIUS-50)//此處設置為調平探針移動範圍，增大調平範圍減少「-50」這個值，減少調平範圍增大「-50」這個值
  //如：#define DELTA_PROBABLE_RADIUS (DELTA_PRINTABLE_RADIUS-60)
  
  #define LEFT_PROBE_BED_POSITION -DELTA_PROBABLE_RADIUS
  #define RIGHT_PROBE_BED_POSITION DELTA_PROBABLE_RADIUS
  #define BACK_PROBE_BED_POSITION DELTA_PROBABLE_RADIUS
  #define FRONT_PROBE_BED_POSITION -DELTA_PROBABLE_RADIUS

  #define X_PROBE_OFFSET_FROM_EXTRUDER 0.0
  #define Y_PROBE_OFFSET_FROM_EXTRUDER 0.0
  #define Z_PROBE_OFFSET_FROM_EXTRUDER 0.5//自動調平設置 過高減小 過低增大

  #define Z_RAISE_BEFORE_HOMING 4       // 配置回原點前Z軸升起的高度，該高度要確保在Z軸最大高度範圍內。 
  
  #define XY_TRAVEL_SPEED 2000         //執行自動調平移動的速度，增大速度增加，減小速度降低

  #define Z_RAISE_BEFORE_PROBING 80  ////經過第一個檢測點前Z軸抬起的高度，該高度要確保調平傳感器可以正常放下。
  #define Z_RAISE_BETWEEN_PROBINGS 5  //經過下一個檢測點前Z軸抬起的高度

  #define Z_SAFE_HOMING   
  #ifdef Z_SAFE_HOMING

    #define Z_SAFE_HOMING_X_POINT (X_MAX_LENGTH/2)   
    #define Z_SAFE_HOMING_Y_POINT (Y_MAX_LENGTH/2)   

  #endif


  #define ACCURATE_BED_LEVELING
  #ifdef ACCURATE_BED_LEVELING
    #define ACCURATE_BED_LEVELING_POINTS 3 //自動調平探頭點點數 3為橫豎向各點3個點，共9點，改為6就是橫豎向各點6個點，共36個點。
    #define ACCURATE_BED_LEVELING_GRID_X ((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1))
    #define ACCURATE_BED_LEVELING_GRID_Y ((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1))
    #define NONLINEAR_BED_LEVELING
  #endif

#endif

//歸位開關設置
#define MANUAL_HOME_POSITIONS  //如果開啟該配置，下面 MANUAL_*_HOME_POS配置將生效
#define BED_CENTER_AT_0_0  //如果開啟該配置，熱床的中心位置在(X=0, Y=0) 

//手動回零開關的位置：
//對於三角洲結構這意味著笛卡爾打印機的頂部和中心的值。
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 200//306.6 // Z軸高度設置
//因每台機器安裝會有差別，需自行測量，測量方法請查看配套視頻教程，設置完後後記得保持修改

//軸設置
#define NUM_AXIS 4 //軸的數量，各軸的配置是順序是X, Y, Z, E
#define HOMING_FEEDRATE {60*60, 60*60, 60*60, 0}  //配置歸位時的速度

#define XYZ_FULL_STEPS_PER_ROTATION 200 //步進電機每週的步數，即360/步進電機上的角度
//如1.8度，步數應該是360/1.8=200；如果是0.9度電機的話就是 360/0.9=400。27號以前購買的用戶請修改為400，27號以後的用戶請修改為200。
#define XYZ_MICROSTEPS 16 //步進驅動的細分數
#define XYZ_BELT_PITCH 2 //同步帶齒間距
#define XYZ_PULLEY_TEETH 16 //同步輪齒數
#define XYZ_STEPS (XYZ_FULL_STEPS_PER_ROTATION * XYZ_MICROSTEPS / double(XYZ_BELT_PITCH) / double(XYZ_PULLEY_TEETH))
//這是計算公式：步進電機數*步進驅動的細分數/同步帶齒間距/同步輪齒數

#define DEFAULT_AXIS_STEPS_PER_UNIT   {XYZ_STEPS, XYZ_STEPS, XYZ_STEPS, 150}   //擠出機擠出量
#define DEFAULT_MAX_FEEDRATE          {200, 200, 200, 200}   
#define DEFAULT_MAX_ACCELERATION      {3000,3000,3000,3000}    

//加速度配置，假如打印時失步太大，可以將這個值改小
#define DEFAULT_ACCELERATION          3000    
#define DEFAULT_RETRACT_ACCELERATION  3000   

//各軸不需要加速的距離，即無需加速，立即完成的距離（即軟件認為他可以在瞬間完成的）
#define DEFAULT_XYJERK                20.0   
#define DEFAULT_ZJERK                 20.0    
#define DEFAULT_EJERK                 20.0  

//===========================================================================
//===============================附加功能====================================
//===========================================================================
//以下內容暫未漢化，後續會持續漢化更新，歡迎加入我們的交流QQ群：224981313
//感謝您的支持，也歡迎更多朋友可以持續補充，完善，歡迎散播複製，尊重勞動者，使用發佈請註明出處。
// EEPROM
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable EEPROM support
//#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
#define EEPROM_CHITCHAT

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 180
#define PLA_PREHEAT_HPB_TEMP 70
#define PLA_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

//LCD and SD support
//#define ULTRA_LCD  //general LCD support, also 16x2
//#define DOGLCD  // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
//#define SDSUPPORT // Enable SD Card Support in Hardware Console
//#define SDSLOW // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define ENCODER_PULSES_PER_STEP 1 // Increase if you have a high resolution encoder
//#define ENCODER_STEPS_PER_MENU_ITEM 5 // Set according to ENCODER_PULSES_PER_STEP or your liking
//#define ULTIMAKERCONTROLLER //as available from the Ultimaker online store.
//#define ULTIPANEL  //the UltiPanel as on Thingiverse
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000	// this is the tone frequency the buzzer plays when on UI feedback. ie Screen Click
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100 // the duration the buzzer plays the UI feedback sound. ie Screen Click

// The MaKr3d Makr-Panel with graphic controller and SD support
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//#define MAKRPANEL

// The RepRapDiscount Smart Controller (white PCB)
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
#define REPRAP_DISCOUNT_SMART_CONTROLLER

// The GADGETS3D G3D LCD/SD Controller (blue PCB)
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//#define G3D_PANEL

// The RepRapDiscount FULL GRAPHIC Smart Controller (quadratic white PCB)
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

// The RepRapWorld REPRAPWORLD_KEYPAD v1.1
// http://reprapworld.com/?products_details&products_id=202&cPath=1591_1626
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 // how much should be moved when a key is pressed, eg 10.0 means 10mm per click

// The Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
// REMEMBER TO INSTALL LiquidCrystal_I2C.h in your ARUDINO library folder: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//#define RA_CONTROL_PANEL

//automatic expansion
#if defined (MAKRPANEL)
 #define DOGLCD
 #define SDSUPPORT
 #define ULTIPANEL
 #define NEWPANEL
 #define DEFAULT_LCD_CONTRAST 17
#endif

#if defined (REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
 #define DOGLCD
 #define U8GLIB_ST7920
 #define REPRAP_DISCOUNT_SMART_CONTROLLER
#endif

#if defined(ULTIMAKERCONTROLLER) || defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
 #define ULTIPANEL
 #define NEWPANEL
#endif

#if defined(REPRAPWORLD_KEYPAD)
  #define NEWPANEL
  #define ULTIPANEL
#endif
#if defined(RA_CONTROL_PANEL)
 #define ULTIPANEL
 #define NEWPANEL
 #define LCD_I2C_TYPE_PCA8574
 #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
#endif

//I2C PANELS

//#define LCD_I2C_SAINSMART_YWROBOT
#ifdef LCD_I2C_SAINSMART_YWROBOT
  // This uses the LiquidCrystal_I2C library ( https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home )
  // Make sure it is placed in the Arduino libraries directory.
  #define LCD_I2C_TYPE_PCF8575
  #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
  #define NEWPANEL
  #define ULTIPANEL
#endif

// PANELOLU2 LCD with status LEDs, separate encoder and click inputs
//#define LCD_I2C_PANELOLU2
#ifdef LCD_I2C_PANELOLU2
  // This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
  // Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
  // (v1.2.3 no longer requires you to define PANELOLU in the LiquidTWI2.h library header file)
  // Note: The PANELOLU2 encoder click input can either be directly connected to a pin
  //       (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD
  #define NEWPANEL
  #define ULTIPANEL

  #ifndef ENCODER_PULSES_PER_STEP
	#define ENCODER_PULSES_PER_STEP 4
  #endif

  #ifndef ENCODER_STEPS_PER_MENU_ITEM
	#define ENCODER_STEPS_PER_MENU_ITEM 1
  #endif


  #ifdef LCD_USE_I2C_BUZZER
	#define LCD_FEEDBACK_FREQUENCY_HZ 1000
	#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
  #endif

#endif

// Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
//#define LCD_I2C_VIKI
#ifdef LCD_I2C_VIKI
  // This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
  // Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
  // Note: The pause/stop/resume LCD button pin should be connected to the Arduino
  //       BTN_ENC pin (or set BTN_ENC to -1 if not used)
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD (requires LiquidTWI2 v1.2.3 or later)
  #define NEWPANEL
  #define ULTIPANEL
#endif

// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection
//#define SR_LCD
#ifdef SR_LCD
   #define SR_LCD_2W_NL    // Non latching 2 wire shift register
   //#define NEWPANEL
#endif


#ifdef ULTIPANEL
//  #define NEWPANEL  //enable this if you have a click-encoder panel
  #define SDSUPPORT
  #define ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the DOG graphic display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 4
  #endif
#else //no panel but just LCD
  #ifdef ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the 128x64 graphics display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 16
    #define LCD_HEIGHT 2
  #endif
  #endif
#endif

// default LCD contrast for dogm-like LCD displays
#ifdef DOGLCD
# ifndef DEFAULT_LCD_CONTRAST
#  define DEFAULT_LCD_CONTRAST 32
# endif
#endif

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

// Temperature status LEDs that display the hotend and bet temperature.
// If all hotends and bed temperature and temperature setpoint are < 54C then the BLUE led is on.
// Otherwise the RED led is on. There is 1C hysteresis.
//#define TEMP_STAT_LEDS

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not ass annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT_PWM_SCALE.
//#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
// #define PHOTOGRAPH_PIN     23

// SF send wrong arc g-codes when using Arc Point as fillet procedure
//#define SF_ARC_FIX

// Support for the BariCUDA Paste Extruder.
//#define BARICUDA

//define BlinkM/CyzRgb Support
//#define BLINKM

/*********************************************************************\
* R/C SERVO support
* Sponsored by TrinityLabs, Reworked by codexmas
**********************************************************************/

// Number of servos
//
// If you select a configuration below, this will receive a default value and does not need to be set manually
// set it manually if you have more servos than extruders and wish to manually control some
// leaving it undefined or defining as 0 will disable the servo subsystem
// If unsure, leave commented / disabled
//
//#define NUM_SERVOS 3 // Servo index starts with 0 for M280 command

// Servo Endstops
//
// This allows for servo actuated endstops, primary usage is for the Z Axis to eliminate calibration or bed height changes.
// Use M206 command to correct for switch height offset to actual nozzle height. Store that setting with M500.
//
//#define SERVO_ENDSTOPS {-1, -1, 0} // Servo index for X, Y, Z. Disable with -1
//#define SERVO_ENDSTOP_ANGLES {0,0, 0,0, 70,0} // X,Y,Z Axis Extend and Retract angles

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H
