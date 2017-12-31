# Copyright 2016 Nikolai Ozerov (VE3NKL)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Version 0.79. Last updated on Oct 09, 2016.

#include <stdint.h>
#include <DS3231.h>
#include <Wire.h>

#include <EEPROM.h>

#include <LiquidCrystal_I2C.h>

#define VERSION "0.79"       // Current microcode version

#define DS3231_ADDRESS 0x86  // RTC I2C Address

#define EBS_UPPER_LIMIT 1024 // Upper limit for EBS storage
#define EBS_START_ADDRESS 24 // Starting address of EBS storage in EEPROM
#define EBS_N_BLOCKS      10 // Number of blocks
#define EBS_BLOCK_SIZE   100 // Block size
#define EBS_BLOCK_ID    0x5a // Block identifier

#define SYM_ARROW_DOWN 0x03  // Special ARROW DOWN symbol
#define SYM_DEGREE 0x05      // Special degree symbol       
#define SYM_SELECTOR 0x06    // Special SELECTOR symbol
#define SYM_COMM 0x07        // Special Communication symbol

#define DISPLAY_LINE_SIZE 16 // Display line size in characters

#define EEPROM_AZIMUTH 4     // Azimuth address in EEPROM
#define EEPROM_ELEVATION 6   // Elevation address in EEPROM
#define EEPROM_OFFSET 8      // Balance Position Offset address in EEPROM
#define EEPROM_MAXOFFS 10    // Maximum allowed offset address in EEPROM

#define PULSE_INT_PIN 7      // Pin for an external interrupts to
// count pulses from Alpha Rotator

#define BUTTON_LEFT_PIN 6
#define BUTTON_RIGHT_PIN 8
#define BUTTON_UP_PIN 9
#define BUTTON_DOWN_PIN 4
#define BUTTON_MENU_PIN A4

#define RELAY_AZ1_PIN 5      // Azimuth control relay 1 pin
#define RELAY_AZ2_PIN 10     // Azimuth control relay 2 pin
#define RELAY_EL1_PIN 11     // Elevation control relay 1 pin
#define RELAY_EL2_PIN 12     // Elevation control relay 2 pin

#define BUTTON_VIBRATION_THRESHOLD 50       // 50 ms
#define BUTTON_HELD_LONG_CALL_INTERVAL 750  // 0.5 s

#define PULSE_TOO_SHORT 20   // Pulse changes less than 20 ms are wrong

#define JAMMED_DETECTION_TIME 750 // 3/4 of a second w/o rotation indication means rotor is jammed

const unsigned long max_ul = 4294967295UL;

DS3231 RTC; //Create the DS3231 object

static uint8_t bin2bcd (uint8_t val) {
  return val + 6 * (val / 10);
}

const unsigned long time_sync_interval = 3600000UL; // Synchronize with RTC every hour

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

const byte degree[] PROGMEM      = { 0x08, 0x14, 0x14, 0x08, 0x00, 0x00, 0x00, 0x00 };
const byte arrow_down[] PROGMEM  = { 0x04, 0x04, 0x04, 0x04, 0x04, 0x15, 0x0e, 0x04 };
const byte arrow_up[] PROGMEM    = { 0x04, 0x0e, 0x15, 0x04, 0x04, 0x04, 0x04, 0x04 };
const byte arrow_right[] PROGMEM = { 0x04, 0x02, 0x1f, 0x02, 0x04, 0x00, 0x00, 0x00 };
const byte arrow_left[] PROGMEM  = { 0x04, 0x08, 0x1f, 0x08, 0x04, 0x00, 0x00, 0x00 };
const byte selector[] PROGMEM    = { 0x08, 0x0c, 0x0e, 0x0f, 0x0e, 0x0c, 0x08, 0x00 };
const byte communic[] PROGMEM    = { 0x0e, 0x06, 0x0a, 0x10, 0x02, 0x14, 0x18, 0x1c };

const char splashText1[] PROGMEM = " VE3NKL Rotator";
const char splashText2[] PROGMEM = "Controller V";
const char splashText3[] PROGMEM = VERSION;

// ------------------------------------------------------------------ //
// EEPROM BLOCK STORAGE                                               //
// ------------------------------------------------------------------ //

uint8_t ebs_free_ix  = 0;          // Index of the next free block
uint8_t ebs_next_seq = 0;          // Expected next SEQ block number
uint8_t ebs_error_code = 0;        // Error code set by ESB_Open
uint8_t ebs_current_ix = 0;        // Index of the current block

unsigned long ebs_block_map;       // Each bit maps a block with usable data

// ------------------------------------------------------------------ //
// Frequently used strings                                            //
// ------------------------------------------------------------------ //

char EXCEPTION_LEFT_BOUNDARY[]  = "[ <-\0";
char EXCEPTION_RIGHT_BOUNDARY[] = "-> ]\0";
char EXCEPTION_JAMMED[]         = "JAMD\0";

// ------------------------------------------------------------------ //
// Button anti-vbration service. bstate_xxxx is set to 0 when the     //
// corresponding button is pressed.                                   //
// ------------------------------------------------------------------ //

unsigned long tms_left = 0;
int bstate_left = 1;
int tstate_left = 1;
unsigned long tms_right = 0;
int bstate_right = 1;
int tstate_right = 1;
unsigned long tms_up = 0;
int bstate_up = 1;
int tstate_up = 1;
unsigned long tms_down = 0;
int bstate_down = 1;
int tstate_down = 1;
unsigned long tms_menu = 0;
int bstate_menu = 1;
int tstate_menu = 1;

unsigned long held_long_called = 0;

const int KEY_NONE = 0;
const int KEY_LEFT = 1;
const int KEY_RIGHT = 2;
const int KEY_UP = 3;
const int KEY_DOWN = 4;
const int KEY_MENU = 5;

int pressed_button = KEY_NONE;

// ------------------------------------------------------------------ //
// Date and Time service (RTC based on DS3231)                        //
// ------------------------------------------------------------------ //

DateTime currentDateTime;

unsigned long initMs = 0;
unsigned long syncTime = 0;

const int monthsNorm[] PROGMEM = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
const int monthsLeap[] PROGMEM = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};

const char monthsString[] PROGMEM = "JanFebMarAprMayJunJulAugSepOctNovDec";

long days = 0L;
long secs = 0L;

char yearStr[5];
char dateStr[9];
char timeStr[9];
char statStr[16];

char text[DISPLAY_LINE_SIZE+1];
char items_w_buf[DISPLAY_LINE_SIZE+1];

// ------------------------------------------------------------------ //
// Internal function definitions                                      //
// ------------------------------------------------------------------ //

void incomingCommand(char* cmd);
void modeSwitchingController();

void selectMenuItem(struct MenuItem_T* item, boolean initialize);
int getAzimuth();
void setAzimuth(int);
int getElevation();
void setElevation(int);
int getBalance();
void setBalance(int pos);
int getMaxOffs();
void setMaxOffs(int maxOffs);
void switchToManualMode();
void returnToBalance();
void saveParameters();

void pulseDetector();             // Definition for our interrupt handler

int getYear();
void setYear(int y);
int getMonth();
void setMonth(int m);
int getDay();
void setDay(int d);
int getHours();
void setHours(int hh);
int getMinutes();
void setMinutes(int mm);
void zeroSeconds();

void setException(char* exc);
void createLcdChar(int n, const byte* codes);

int getNumSatPassItems();
int getCurrentPassItem();
void formatSatPassItemDescr(int n, char* descr);
void passItemConfirm(int n);

// ------------------------------------------------------------------ //
// Communication fields and function definitions                      //
// ------------------------------------------------------------------ //

char commBuf[21];
int  commBufIx = 0;
char c;
char comm_indicator = ' ';
int Comm_Speed = 8;

unsigned long last_comm_activity = 0UL;

volatile bool updateStatLine = false;

// Date and time related definitions

int datePosition = -1;          // Negative value means do not print date
int datePosLine  = 0;           // Line number
int timePosition = -1;          // Negative value means do not print time
int timePosLine  = 0;           // Line number
const int TF_HHMM = 1;          // Time format hh:mm
const int TF_HHMMSS = 2;        // Time format hh:mm:ss
const int TF_DELTA  = 3;        // Delta time format +..d..h/+..h..m/+..:..
int timeFormat   =  TF_HHMM;    // Initial time format

// ------------------------------------------------------------------ //
// Satellite Pass description for "offline" tracking                  //
// ------------------------------------------------------------------ //

typedef struct PassDescription_T {
  char satelliteName[8];             // Satellite name (id)
  long daysAOS;                      // Day number for AOS
  long secsAOS;                      // Second number in the day for AOS
  
  int  interval;                     // Length between LOS and AOS in seconds
  int  half_point_y;                 // Half-point of the working range for calculations of azimuth  
  int  half_point_z;                 // Half-point of the working range for calculations of elevation
  
  uint8_t ncoef_y;                   // Number of coefficients to be used for calculations of azimuth  
  uint8_t ncoef_z;                   // Number of coefficients to be used for calculations of elevation
  float coef[18];                    // Polynomial coefficients for position calculation
};

struct PassDescription_T satPass01;
struct PassDescription_T * currentSatPassDescr = NULL;

// ------------------------------------------------------------------ //
// Definition of Menu structure and menu items                        //
// ------------------------------------------------------------------ //

uint8_t MIF_Hidden_Menu_Count = 0;

const uint8_t MIT_Group = 1;
const uint8_t MIT_Integer = 2;
const uint8_t MIT_Selection = 3;
const uint8_t MIT_List = 4;

typedef struct MenuItem_T {
  uint8_t type;                        // Item type (see MIT_ constants)
  char*   text;                        // Text representation of the item
  struct  MenuItem_T* parent;
  struct  MenuItem_T* youngerBrother;  // Reference to the next sibling
};

typedef struct MenuItemGroup_T {
  struct MenuItem_T base;

  struct MenuItem_T* firstChild;       // Reference to the first child
  uint8_t childPos[8];
  uint8_t currentChildIx;
  uint8_t nofChildren;
};

typedef struct MenuItemInteger_T {
  struct MenuItem_T base;

  int               minValue;
  int               maxValue;
  int            (* getValueCallback)();
  void           (* setValueCallback)(int);
  char*             decorator;

  int               value;
  int               tempValue;
};

typedef struct MenuItemSelection_T {
  struct MenuItem_T base;

  void           (* confirmedCallback)();
};

typedef struct MenuItemList_T {
  struct MenuItem_T base;

  int            (* initNofItemsCallback)();
  int            (* initCurrentItemNumCallback)();
  void           (* formatItemDescrCallback)(int, char*);
  void           (* confirmNewItemNumCallback)(int);

  int            nof_items;
  int            cur_item;
  int            temp_item;
};

typedef union MIF_ItemTypes {
  struct MenuItem_T * item;
  struct MenuItemGroup_T* g;
  struct MenuItemInteger_T* i;
  struct MenuItemSelection_T* s;
  struct MenuItemList_T* l;
};

void MIF_init(struct MenuItem_T* item);

struct MenuItemGroup_T mi_root       = { { MIT_Group, "Set", NULL } };
struct MenuItemGroup_T mi_action     = { { MIT_Group, "Act",  (struct MenuItem_T *) &mi_root } };
struct MenuItemSelection_T mi_manual = { { MIT_Selection, "Man", (struct MenuItem_T *) &mi_action }, &switchToManualMode };
struct MenuItemSelection_T mi_auto   = { { MIT_Selection, "Aut", (struct MenuItem_T *) &mi_action }, &switchToAutoMode };
struct MenuItemList_T mi_satpass     = { { MIT_List, "P", (struct MenuItem_T *) &mi_action }, &getNumSatPassItems, &getCurrentPassItem, &formatSatPassItemDescr, &passItemConfirm };
struct MenuItemGroup_T mi_commands   = { { MIT_Group, "Cmd", (struct MenuItem_T *) &mi_action } };
struct MenuItemSelection_T mi_zbalan = { { MIT_Selection, "Park", (struct MenuItem_T *) &mi_commands }, &returnToBalance };
struct MenuItemSelection_T mi_save   = { { MIT_Selection, "Save", (struct MenuItem_T *) &mi_commands }, &saveParameters };
struct MenuItemGroup_T mi_position   = { { MIT_Group, "Dir", (struct MenuItem_T *) &mi_root } };
struct MenuItemInteger_T mi_azim     = { { MIT_Integer, "Az", (struct MenuItem_T *) &mi_position }, 0, 359, &getAzimuth, &setAzimuth, "\x05" };
struct MenuItemInteger_T mi_elev     = { { MIT_Integer, "El", (struct MenuItem_T *) &mi_position }, 0, 90, &getElevation, &setElevation, "\x05" };
struct MenuItemGroup_T mi_cable      = { { MIT_Group, "BPO", (struct MenuItem_T *) &mi_root } };
struct MenuItemInteger_T mi_balance  = { { MIT_Integer, "Of", (struct MenuItem_T *) &mi_cable }, -540, 540, &getBalance, &setBalance, "\x05" };
struct MenuItemInteger_T mi_maxoffs  = { { MIT_Integer, "Mx", (struct MenuItem_T *) &mi_cable }, 0, 540, &getMaxOffs, &setMaxOffs, "\x05" };
struct MenuItemGroup_T mi_other      = { { MIT_Group, "Oth", (struct MenuItem_T *) &mi_root } };
struct MenuItemGroup_T mi_date       = { { MIT_Group, "Date", (struct MenuItem_T *) &mi_other } };
struct MenuItemGroup_T mi_time       = { { MIT_Group, "Time", (struct MenuItem_T *) &mi_other } };
struct MenuItemList_T mi_comm        = { { MIT_List,  ".Comm", (struct MenuItem_T *) &mi_other }, &commGetNumOfSpeeds, &commGetCurrentSpeed, &commFormatSpeed, &commSpeedConfirm };
struct MenuItemInteger_T mi_year     = { { MIT_Integer, "Y", (struct MenuItem_T *) &mi_date }, 0, 99, &getYear, &setYear, "" };
struct MenuItemInteger_T mi_month    = { { MIT_Integer, "M", (struct MenuItem_T *) &mi_date }, 1, 12, &getMonth, &setMonth, "" };
struct MenuItemInteger_T mi_day      = { { MIT_Integer, "D", (struct MenuItem_T *) &mi_date }, 1, 31, &getDay, &setDay, "" };
struct MenuItemInteger_T mi_hours    = { { MIT_Integer, "H", (struct MenuItem_T *) &mi_time }, 0, 23, &getHours, &setHours, "" };
struct MenuItemInteger_T mi_minutes  = { { MIT_Integer, "M", (struct MenuItem_T *) &mi_time }, 0, 59, &getMinutes, &setMinutes, "" };
struct MenuItemSelection_T mi_secs   = { { MIT_Selection, "Zero", (struct MenuItem_T *) &mi_time }, &zeroSeconds };

struct MenuItem_T* currentMenuItem = NULL;

struct MenuItem_T* all_items[] = {
  (MenuItem_T*)&mi_root,
  (MenuItem_T*)&mi_action,
  (MenuItem_T*)&mi_manual,
  (MenuItem_T*)&mi_auto,
  (MenuItem_T*)&mi_satpass,
  (MenuItem_T*)&mi_commands,
  (MenuItem_T*)&mi_zbalan,
  (MenuItem_T*)&mi_save,
  (MenuItem_T*)&mi_position,
  (MenuItem_T*)&mi_azim,
  (MenuItem_T*)&mi_elev,
  (MenuItem_T*)&mi_cable,
  (MenuItem_T*)&mi_balance,
  (MenuItem_T*)&mi_maxoffs,
  (MenuItem_T*)&mi_other,
  (MenuItem_T*)&mi_date,
  (MenuItem_T*)&mi_time,
  (MenuItem_T*)&mi_comm,
  (MenuItem_T*)&mi_year,
  (MenuItem_T*)&mi_month,
  (MenuItem_T*)&mi_day,
  (MenuItem_T*)&mi_hours,
  (MenuItem_T*)&mi_minutes,
  (MenuItem_T*)&mi_secs
};

// ------------------------------------------------------------------ //
// Balance position. Antenna can be rotated from -Max to +Max         //
// degrees from the Balance position to avoid cable stress.           //
// ------------------------------------------------------------------ //

int Balance_Position = 0;
int Max_Offs = 270;

// ------------------------------------------------------------------ //
// Exception-related fields                                           //
// ------------------------------------------------------------------ //

char* Exception = "";
int Exception_Row = -1;              // Do not display exceptions if
int Exception_Col = 0;               // Row value is negative 


// ------------------------------------------------------------------ //
// Horizontal rotation indicator                                      //
// ------------------------------------------------------------------ //

// 'Horizontal_Direction' holds the current status of relays controlling 
// the horizontal rotator. It is changed in accord with changing the relay
// control voltage.
// 'Horizontal_Direction' is set by the rotationController function only.
// When there is a need to control the rotator, the 'Horizontal_Request'
// variable should be set to the new desired status. The rotationController
// will make sure that the control is executed at the appropriate time.
// 'Last_Horizontal_Direction' has the HORIZONTAL_STP value only after
// initalization (power on) of the Rotator Controller. After the first 
// horizontal move it will be assigned either HORIZONTAL_CW or HORIZONTAL_CCW
// value which will stay even after the rotator is stopped. 

const int HORIZONTAL_STP = 0;  // Motor stopped
const int HORIZONTAL_CW  = 1;  // Clock-wise
const int HORIZONTAL_CCW = 2;  // Counter clock-wise
const int HORIZONTAL_IMM = -1; // Stop the motor immediately (Request only)

volatile int Horizontal_Direction = HORIZONTAL_STP;
volatile int Horizontal_Position = 0;
volatile unsigned long Horizontal_Motion_Time = 0;
volatile int Last_Horizontal_Direction = HORIZONTAL_STP;

// ------------------------------------------------------------------ //
// Horizontal rotation request                                        //
// ------------------------------------------------------------------ //

int Horizontal_Request = HORIZONTAL_STP;

// ------------------------------------------------------------------ //
// Horizontal Target position request                                 //
// ------------------------------------------------------------------ //

const int HORIZONTAL_TARGET_NOT_SET = -32767;
const int HORIZONTAL_TARGET_AZIMUTH = -32766;

int Horizontal_Target_Offset  = HORIZONTAL_TARGET_NOT_SET;
int Horizontal_Target_Azimuth = 0;

const int PREPROG_TARGET_DELTA_UNDEFINED = 32767;
int PreProg_Target_Delta = PREPROG_TARGET_DELTA_UNDEFINED;

int PreProg_Initial_Offset = 0;

// When in the PREPROG_WRK mode and calculations started, after each
// calculation we remember the 'secs' field to avoid making calculations again
// for the same second.

long secsForLastCalculation = -1L;
                 

// ------------------------------------------------------------------ //
// Vertical movement indicator                                        //
// ------------------------------------------------------------------ //

const int VERTICAL_STP  = 0;   // Motor stopped
const int VERTICAL_UP   = 1;   // Up
const int VERTICAL_DOWN = 2;   // Down

volatile int Vertical_Direction = VERTICAL_STP;
volatile int Vertical_Position = 0;

// ------------------------------------------------------------------ //
// Vertical rotation request                                          //
// ------------------------------------------------------------------ //

volatile int Vertical_Request = VERTICAL_STP;

// ------------------------------------------------------------------ //
// Maintain Alpha-Rotor pulse status for updating azimuth             //
// ------------------------------------------------------------------ //

const int PULSE_OFF = 0;               // Pulse status can be ON. OFF //
const int PULSE_ON  = 1;               // or UNSETTLED meaning that   //
const int PULSE_UNSETTLED = 2;         // it is in the process of     //
// being changed.              //
const int PULSE_ERROR_NONE     = 0;
const int PULSE_ERROR_ONON     = 1;
const int PULSE_ERROR_OFFOFF   = 2;
const int PULSE_ERROR_ONSHORT  = 4;
const int PULSE_ERROR_OFFSHORT = 8;

// Horizontal rotator generates pulses during changing its angular position by 1 degree.
// A pulse is a sequence of ON and OFF states. Current state is kept in the Pulse_Status.
// Whenever the pulse status becomes ON Pulse_On_Horizontal remembers the direction of
// rotation. This is necessary to correctly determine when the azimuth needs to be changed.

volatile int Pulse_Status = PULSE_OFF;
volatile unsigned long Pulse_Last_Change = 0;
volatile int Pulse_On_Horizontal = HORIZONTAL_STP;

// How many bounces and for how long happened during the unsettled stage.

volatile int Pulse_Unsettled_Bounces = 0;
volatile unsigned long Pulse_Unsettled_Time    = 0;

volatile int Pulse_Unsettled_Bounces_Prev = 0;
volatile unsigned long Pulse_Unsettled_Time_Prev = 0;

// ------------------------------------------------------------------ //
// Global rotator mode.                                               //
// In each mode the rotator can be either busy or idle. Switching     //
// between two modes is only allowed if the Pending_Status is IDLE.   // 
// ------------------------------------------------------------------ //

const int MODE_MENU = 1;            // Menu mode
const int MODE_MANUAL = 2;          // Manual mode
const int MODE_AUTO   = 3;          // Auto mode
const int MODE_PREPROG_SET = 4;     // Preprogrammed mode setting up
const int MODE_PREPROG_WRK = 5;     // Preprogrammed mode working

int Mode = 0;
int Target_Mode = 0;

const int PENDING_STATUS_IDLE = 0;
const int PENDING_STATUS_BUSY = 1;

int Pending_Status = PENDING_STATUS_IDLE;

// ------------------------------------------------------------------ //
// Initial Setup                                                      //
// ------------------------------------------------------------------ //

void setup() {

  // Initalize relay pins with HIGH levels

  pinMode(RELAY_AZ1_PIN, INPUT_PULLUP);
  pinMode(RELAY_AZ1_PIN, OUTPUT);
  digitalWrite(RELAY_AZ1_PIN, HIGH);

  pinMode(RELAY_AZ2_PIN, INPUT_PULLUP);
  pinMode(RELAY_AZ2_PIN, OUTPUT);
  digitalWrite(RELAY_AZ2_PIN, HIGH);

  pinMode(RELAY_EL1_PIN, INPUT_PULLUP);
  pinMode(RELAY_EL1_PIN, OUTPUT);
  digitalWrite(RELAY_EL1_PIN, HIGH);

  pinMode(RELAY_EL2_PIN, INPUT_PULLUP);
  pinMode(RELAY_EL2_PIN, OUTPUT);
  digitalWrite(RELAY_EL2_PIN, HIGH);

  // Attach PULL-UP resistors to button pins                            //

  pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MENU_PIN, INPUT_PULLUP);  

  // Read last stored azimuth, elevation and Balance_offset positions

  Horizontal_Position = EEPROMReadInt(EEPROM_AZIMUTH);
  Vertical_Position   = EEPROMReadInt(EEPROM_ELEVATION);
  Balance_Position    = EEPROMReadInt(EEPROM_OFFSET);
  Max_Offs            = EEPROMReadInt(EEPROM_MAXOFFS);
  if (Horizontal_Position < 0 || Vertical_Position < 0) {
    Horizontal_Position = 0;
    Vertical_Position = 0;
    Balance_Position = 0;    
    Max_Offs = 270;
  };

  // Initialize i2c

  Wire.begin();

  // Initalize software to access RTC module

  RTC.begin();

  // Identification screen

  lcd.begin(16, 2);              // initialize the lcd
  lcd.backlight();
  printLogoLCD();

  // Define custom characters on the display

  createLcdChar(1, arrow_up);
  createLcdChar(2, arrow_right);
  createLcdChar(3, arrow_down);
  createLcdChar(4, arrow_left);
  createLcdChar(5, degree);
  createLcdChar(6, selector);
  createLcdChar(7, communic);  

  delay(2000);
  
  EBS_Open();

  // Initialize Menu Items

  MIF_init_all(all_items, sizeof(all_items)/sizeof(currentMenuItem));

  // Start the code in the MENU mode

  Target_Mode = MODE_MENU;
  syncWithRTC();
  selectMenuItem((struct MenuItem_T*) &mi_root, true);
  timePosition = 11;

  // Attach interrupt handler to pin 7 (interrupt 4 on Micro)

  Pulse_Status = PULSE_UNSETTLED;
  Pulse_Last_Change = millis();
  attachInterrupt(digitalPinToInterrupt(PULSE_INT_PIN), pulseDetector, CHANGE);
  pinMode(PULSE_INT_PIN, INPUT_PULLUP);
  
  memset(commBuf, 0, sizeof(commBuf));
  commBufIx = 0;
  Serial1.begin(115200);

}

// ------------------------------------------------------------------ //
// Main loop                                                          //
// ------------------------------------------------------------------ //

void loop() {

  unsigned long ms = millis();
  long w; int t;
  int bn, bt;
  char c;

  if (ms - initMs >= 1000) {
    if (getCurrentTime() == 1) {
      if (datePosition >= 0) {
        lcd.setCursor(datePosition, datePosLine);
        lcd.print(dateStr);
      }
    }
    if (timePosition >= 0) {
      lcd.setCursor(timePosition, timePosLine);
      memset(timeStr, 0, sizeof(timeStr));
      formatCurrentTime(timeFormat, timeStr);
      lcd.print(timeStr);
    }
  };
  
  if (updateStatLine) {
    switch(Mode) {
      case MODE_MANUAL:
      case MODE_AUTO:      
      case MODE_PREPROG_WRK:
        if (strcmp(Exception, "") != 0) {
          sprintf(statStr, "%.3d\x05   %.4s  %.2d\x05", getAzimuth(), Exception, getElevation());
        } else if (Pulse_Unsettled_Bounces_Prev <= 1) {
          sprintf(statStr, "%.3d\x05         %.2d\x05", getAzimuth(), getElevation());
        } else {
          if (Pulse_Unsettled_Bounces_Prev <= 99) {
            bn = Pulse_Unsettled_Bounces_Prev;
          } else {
            bn = 99;
          }
          if (Pulse_Unsettled_Time_Prev <= 99) {
            bt = Pulse_Unsettled_Time_Prev;
          } else {
            bt = 99;
          }
          sprintf(statStr, "%.3d\x05 B:%.2d-%.2d %.2d\x05", getAzimuth(), bn, bt, getElevation());
        }
        lcd.setCursor(0, 1);
        lcd.print(statStr);
        break;
      case MODE_PREPROG_SET:
        if (PreProg_Target_Delta == PREPROG_TARGET_DELTA_UNDEFINED) {
          sprintf(statStr, " *Insuff.Range*");
        } else {
          c = ' ';
          if (PreProg_Target_Delta < 0) {
            c = 'C';
          };
          sprintf(statStr, "%.3d\x05 %cCW %.3d \x03Go", Horizontal_Position, c, abs(PreProg_Target_Delta));
        };
        
        lcd.setCursor(0, 1);
        lcd.print(statStr);        
        break;
      default:
        break;    
    };
    updateStatLine = false;    
  };
  
  if (Mode == MODE_AUTO && timePassed(last_comm_activity, ms) <= 1000UL) {
    if (comm_indicator == ' ') {
      comm_indicator = 0x07;
      lcd.setCursor(5, 0);
      lcd.print(comm_indicator);
    };
  } else {
    if (comm_indicator != ' ') {
      comm_indicator = ' ';
      lcd.setCursor(5, 0);
      lcd.print(comm_indicator);
    };
  };
  
  if ((Mode == MODE_PREPROG_WRK) && ((Target_Mode == MODE_PREPROG_WRK)) && (Horizontal_Target_Offset == HORIZONTAL_TARGET_NOT_SET) && (Horizontal_Request == HORIZONTAL_STP) && (secs != secsForLastCalculation)) {
    if ((currentSatPassDescr != NULL) && (days >= currentSatPassDescr->daysAOS) && (days <= currentSatPassDescr->daysAOS + 1L)) {
      w = (days - currentSatPassDescr->daysAOS) * 86400L + secs;
      if ((w >= currentSatPassDescr->secsAOS) && (w < currentSatPassDescr->secsAOS + currentSatPassDescr->interval)) {
        t = calculateAzimuth(currentSatPassDescr, (int)(w - currentSatPassDescr->secsAOS)) + PreProg_Initial_Offset;
        secsForLastCalculation = secs;
        if (t != Balance_Position) {
          Horizontal_Target_Offset = t;
        };
        t = calculateElevation(currentSatPassDescr, (int)(w - currentSatPassDescr->secsAOS));
        if (Vertical_Position != t) {
          Vertical_Position = t;
          updateStatLine = true;
        };
      };
    };
  };

  checkArrowButtons(ms);
  rotationController();
  
  if (Serial1.available() > 0) {
    while (Serial1.available() > 0) {
      c = Serial1.read();
      if (c == '\n' | c == '\r') {
        incomingCommand(commBuf);
        memset(commBuf, 0, sizeof(commBuf));
        commBufIx = 0;
      } else {
        if (commBufIx <= (sizeof(commBuf) - 2)) {
          commBuf[commBufIx] = c;
          commBufIx ++;
        }
      }
    }
  }
  
  modeSwitchingController();

}

// ------------------------------------------------------------------ //
// Interrupt handler to count pulses from Alpha rotator               //
// ------------------------------------------------------------------ //

void pulseDetector() {

  unsigned long time = millis();

  if (Pulse_Status == PULSE_ON || Pulse_Status == PULSE_OFF) {  // ON or OFF
    Pulse_Status = PULSE_UNSETTLED;
    Pulse_Unsettled_Bounces_Prev = Pulse_Unsettled_Bounces;
    Pulse_Unsettled_Time_Prev = Pulse_Unsettled_Time;
    Pulse_Unsettled_Bounces = 1;
    Pulse_Unsettled_Time = 0;
  } else {                                                     // UNSETTLED
    Pulse_Unsettled_Bounces = Pulse_Unsettled_Bounces + 1;
    Pulse_Unsettled_Time = Pulse_Unsettled_Time + timePassed(Pulse_Last_Change, time);
  }
  Pulse_Last_Change = time;

};

// ------------------------------------------------------------------ //
// Callback functions to get and set values. These methods are called //
// from MenuItem objects.                                             //
// ------------------------------------------------------------------ //

int getAzimuth() {
  return Horizontal_Position;
};

void setAzimuth(int az) {
  Horizontal_Position = az;
};

int getElevation() {
  return Vertical_Position;
};

void setElevation(int el) {
  Vertical_Position = el;
};

void switchToManualMode() {
  Target_Mode = MODE_MANUAL;
};

void switchToAutoMode() {
  Target_Mode = MODE_AUTO;
};

void returnToBalance() {
  Target_Mode = MODE_MANUAL;
  Horizontal_Target_Offset = 0;
};

void saveParameters() {
  EEPROMWriteInt(EEPROM_AZIMUTH, Horizontal_Position);
  EEPROMWriteInt(EEPROM_ELEVATION, Vertical_Position);
  EEPROMWriteInt(EEPROM_OFFSET, Balance_Position);
  EEPROMWriteInt(EEPROM_MAXOFFS, Max_Offs);
};

int getBalance() {
  return Balance_Position;
};

void setBalance(int pos) {
  Balance_Position = pos;
};

int getMaxOffs() {
  return Max_Offs;
};

void setMaxOffs(int maxOffs) {
  Max_Offs = maxOffs;
};

int commGetNumOfSpeeds() {
  return 8;
};

int commGetCurrentSpeed() {
  return Comm_Speed;
};

long commCalcSpeed(int item) {
  long s = 1200;
  
  for (int i = 1; i < item; i++) {
    if (i == 6) {
      s = s + (s >> 1);
    } else {
      s = s + s;
    };
  };
  
  return s;
};

void commFormatSpeed(int item, char* descr) {
  long s = commCalcSpeed(item);
  char left = '\x04'; char right = '\x02';
  if (item == 1) {
    left = ' ';
  };
  if (item == commGetNumOfSpeeds()) {
    right = ' ';
  };
  sprintf(descr, "%c %ld %c  ", left, s, right);
};

void commSpeedConfirm(int item) {
  long s = commCalcSpeed(item);
  Serial1.end();
  Serial1.begin(s);
  Comm_Speed = item;
};

// ------------------------------------------------------------------ //
// This function makes the necessary steps to switch to a new         //
// MenuItem.                                                          //
// ------------------------------------------------------------------ //

void selectMenuItem(struct MenuItem_T* item, boolean initialize) {
  MIF_selectItem(initialize, item);
  lcd.clear();
  lcd.setCursor(0, 0);
  MIF_getName(item, text); lcd.print(text);
  lcd.setCursor(0, 1);
  MIF_getDescription(item, text); lcd.print(text);
  currentMenuItem = item;
}

// ------------------------------------------------------------------ //
// This routine checks the buttons and decides which one is pressed.  //
// ------------------------------------------------------------------ //

void debounceButton(int v, int* a_tstate, unsigned long ms, unsigned long* a_tms, int* a_bstate) {
  if (v == *a_tstate) {                                   // State did not change
    if (timePassed(*a_tms, ms) >= BUTTON_VIBRATION_THRESHOLD) {
      *a_bstate = *a_tstate;
    }
  } else {                                                // State changed
    *a_tstate = v;
    *a_tms = ms;
  }
};

void checkArrowButtons(unsigned long ms) {

  int v_left = digitalRead(BUTTON_LEFT_PIN);
  int v_right = digitalRead(BUTTON_RIGHT_PIN);
  int v_up = digitalRead(BUTTON_UP_PIN);
  int v_down = digitalRead(BUTTON_DOWN_PIN);
  int v_menu = digitalRead(BUTTON_MENU_PIN);
  
  debounceButton(v_menu, &tstate_menu, ms, &tms_menu, &bstate_menu);
  debounceButton(v_left, &tstate_left, ms, &tms_left, &bstate_left);
  debounceButton(v_right, &tstate_right, ms, &tms_right, &bstate_right);
  debounceButton(v_down, &tstate_down, ms, &tms_down, &bstate_down);
  debounceButton(v_up, &tstate_up, ms, &tms_up, &bstate_up);  

  if (bstate_left + bstate_right + bstate_up + bstate_down + bstate_menu == 4) {
    int key = KEY_NONE;
    if (bstate_left == 0) {
      key = KEY_LEFT;
    } else if (bstate_right == 0) {
      key = KEY_RIGHT;
    } else if (bstate_up == 0) {
      key = KEY_UP;
    } else if (bstate_down == 0) {
      key = KEY_DOWN;
    } else if (bstate_menu == 0) {
      key = KEY_MENU;
    } 
    if (key != KEY_NONE) {
      if (pressed_button != key) {
        if (pressed_button != KEY_NONE) {
          key_goes_up(pressed_button);
        }
        key_goes_down(key);
        pressed_button = key;
        held_long_called = ms;
      } else {
        if (timePassed(held_long_called, ms) >= BUTTON_HELD_LONG_CALL_INTERVAL) {
          key_held_long(key);
          held_long_called = ms;
        }
      }
    }
  } else {
    if (pressed_button != KEY_NONE) {
      key_goes_up(pressed_button);
      pressed_button = KEY_NONE;
    }
  }
}

// ------------------------------------------------------------------ //
// Horizontal and Vertical rotation controller                        //
// ------------------------------------------------------------------ //

void setException(char* exc) {
  Exception = exc;
  updateStatLine = true;
};

void requestHorizontalCW() {
  if (Balance_Position < Max_Offs) {
    Horizontal_Request = HORIZONTAL_CW;
    setException("");
  } else {
    setException(EXCEPTION_RIGHT_BOUNDARY);
  };
};

void requestHorizontalCCW() {
  if (Balance_Position > -Max_Offs) {
    Horizontal_Request = HORIZONTAL_CCW;
    setException("");
  } else {
    setException(EXCEPTION_LEFT_BOUNDARY);
  };
};

void rotationController() {

  unsigned long time = millis();
  int distance = 0;
  boolean HorizontalPositionUpdated = false;

  if (Pulse_Status == PULSE_UNSETTLED) {
    if (timePassed(Pulse_Last_Change, time) > PULSE_TOO_SHORT) {
      if (digitalRead(PULSE_INT_PIN) == LOW) {
        Pulse_Status = PULSE_ON;
        Horizontal_Motion_Time = time;
        Pulse_On_Horizontal = Last_Horizontal_Direction;
      } else {
        Pulse_Status = PULSE_OFF;
        Horizontal_Motion_Time = time;        
        if (Last_Horizontal_Direction == HORIZONTAL_CW && Last_Horizontal_Direction == Pulse_On_Horizontal) {
          Horizontal_Position ++;
          if (Horizontal_Position > 359) {
            Horizontal_Position = 0;
          };
          Balance_Position ++;
          HorizontalPositionUpdated = true;
        } else if (Last_Horizontal_Direction == HORIZONTAL_CCW && Last_Horizontal_Direction == Pulse_On_Horizontal) {
          Horizontal_Position --;
          if (Horizontal_Position < 0) {
            Horizontal_Position = 359;
          };
          Balance_Position --;
          HorizontalPositionUpdated = true;            
        };
        if (HorizontalPositionUpdated) {
          
          updateStatLine = true;
          
          // If we reached our target, stop the rotor
          
          if (Horizontal_Target_Offset != HORIZONTAL_TARGET_NOT_SET) {
            if ((Horizontal_Target_Offset == HORIZONTAL_TARGET_AZIMUTH && Horizontal_Target_Azimuth == Horizontal_Position) || (Horizontal_Target_Offset == Balance_Position)) {
              Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;
              Horizontal_Request = HORIZONTAL_STP;
            };
          };          
          
          // Make sure we don't go beyond the "mechanical" boundaries
          
          if (Horizontal_Request != HORIZONTAL_STP && (Balance_Position >= Max_Offs || Balance_Position <= -Max_Offs)) {
            Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;
            Horizontal_Request = HORIZONTAL_STP;            
            if (Balance_Position >= Max_Offs) {
              setException(EXCEPTION_RIGHT_BOUNDARY);
            } else {
              setException(EXCEPTION_LEFT_BOUNDARY);
            };
          };
      
        };
      }
    }
  }
  
  if ((Horizontal_Direction != HORIZONTAL_STP) && (timePassed(Horizontal_Motion_Time, time) >= JAMMED_DETECTION_TIME)) {
    Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;            
    Horizontal_Request = HORIZONTAL_IMM;        
    setException(EXCEPTION_JAMMED);
  };  

  if (Horizontal_Request != Horizontal_Direction) {
    switch (Horizontal_Request) {
      case HORIZONTAL_STP:
      case HORIZONTAL_IMM:
        switch (Horizontal_Direction) {
          case HORIZONTAL_CCW:
            if (Pulse_Status == PULSE_OFF || Horizontal_Request == HORIZONTAL_IMM) {
              digitalWrite(RELAY_AZ2_PIN, HIGH);
              Horizontal_Direction = HORIZONTAL_STP;
              Pending_Status = PENDING_STATUS_IDLE;
            }
            break;
          case HORIZONTAL_CW:
            if (Pulse_Status == PULSE_OFF  || Horizontal_Request == HORIZONTAL_IMM) {
              digitalWrite(RELAY_AZ1_PIN, HIGH);
              Horizontal_Direction = HORIZONTAL_STP;
              Pending_Status = PENDING_STATUS_IDLE;
            }
            break;
          default:
            break;
        };
        if (Horizontal_Request == HORIZONTAL_IMM) {
          Horizontal_Request == HORIZONTAL_STP;
        };
        break;
      case HORIZONTAL_CW:
        switch (Horizontal_Direction) {
          case HORIZONTAL_STP:
            digitalWrite(RELAY_AZ1_PIN, LOW);
            Horizontal_Direction = HORIZONTAL_CW;
            Last_Horizontal_Direction = Horizontal_Direction;
            Horizontal_Motion_Time = time;
            Pending_Status = PENDING_STATUS_BUSY;
            break;
          case HORIZONTAL_CCW:
            if (Pulse_Status == PULSE_OFF) {
              digitalWrite(RELAY_AZ2_PIN, HIGH);
              Horizontal_Direction = HORIZONTAL_STP;
              Pending_Status = PENDING_STATUS_IDLE;
            }
            break;
          default:
            break;
        };
        break;
      case HORIZONTAL_CCW:
        switch (Horizontal_Direction) {
          case HORIZONTAL_STP:
            digitalWrite(RELAY_AZ2_PIN, LOW);
            Horizontal_Direction = HORIZONTAL_CCW;
            Last_Horizontal_Direction = Horizontal_Direction;
            Horizontal_Motion_Time = time;
            Pending_Status = PENDING_STATUS_BUSY;
            break;
          case HORIZONTAL_CW:
            if (Pulse_Status == PULSE_OFF) {
              digitalWrite(RELAY_AZ1_PIN, HIGH);
              Horizontal_Direction = HORIZONTAL_STP;
              Pending_Status = PENDING_STATUS_IDLE;
            }
            break;
          default:
            break;
        };
        break;
      default:
        break;
    }
  };
   
  if (Horizontal_Target_Offset != HORIZONTAL_TARGET_NOT_SET && Horizontal_Request == HORIZONTAL_STP && Horizontal_Direction == HORIZONTAL_STP) {
    if (Horizontal_Target_Offset == HORIZONTAL_TARGET_AZIMUTH) {   // Azimuth target
      if (Horizontal_Target_Azimuth > Horizontal_Position) {
        distance = Horizontal_Target_Azimuth - Horizontal_Position; // CW
        if (distance > 180) {
          distance = -(360 - distance);                             // CCW
        };
      } else if (Horizontal_Target_Azimuth < Horizontal_Position) {
        distance = Horizontal_Position - Horizontal_Target_Azimuth; // CCW
        if (distance > 180) {
          distance = 360 - distance;                                // CW
        } else {
          distance = - distance;
        };
      } else {
        Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;      // Target reached
      };
    } else {                                                       // Offset target
      distance = Horizontal_Target_Offset - Balance_Position;
    };
      
    if (distance > 0) {                                            // CW rotation
      if (Balance_Position + distance >= Max_Offs) {
        Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;      // Rotation cannot be initiated
        setException(EXCEPTION_RIGHT_BOUNDARY);                    // Set the exception
      } else {
        requestHorizontalCW();
      };
    } else if (distance < 0) {                                                       // CCW rotation
      if (Balance_Position + distance <= -Max_Offs) {
        Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;      // Rotation cannot be initiated
        setException(EXCEPTION_LEFT_BOUNDARY);                     // Set the exception
      } else {
        requestHorizontalCCW();
      };  
    } else {
      Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;       // We are already at the target
    };
  };

}

// ------------------------------------------------------------------ //
// Functions called when an arrow key is pressed, or released.        //
// When an arrow key is held pressed for 'long' time a special        //
// function (key_held_long) is called periodically.                   //
// ------------------------------------------------------------------ //

void key_goes_down(int key) {
  if (Mode == MODE_MENU) {
    Pending_Status = PENDING_STATUS_BUSY;
    if (currentMenuItem != NULL) {
      if (key == KEY_LEFT || key == KEY_RIGHT) {
        if (key == KEY_LEFT) {
          MIF_signalLeft(currentMenuItem, 1);
        } else {
          MIF_signalRight(currentMenuItem, 1);
        };
        lcd.setCursor(0, 1);
        MIF_getDescription(currentMenuItem, text);
        lcd.print(text);
      } else if (key == KEY_DOWN) {
        MIF_signalDown(currentMenuItem);
      } else if (key == KEY_UP) {
        MIF_signalUp(currentMenuItem);
      }
    }
  } else if (Mode == MODE_MANUAL) {
    if (key == KEY_LEFT) {
      requestHorizontalCCW();
    } else if (key == KEY_RIGHT) {
      requestHorizontalCW();
    }  else if (key == KEY_UP) {
//      Vertical_Request = VERTICAL_UP;
    } else if (key == KEY_DOWN) {
//      Vertical_Request = VERTICAL_DOWN;
    } else if (key == KEY_MENU) {
      Target_Mode = MODE_MENU;
      Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;
      Horizontal_Request = HORIZONTAL_STP;
    }
  } else if (Mode == MODE_PREPROG_SET) {
    if (key == KEY_MENU) {
      Target_Mode = MODE_MENU;
      Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;
      Horizontal_Request = HORIZONTAL_STP;      
    } else if (key == KEY_DOWN) {
      if ((PreProg_Target_Delta != PREPROG_TARGET_DELTA_UNDEFINED) && (currentSatPassDescr != NULL)) {
        Target_Mode = MODE_PREPROG_WRK;
        Horizontal_Target_Offset = Balance_Position + PreProg_Target_Delta;
        PreProg_Initial_Offset = Horizontal_Target_Offset - calculateAzimuth(currentSatPassDescr, 0);
      };
    };
  } else if (Mode == MODE_PREPROG_WRK) {
    if (key == KEY_MENU) {
      Target_Mode = MODE_MENU;
      Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;
      Horizontal_Request = HORIZONTAL_STP;      
    };
  } else if (Mode == MODE_AUTO) {
    if (key == KEY_MENU) {
      Target_Mode = MODE_MENU;
      Horizontal_Target_Offset = HORIZONTAL_TARGET_NOT_SET;
      Horizontal_Request = HORIZONTAL_STP;
    };    
  };
}

void key_goes_up(int key) {
  if (Mode == MODE_MANUAL) {
    if (key == KEY_LEFT) {
      Horizontal_Request = HORIZONTAL_STP;
    } else if (key == KEY_RIGHT) {
      Horizontal_Request = HORIZONTAL_STP;
    } else if (key == KEY_UP) {
      Vertical_Request = VERTICAL_STP;
    } else if (key == KEY_DOWN) {
      Vertical_Request = VERTICAL_STP;
    }
  } else if (Mode == MODE_MENU) {
    Pending_Status = PENDING_STATUS_IDLE;     
  }
}

void key_held_long(int key) {
  if (Mode == MODE_MENU) {
    if (currentMenuItem != NULL) {
      if (key == KEY_LEFT || key == KEY_RIGHT) {
        if (key == KEY_LEFT) {
          MIF_signalLeft(currentMenuItem, 10);
        } else {
          MIF_signalRight(currentMenuItem, 10);
        };
        lcd.setCursor(0, 1);
        MIF_getDescription(currentMenuItem, text);
        lcd.print(text);        
      }
    }
  }
}

// ------------------------------------------------------------------ //
// Mode switching controller.                                         //
// ------------------------------------------------------------------ //

void modeSwitchingController() {
  int begins, passDelta, w1, w2, w;
  
  if (Target_Mode != Mode && Pending_Status == PENDING_STATUS_IDLE) {
    switch(Target_Mode) {
      case MODE_MANUAL:
        lcd.clear();
        timePosition = 8;
        timeFormat = TF_HHMMSS;
        datePosition = 0;
        lcd.setCursor(0, 0);
        lcd.print(dateStr);
        updateStatLine = true;
        Exception_Row = 1;        
        Exception_Col = 6;
        break;
      case MODE_AUTO:
        lcd.clear();
        timePosition = 8;
        timeFormat = TF_HHMMSS;
        datePosition = -1;                
        lcd.setCursor(0, 0);
        lcd.print("AUTO");
        updateStatLine = true;
        Exception_Row = 1;        
        Exception_Col = 6;
        break;        
      case MODE_MENU:
        lcd.clear();
        lcd.setCursor(0, 0);
        if (currentMenuItem == NULL) {
          currentMenuItem = (MenuItem_T*) &mi_root;
          selectMenuItem(currentMenuItem, true);
        };
        MIF_getName(currentMenuItem, text); lcd.print(text);
        lcd.setCursor(0, 1);
        MIF_getDescription(currentMenuItem, text); lcd.print(text);
        datePosition = -1;        
        timePosition = 11;
        timeFormat = TF_HHMM;
        Exception_Row = -1;
        break;
      case MODE_PREPROG_SET:
        if (currentSatPassDescr != NULL) {
          lcd.clear();
          timePosition = 9;
          timeFormat = TF_DELTA;
          datePosition = -1;
          lcd.setCursor(0, 0);
          lcd.print(currentSatPassDescr->satelliteName);
          updateStatLine = true;
          Exception_Row = 1;        
          Exception_Col = 6;
          
          begins    = calculateAzimuth(currentSatPassDescr, 0);
          passDelta = calculateAzimuth(currentSatPassDescr, currentSatPassDescr->interval - 1) - begins;
          
          PreProg_Target_Delta = PREPROG_TARGET_DELTA_UNDEFINED;
          
          // Say, we move clock-wise (CW) ...
          
          if (begins >= Horizontal_Position) {
            w1 = begins - Horizontal_Position + Balance_Position;
          } else {
            w1 = 360 - Horizontal_Position + begins + Balance_Position;
          };
          w2 = w1 + passDelta;          
          
          if (w1 >= -Max_Offs && w1 <= Max_Offs && w2 >= -Max_Offs && w2 <= Max_Offs) {
            PreProg_Target_Delta = w1 - Balance_Position;
          };
          
          // Try counter-clock-wise (CCW) ...
          
          if (begins > Horizontal_Position) {
            w1 = Balance_Position - (360 - begins + Horizontal_Position);
          } else {
            w1 = Balance_Position - (Horizontal_Position - begins);
          };
          w2 = w1 + passDelta;
     
          if (w1 >= -Max_Offs && w1 <= Max_Offs && w2 >= -Max_Offs && w2 <= Max_Offs) {
            if (PreProg_Target_Delta == PREPROG_TARGET_DELTA_UNDEFINED) {
              PreProg_Target_Delta = w1 - Balance_Position;
            } else {
              if (abs(PreProg_Target_Delta) > abs(w1 - Balance_Position)) {
                PreProg_Target_Delta = w1 - Balance_Position;
              };
            };
          };
          
          secsForLastCalculation = -1L;
          
        } else {
          Target_Mode = Mode;                             // Stay in the current mode, do NOT switch
        };
        break;
      case MODE_PREPROG_WRK:        
        updateStatLine = true;
        break;
      default:
        break;
    };
    Mode = Target_Mode;
  }
}

// ------------------------------------------------------------------ //
// This routine reads the current date and time from the DS3231-based //
// RTC and remembers it along with the relative time in ms returned by//
// the millis() function. When necessary, the new millis() value is   //
// obtained and the date and time is recalculated.                    //
// -------------------------------------------------------------------//

void syncWithRTC() {

  currentDateTime.sync();

  initMs = millis();
  syncTime = initMs;

  if (currentDateTime.year() > 2000) {
    days = (currentDateTime.year() - 2000) * 365L + (currentDateTime.year() - 2001) / 4L + 1;
  } else {
    days = 0L;
  }
  if (currentDateTime.year() % 4 == 0) {
    days = days + (int)pgm_read_word_near(monthsLeap + (currentDateTime.month() - 1));
  } else {
    days = days + (int)pgm_read_word_near(monthsNorm + (currentDateTime.month() - 1));
  }
  days = days + currentDateTime.date();

  secs = currentDateTime.hour() * 3600L + currentDateTime.minute() * 60L + currentDateTime.second();

  formatDate(days);
}

// ------------------------------------------------------------------ //
// Format date as MMM DD,YYYY                                         //
// ------------------------------------------------------------------ //

void getYMD(int ymd[], long days) {
  int yy = 0, mm = 0, dd = 0, leap = 0;
  int y4 = (days - 1) / 1461L;
  int dy4 = (days - 1) % 1461L + 1;
  int dy  = 0;
  if (dy4 <= 366) {
    yy = y4 * 4;
    dy = dy4;
    leap = 1;
  } else if (dy4 <= 731) {
    yy = y4 * 4 + 1;
    dy = dy4 - 366;
  } else if (dy4 <= 1096) {
    yy = y4 * 4 + 2;
    dy = dy4 - 731;
  } else {
    yy = y4 * 4 + 3;
    dy = dy4 - 1096;
  }
  if (leap == 1) {                                  // Leap year
    for (int m = 11; m >= 0; m--) {
      if (dy > (int)pgm_read_word_near(monthsLeap + m)) {
        mm = m + 1;
        break;
      }
    }
    dd = dy - (int)pgm_read_word_near(monthsLeap + (mm - 1));
  } else {                                          // Non-leap year
    for (int m = 11; m >= 0; m--) {
      if (dy > (int)pgm_read_word_near(monthsNorm + m)) {
        mm = m + 1;
        break;
      }
    }
    dd = dy - (int)pgm_read_word_near(monthsNorm + (mm - 1));
  }
  ymd[0] = yy;
  ymd[1] = mm;
  ymd[2] = dd;
}

void formatDate(long days) {

  int ymd[3];

  getYMD(ymd, days);

  sprintf(dateStr, "%c%c%c %.2d", pgm_read_byte_near(monthsString+(ymd[1]-1)*3), pgm_read_byte_near(monthsString+(ymd[1]-1)*3+1), pgm_read_byte_near(monthsString+(ymd[1]-1)*3+2), ymd[2]);
  sprintf(yearStr, "%.4d", ymd[0]);
}

// ------------------------------------------------------------------ //
// Setting Date and time using menu                                   //
// ------------------------------------------------------------------ //

int getYear() {
  int ymd[3];
  getYMD(ymd, days);
  return ymd[0];
}

void setYear(int y) {
  currentDateTime.sync();
  currentDateTime.setYear(y);
  RTC.adjust(currentDateTime);
  syncWithRTC();
}

int getMonth() {
  int ymd[3];
  getYMD(ymd, days);
  return ymd[1];
}

void setMonth(int m) {
  currentDateTime.sync();
  currentDateTime.setMonth(m);
  RTC.adjust(currentDateTime);
  syncWithRTC();  
}

int getDay() {
  int ymd[3];
  getYMD(ymd, days);
  return ymd[2];
}

void setDay(int d) {
  currentDateTime.sync();
  currentDateTime.setDay(d);
  RTC.adjust(currentDateTime);
  syncWithRTC();    
}

int getDays() {
  return (int)days;
}

void setDays(int dd) {
}

int getHours() {
  return (int) (secs / 3600);
}

void setHours(int hh) {
  currentDateTime.sync();
  currentDateTime.setHours(hh);
  RTC.adjust(currentDateTime);
  syncWithRTC();      
}

int getMinutes() {
  return (int) (secs / 60) - (secs / 3600) * 60;
}

void setMinutes(int mm) {
  currentDateTime.sync();
  currentDateTime.setMinutes(mm);
  RTC.adjust(currentDateTime);
  syncWithRTC();        
}

void zeroSeconds() {
  currentDateTime.sync();
  currentDateTime.setSeconds(0);
  RTC.adjust(currentDateTime);
  syncWithRTC();          
}

// ------------------------------------------------------------------ //
// This routine recalculates Time and possible Date strings. When the //
// return value is not zero, it indicates that the Date string was    //
// also updated.                                                      //
// ------------------------------------------------------------------ //

int getCurrentTime() {
  unsigned long w = millis();                          // Obtain current time
  unsigned long delta = timePassed(initMs, w);         // Time since the last date/time recalculation

  int r = 0;

  if (timePassed(syncTime, w) >= time_sync_interval) { // Is it time to Sync again ?
    syncWithRTC();                                     // - Yes, sync with RTC
    w = initMs;                                        // Adjust current time and interval to
    delta = 0UL;                                       // reflect the fact that we just synced
    r = 1;                                             // dateStr field potentially has a new value
  }

  long s = (delta + 500L) / 1000L + secs;              // Number of seconds since the last sync
  long d = days + s / 86400L;                          // Express this interval as a number of
  s = s % 86400L;                                      // days and seconds in the day

  initMs = w;                                         
  secs = s;

  if (d > days) {
    days = d;
    formatDate(days);
    r = 1;
  }

  return r;
}

// ------------------------------------------------------------------ //
// Calculate time interval between two UL values in ms: t1 (earlier)  //
// and t2 (later)                                                     //
// ------------------------------------------------------------------ //

unsigned long timePassed(unsigned long t1, unsigned long t2) {
  if (t2 >= t1) {
    return t2 - t1;
  } else {
    return max_ul - t1 + 1 + t2;
  }
}

// ------------------------------------------------------------------ //
// Format current time according to the requested format.             //
// For TF_HHMMSS and TF_HHMM print regular time format. For TF_DELTA  //
// calculate difference between current time and the moment when the  //
// current satellite pass starts. The difference may be negative or   //
// positive.                                                          //
// ------------------------------------------------------------------ //

void formatCurrentTime(int format, char* text) {
  uint8_t dd, hh, mm, ss; long w; char c;
  
  if (format == TF_DELTA && currentSatPassDescr != NULL) {
    if (currentSatPassDescr->daysAOS > days) {
      c = '-';
      if (currentSatPassDescr->daysAOS - days >= 98) {
        dd = 99; w = 0;
      } else {
        dd = currentSatPassDescr->daysAOS - days;
        
        if (currentSatPassDescr->secsAOS >= secs) {
          w = currentSatPassDescr->secsAOS - secs;
        } else {
          w = currentSatPassDescr->secsAOS + 86400L - secs;
          dd = dd - 1;
        };
      };
    } else if (currentSatPassDescr->daysAOS < days) {
      c = '+';
      if (currentSatPassDescr->daysAOS - days <= -98) {
        dd = 99; w = 0;
      } else {
        dd = days - currentSatPassDescr->daysAOS;
        
        if (currentSatPassDescr->secsAOS <= secs) {
          w = secs - currentSatPassDescr->secsAOS;
        } else {
          w = secs + 86400L - currentSatPassDescr->secsAOS;
          dd = dd - 1;
        };
      };
    } else {
      dd = 0;
      
      if (currentSatPassDescr->secsAOS > secs) {
        c = '-';
        w = currentSatPassDescr->secsAOS - secs;
      } else if (currentSatPassDescr->secsAOS < secs) {
        c = '+';
        w = secs - currentSatPassDescr->secsAOS;
      } else {
        c = '+';
        w = 0;
      };
    };
    
    if (c == '+' && w > currentSatPassDescr->interval) {
      c = ' ';
    };
   
    hh = (uint8_t)(w / 3600L);
    w = w % 3600L;
    mm = (uint8_t)(w / 60L);
    ss = (uint8_t)(w % 60L);
    
    if (dd > 0) {
      sprintf(text, "%c%.2dd%.2dh", c, dd, hh);
    } else if (hh > 0) {
      sprintf(text, "%c%.2dh%.2dm", c, hh, mm);      
    } else {
      sprintf(text, "%c%.2d:%.2d ", c, mm, ss);            
    };

  } else if (format == TF_HHMM) {
    w = secs;
    hh = (uint8_t)(w / 3600L);
    w = w % 3600L;
    mm = (uint8_t)(w / 60L);
    sprintf(text, "%.2d:%.2d", hh, mm);
  } else {
    w = secs;
    hh = (uint8_t)(w / 3600L);
    w = w % 3600L;
    mm = (uint8_t)(w / 60L);    
    ss = (uint8_t)(w % 60L);
    sprintf(text, "%.2d:%.2d:%.2d", hh, mm, ss);
  };
  
}

// ------------------------------------------------------------------ //
// Write int to EEPROM starting with the specified address.           //
// ------------------------------------------------------------------ //

void EEPROMWriteInt(int p_address, int p_value) {

  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);

}

// ------------------------------------------------------------------ //
// Read int from EEPROM starting with the specified address.          //
// ------------------------------------------------------------------ //

int EEPROMReadInt(int p_address) {
 
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

// ------------------------------------------------------------------ //
// LCD-related functions                                              //
// ------------------------------------------------------------------ //

void createLcdChar(int n, const byte* codes) {
  byte work[8];
  for (int i = 0; i < 8; i++) {
    work[i] = pgm_read_byte_near(codes + i);
  };
  lcd.createChar(n, work);
}

void printLogoLCD() {
  int i;
  for (i = 0; i < 15; i++) {
    lcd.setCursor(i, 0); 
    lcd.print((char)pgm_read_byte_near(splashText1 + i));
  };
  for (i = 0; i < 12; i++) {
    lcd.setCursor(i, 1); 
    lcd.print((char)pgm_read_byte_near(splashText2 + i));
  };  
  for (i = 0; i < 4; i++) {
    lcd.setCursor(12+i, 1); 
    lcd.print((char)pgm_read_byte_near(splashText3 + i));
  }    
}

// ------------------------------------------------------------------ //
// Satellite Pass description-related functions                       //
// ------------------------------------------------------------------ //

int getNumSatPassItems() {
  
  int n = 0;                            // Assume the description list is empty
  ebs_block_map = 0L;
  boolean have_data = true;
  unsigned long marker = 1UL;
  
  if (ebs_error_code == 0) {
    EBS_ReadFirst(&satPass01);
    while(have_data) {
      if (satPass01.half_point_y > 0) {      // If an entry contains valid data
        if ((unsigned long)days * 86400UL + (unsigned long)secs < (unsigned long)(satPass01.daysAOS) * 86400UL + (unsigned long)(satPass01.secsAOS) + (unsigned long)(satPass01.interval)) {
          n ++;
          ebs_block_map |= marker;
        }
      }
      marker = marker << 1;
      have_data = EBS_ReadNext(&satPass01);
    };
  } else {
    n = -1;
  };
  
  return n;
};

int getCurrentPassItem() {
  return 1;
};

uint8_t item2block(int item) {
  unsigned long marker = 1UL;  
  uint8_t wn = (uint8_t)item;
  uint8_t bn = 0;
  while(wn > 0) {
    if (bn >= EBS_N_BLOCKS) {           // Safety check.
      break;
    };
    if (ebs_block_map & marker ) {
      wn --;
    };
    bn ++;
    marker = marker << 1;
  };
  bn--;
  return bn;
};

void formatSatPassItemDescr(int n, char* descr) {
  int  hh, mm;
  long w;
  
  if (EBS_ReadRandom(&satPass01, item2block(n)) && satPass01.half_point_y > 0) {  // half_point_y > 0 means an entry has valid data
    w = satPass01.secsAOS / 60L;                         
    hh  = w / 60L;
    mm = w % 60L;

    sprintf(descr, "%-6s %.2d:%.2d %.2d\x05", satPass01.satelliteName, hh, mm, calculateElevation(&satPass01,satPass01.half_point_z));
        
    w = satPass01.daysAOS - days;
   
    if (w >= 10) {
      strncpy(&descr[7], ">10ds", 5);
    } else if (w > 0) {
      descr[9]  = descr[7];
      descr[10] = descr[8];
      descr[7] = (byte)w + 48;
      descr[8] = 'd';
      descr[11] = 'h';
    };
    
  } else {
    strcpy(descr, "NO DATA");
  };
};

void passItemConfirm(int n) {
  if (EBS_ReadRandom(&satPass01, item2block(n)) && satPass01.half_point_y > 0) {  // half_point_y > 0 means an entry has valid data
    currentSatPassDescr = &satPass01;
    Target_Mode = MODE_PREPROG_SET;
  };
};

void MIF_Hidden_Item_Process() {
  lcd.setCursor(5, 0);
  lcd.print("Init");
  EBS_Initialize();
  EBS_Open();
  delay(1000);
  lcd.setCursor(5, 0);  
  lcd.print("    ");
};

// ------------------------------------------------------------------ //
// Functions to work with menu items.                                 //
// ------------------------------------------------------------------ //

void MIF_addChild(struct MenuItemGroup_T* parent, struct MenuItem_T* child) {
  if (parent->firstChild == NULL) {
    parent->firstChild = child;
  } else {
    struct MenuItem_T* s = parent->firstChild;
    while (s->youngerBrother != NULL) {
      s = s->youngerBrother;
    }
    s->youngerBrother = child;
  }
}

void MIF_init_all(struct MenuItem_T** items, int n) {
  MIF_ItemTypes mi;
  
  for (int i = 0; i < n; i++) {
    mi.item= items[i];
    
    mi.item->youngerBrother = NULL;
    if (mi.item->parent != NULL) {
      MIF_addChild((MenuItemGroup_T *)mi.item->parent, mi.item);
    }

    if (mi.item->type == MIT_Group) {
      mi.g->firstChild = NULL;      
    };
  };
};

void MIF_getName(struct MenuItem_T* item, char result[]) {
  MIF_ItemTypes mi;
  mi.item= item;   
  int n;

  switch (item->type) {
    case MIT_Group:
      strcpy(result, mi.g->base.text);
      break;
    case MIT_Integer:
      mi.i->value = mi.i->getValueCallback();
      memset(result, 0, DISPLAY_LINE_SIZE+1);
      sprintf(result, "%s:%d%s", mi.i->base.text, mi.i->value, mi.i->decorator);
      break;
    case MIT_Selection:
      strcpy(result, mi.s->base.text);      
      break;
    case MIT_List:
      if (mi.l->base.text[0] == '.') {
        strcpy(result, &(mi.l->base.text[1]));
      } else {
        n = mi.l->initNofItemsCallback();
        if (n >= 0) {
          sprintf(result, "%s%.2d", mi.l->base.text, n);
        } else {
          sprintf(result, "%s--", mi.l->base.text);
        };
      };
      break;
    default:
      strcpy(result, "?");
      break;
  }
}

void MIF_selectItem(boolean initialize, struct MenuItem_T* item) {
  MIF_ItemTypes mi;
  mi.item= item;       

  int ix = 0; int pos = 0;
  struct MenuItem_T* c = NULL;

  switch (item->type) {
    case MIT_Group:
      c = mi.g->firstChild;      
      while (c != NULL) {
//        mi.g->children[ix] = c;
        c = c->youngerBrother;
        ix ++;
      }

      mi.g->nofChildren = ix;

      if (initialize) {
        mi.g->currentChildIx = 0;
      } else {
        if (mi.g->currentChildIx >= mi.g->nofChildren) {
          mi.g->currentChildIx = 0;
        }
      };
      break;
    case MIT_Integer:
      mi.i->tempValue = mi.i->value;
      break;
    case MIT_Selection:
      break;
    case MIT_List:
      if (initialize) {
        mi.l->nof_items = mi.l->initNofItemsCallback();
        mi.l->cur_item  = mi.l->initCurrentItemNumCallback();
      };
      mi.l->temp_item = mi.l->cur_item;      
      break;
    default:
      break;
  }
}

void MIF_signalLeft(struct MenuItem_T* item, int n) {
  MIF_ItemTypes mi;
  mi.item= item;     

  MIF_Hidden_Menu_Count = 0;

  switch (item->type) {
    case MIT_Group:
      if (mi.g->currentChildIx > 0) {
        mi.g->currentChildIx --;
      }
      break;
    case MIT_Integer:
      mi.i->tempValue = mi.i->tempValue - n;
      if (mi.i->tempValue < mi.i->minValue) {
        mi.i->tempValue = mi.i->maxValue;
      }
      break;
    case MIT_Selection:
      break;
    case MIT_List:
      if (mi.l->temp_item >= 2) {
        mi.l->temp_item --;
      };
      break;      
    default:
      break;
  }
}

void MIF_signalRight(struct MenuItem_T* item, int n) {
  MIF_ItemTypes mi;
  mi.item= item;     

  MIF_Hidden_Menu_Count = 0;

  switch (item->type) {
    case MIT_Group:
      if (mi.g->currentChildIx < mi.g->nofChildren - 1) {
        mi.g->currentChildIx ++;
      }
      break;
    case MIT_Integer:
      mi.i->tempValue = mi.i->tempValue + n;
      if (mi.i->tempValue > mi.i->maxValue) {
        mi.i->tempValue = mi.i->minValue;
      }
      break;
    case MIT_Selection:
      break;
    case MIT_List:
      if (mi.l->temp_item < mi.l->nof_items) {
        mi.l->temp_item ++;
      };      
      break;
    default:
      break;
  }
}

void MIF_signalUp(struct MenuItem_T* item) {
  MIF_ItemTypes mi;
  mi.item= item;     

  if (strcmp(item->text,"Set") == 0) {
    MIF_Hidden_Menu_Count ++;
    if (MIF_Hidden_Menu_Count > 9) {
      MIF_Hidden_Item_Process();
      MIF_Hidden_Menu_Count = 0;    
    };
  } else {
    MIF_Hidden_Menu_Count = 0;    
  };

  switch (item->type) {
    case MIT_Group:
      if (mi.g->base.parent != NULL) {
        selectMenuItem(mi.g->base.parent, false);
      };
      break;
    case MIT_Integer:
      if (mi.i->base.parent != NULL) {
        selectMenuItem(mi.i->base.parent, false);
      };
      break;
    case MIT_Selection:
      if (mi.s->base.parent != NULL) {
        selectMenuItem(mi.s->base.parent, false);
      };
      break;
    case MIT_List:
      if (mi.l->base.parent != NULL) {
        selectMenuItem(mi.l->base.parent, false);
      };
      break;      
    default:
      break;
  }
}

void MIF_signalDown(struct MenuItem_T* item) {
  MIF_ItemTypes mi;
  mi.item= item;
  struct MenuItem_T* c = NULL;
  
  MIF_Hidden_Menu_Count = 0;  

  switch (item->type) {
    case MIT_Group:
      if (mi.g->currentChildIx < mi.g->nofChildren) {
        c = mi.g->firstChild;
        if (mi.g->currentChildIx > 0) {
          for (int ix = 0; ix < mi.g->currentChildIx && c != NULL; ix ++) {
            c = c->youngerBrother;
          };
        };
        if (c != NULL) {
          selectMenuItem(c, true);
        };
      };
      break;
    case MIT_Integer:
      mi.i->value = mi.i->tempValue;
      mi.i->setValueCallback(mi.i->value);
      if (mi.i->base.parent != NULL) {
        selectMenuItem(mi.i->base.parent, false);
      };
      break;
    case MIT_Selection:
      mi.s->confirmedCallback();
      if (mi.s->base.parent != NULL) {
        selectMenuItem(mi.s->base.parent, false);
      };      
      break;
    case MIT_List:
      if (mi.l->nof_items > 0) {
        mi.l->cur_item = mi.l->temp_item;
        if (mi.l->base.parent != NULL) {
          selectMenuItem(mi.l->base.parent, false);
        };              
        mi.l->confirmNewItemNumCallback(mi.l->cur_item);
      };
    default:
      break;
  }
}

void MIF_getDescription(struct MenuItem_T* item, char result[]) {
  MIF_ItemTypes mi;
  mi.item= item;        
  
  int nd = 0, pos = 0, ix = 0;
  struct MenuItem_T* c = NULL;

  switch (item->type) {
    case MIT_Group:     
      memset(result, 0, DISPLAY_LINE_SIZE+1);
      c = mi.g->firstChild;

      while (c != NULL) {
        strcat(result, " ");
        MIF_getName(c, items_w_buf);
        strcat(result, items_w_buf);
        mi.g->childPos[ix] = pos;
        c = c->youngerBrother;
        pos = pos + strlen(items_w_buf) + 1;
        ix ++;
      }

      if (mi.g->nofChildren > 0) {
        result[mi.g->childPos[mi.g->currentChildIx]] = SYM_SELECTOR;
      };      
      break;
    case MIT_Integer:  
      memset(items_w_buf, 0, DISPLAY_LINE_SIZE+1);
      sprintf(items_w_buf, "%d", mi.i->maxValue);
      nd = strlen(items_w_buf);
      memset(items_w_buf, 0, DISPLAY_LINE_SIZE+1);
      sprintf(items_w_buf, "%d", mi.i->minValue);
      if (nd < strlen(items_w_buf)) {
        nd = strlen(items_w_buf);
      }
      memset(items_w_buf, 0, DISPLAY_LINE_SIZE+1);
      sprintf(items_w_buf, "\x04%%.%dd\x02 Confirm\x03", nd);
      
      memset(result, 0, DISPLAY_LINE_SIZE+1);
      sprintf(result, items_w_buf, mi.i->tempValue);
      break;
    case MIT_Selection:
      strcpy(result, "Confirm \x03");
      break;
    case MIT_List:
      if (mi.l->nof_items > 0) {
        mi.l->formatItemDescrCallback(mi.l->temp_item, result);
      } else if (mi.l->nof_items == 0) {
        strcpy(result, "EMPTY");
      } else {
        strcpy(result, "DISABLED");
      }
      break;      
    default:
      strcpy(result, "?");
      break;
  }
}

// ------------------------------------------------------------------ //
// Diagnostic functions                                               //
// ------------------------------------------------------------------ //

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
};

// --------------------------------------------------------------------- //
// "Unpack" satellite pass data by performing polynomial calculations    //
// --------------------------------------------------------------------- //

int calculateAzimuth(struct PassDescription_T * passDescr, int offset) {
  return calculateDirection(passDescr, offset, passDescr->half_point_y, 0, (int)passDescr->ncoef_y, true);
};

int calculateElevation(struct PassDescription_T * passDescr, int offset) {
  int el = calculateDirection(passDescr, offset, passDescr->half_point_z, (int)passDescr->ncoef_y, (int)passDescr->ncoef_z, false);
  if (el > 90) {
    el = 90;
  } else if (el < 0) {
    el = 0;
  };
  return el;
};

int calculateDirection(struct PassDescription_T * passDescr, int offset, int half_point, int start, int total, bool azimuth) {
  float ks, km, r = -1; bool directCalculation;
  
  if (offset >= 0 && offset < passDescr->interval) {
    if (half_point + half_point >= passDescr->interval) {                      // Approximation in the lower part
      ks = 0.0;
      km = ((float)(half_point + 1)) / 2.0;
      if (offset <= half_point) {
        directCalculation = true;
      } else {
        directCalculation = false;
      };
    } else {                                                                  // Approximation in the higher part
      ks = (float)half_point;
      km = ((float)(passDescr->interval - half_point)) / 2.0;
      if (offset >= half_point) {      
        directCalculation = true;
      } else {
        directCalculation = false;
      };
    };
    
    if (directCalculation) {
      r = calculatePolynomialByCoef(passDescr->coef, start, total, (float)offset, ks, km);
    } else {
      r = calculatePolynomialByCoef(passDescr->coef, start, total, (float)(half_point + half_point - offset), ks, km);
      if (azimuth) {
        r = calculatePolynomialByCoef(passDescr->coef, start, total, (float)half_point, ks, km) * 2 - r;          
      };
    };

    return floor(r + 0.5);
  };
  return -1;
};

float calculatePolynomialByCoef(float c[], int start, int total, float x0, float ks, float km) {
  float xn = (x0 - ks) / km - 1;
  
  float r = c[start];
  for (int i = start + 1; i < total; i++) {
    r = r * xn + c[i];
  }
  
  return r;  
};

int azimuthNormalize(int a) {
  int r = a;
  while(r >= 360) {
    r = r - 360;
  };
  while(r < 0) {
    r = r + 360;
  };
  return r;
};

// ------------------------------------------------------------------ //
// EEPROM BLOCK STORAGE functions                                     //
// -------------------------------------------------------------------//

uint8_t increaseSeq(uint8_t seq, uint8_t delta) {
  if (seq + delta > EBS_N_BLOCKS+1) {
    return seq + delta - EBS_N_BLOCKS - 1;
  } else {
    return seq + delta;
  };
};

void EBS_Initialize() {
  uint8_t seq = 1;
  int a;
  for (int i = 0; i < EBS_N_BLOCKS; i++) {
    a = EBS_START_ADDRESS + i * EBS_BLOCK_SIZE;
    EEPROM.write(a, EBS_BLOCK_ID);
    EEPROM.write(a + 1, seq);
    seq = seq + 1;
  };
};

void EBS_Open() {
  
  ebs_free_ix  = 0;
  ebs_next_seq = 0;
  ebs_error_code = 0;  
  
  uint8_t disruptions = 0;  
  uint8_t seq, expected_seq;
  uint8_t  ix;
  int a;
  
  ix = 0;
  for (ix = 0; (ebs_error_code == 0) && (ix < EBS_N_BLOCKS); ix++) {
    a = EBS_START_ADDRESS + ix * EBS_BLOCK_SIZE;
    if (EEPROM.read(a) == EBS_BLOCK_ID) {
      seq = EEPROM.read(a + 1);      
      if (ix == 0) {
        if (seq >= 1 && seq <= EBS_N_BLOCKS + 1) {
          expected_seq = increaseSeq(seq, 1);
        } else {
          ebs_error_code = 2;
        };
      } else {
        if (seq == expected_seq) {
          expected_seq = increaseSeq(expected_seq, 1);
        } else {
          if (seq == increaseSeq(expected_seq, 1)) {
            if (disruptions == 0) {
              ebs_free_ix = ix;
              disruptions ++;
              ebs_next_seq = expected_seq;
              expected_seq = increaseSeq(expected_seq, 2);              
            } else {
              ebs_error_code = 4;
            };
          } else {
            ebs_error_code = 3;
          };
        };
      };
    } else {
      ebs_error_code = 1;
    };
  };
  
  if (ebs_error_code == 0) {
    seq = EEPROM.read(EBS_START_ADDRESS + 1);
    if (seq != expected_seq) {
      if (seq == increaseSeq(expected_seq, 1)) {
        if (disruptions == 0) {
          ebs_next_seq = expected_seq;
          ebs_free_ix = 0;
          disruptions ++;
        } else {
          ebs_error_code = 5;
        };
      } else {
        ebs_error_code = 6;
      };
    };    
  };
  
  if (ebs_error_code == 0 && disruptions != 1) {
    ebs_error_code = 7;
  };
  
};

void EBS_ReadPassDescr(struct PassDescription_T * sp, uint8_t ix) {
  byte* sb = (byte *) sp;  
  if (ebs_error_code == 0) {  
    int a = EBS_START_ADDRESS + (ix * EBS_BLOCK_SIZE);
    for (int o = 0; o < sizeof(struct PassDescription_T); o++) {
      (*(sb + o)) = EEPROM.read(a + o + 2);
    };
  };
};

void EBS_ReadFirst(struct PassDescription_T * sp) {
  if (ebs_error_code == 0) {
    ebs_current_ix = ebs_free_ix;
    EBS_ReadPassDescr(sp, ebs_current_ix);
    ebs_current_ix ++;
    if (ebs_current_ix >= EBS_N_BLOCKS) {
      ebs_current_ix = 0;
    };    
  };
};

boolean EBS_ReadNext(struct PassDescription_T * sp) {
  if (ebs_error_code == 0) {  
    if (ebs_current_ix == ebs_free_ix) {
      return false;
    } else {
      EBS_ReadPassDescr(sp, ebs_current_ix);
      ebs_current_ix ++;
      if (ebs_current_ix >= EBS_N_BLOCKS) {
        ebs_current_ix = 0;
      };    
      return true;
    };
  } else {
    return false;
  };
};

boolean EBS_ReadRandom(struct PassDescription_T * sp, uint8_t ix) {
  if (ebs_error_code == 0) {  
    if (ix >= 1 && ix <= EBS_N_BLOCKS) {
      ix = ix + ebs_free_ix;
      if (ix >= EBS_N_BLOCKS) {
        ix = ix - EBS_N_BLOCKS;
      };
      EBS_ReadPassDescr(sp, ix);
      return true;
    } else {
      return false;
    }; 
  } else {
    return false;
  };
};

void EBS_Write(struct PassDescription_T * sp) {
  byte* sb = (byte *) sp;  
  if (ebs_error_code == 0) {
    int a = EBS_START_ADDRESS + (ebs_free_ix * EBS_BLOCK_SIZE);
    for (int o = 0; o < sizeof(struct PassDescription_T); o++) {
      EEPROM.write(a + o + 2, *(sb + o));
    };
    EEPROM.write(a + 1, ebs_next_seq);
    ebs_free_ix ++;
    if (ebs_free_ix >= EBS_N_BLOCKS) {
      ebs_free_ix = 0;
    };      
    ebs_next_seq = increaseSeq(ebs_next_seq, 1);
  };
};

// ------------------------------------------------------------------ //
// Communication functions                                            //
// -------------------------------------------------------------------//

void incomingCommand(char* cmd) {
  float wf = 0.0; int wi = 0;
  
  const int RSP_NONE = 0;  
  const int RSP_OK = 1;
  const int RSP_ERR = 2;
  int response = RSP_NONE;  
  
  if (strlen(cmd) >= 2 && cmd[0] == '/') {
    Serial1.println(cmd);
    switch(cmd[1]) {
      
      
// /V                   Display software version      
      
      case 'V':
        syncWithRTC();
        formatCurrentTime(TF_HHMMSS, timeStr);
        Serial1.print(VERSION);
        Serial1.print(" ");
        Serial1.print(dateStr);
        Serial1.print(",");    
        Serial1.print(yearStr);
        Serial1.print(" ");        
        Serial1.println(timeStr);        
        response = RSP_OK;
        break;
        
// /Pmm xxxxxxx         Initiate saving new satellite data number mm
        
      case 'P':
        satPass01.half_point_y = 0;    // The entry does not have valid data yet
        if (strlen(cmd) >= 6) {
          memset(satPass01.satelliteName,0,sizeof(satPass01.satelliteName));
          strncpy(satPass01.satelliteName, &cmd[5], 7);
          satPass01.ncoef_y = 0;
          satPass01.ncoef_z = 0;
          response = RSP_OK;
        } else {
          response = RSP_ERR;
        };
        
        break;
        
// /Cxmm +n.nnnnnnne+nn  Set coefficient number mm
        
      case 'C':
        satPass01.half_point_y = 0;    // The entry does not have valid data yet
        response = RSP_ERR;            // Assume the requst is incorrect
        if (strlen(cmd) >= 20 && cmd[5] == ' ') {
          cmd[5] = 0;
          wi = atoi(&cmd[3]);
          if ((cmd[2] == 'Y' || cmd[2] == 'Z') && wi >= 0 && wi <= 17) {
            satPass01.coef[wi] = atof(&cmd[6]);
            if (cmd[2] == 'Y' && wi >= satPass01.ncoef_y) {
              satPass01.ncoef_y = wi + 1;
            };
            if (cmd[2] == 'Z' && wi >= satPass01.ncoef_z) {
              satPass01.ncoef_z = wi + 1;
            };            
            response = RSP_OK;
          };
        };
        break;        
        
// /A sssss ddddd iiiii Set AOS time, date and length of the pass
//                    (seconds of the day, days starting at Jan 01, 2000,
//                     seconds for the length of the pass).
        
      case 'A':
        satPass01.half_point_y = 0;    // The entry does not have valid data yet
        if (strlen(cmd) >= 20 && cmd[2] == ' ' && cmd[8] == ' ' && cmd[14] == ' ') {
          cmd[8] = 0;
          satPass01.secsAOS = atol(&cmd[3]);
          cmd[14] = 0;
          satPass01.daysAOS = atol(&cmd[9]);
          satPass01.interval = atol(&cmd[15]);
          response = RSP_OK;
        } else {
          response = RSP_ERR;
        };
        break;                
        
// /Zx hhhhh ggggg       Set half-point for azimuth and half-point for elevation (in seconds).
//                       x is either 'W' for writing to EPROM or 'C' to leave EPROM buffer unchanged.
//                       This is the last command in the series starting with /P.
        
      case 'Z':
        satPass01.half_point_y = 0;    // The entry does not have valid data yet
        if (strlen(cmd) >= 15 && cmd[3] == ' ' && cmd[9] == ' ' && (cmd[2] == 'W' || cmd[2] == 'C')) {
            cmd[9] = 0;
            satPass01.half_point_z = atoi(&cmd[10]);
            satPass01.half_point_y = atoi(&cmd[4]); // Changing half_point_y to a non-zero value indicates that filling up of the entry is complete
            if (cmd[2] == 'W') {
              EBS_Write(&satPass01);            
            };
            response = RSP_OK;
        } else {
          response = RSP_ERR;
        };
        break;                        
        
// Testing / Debugging ...        
        
      case 'T':
        if (satPass01.half_point_y > 0) {    // half_point_y > 0 means that an entry has valid data
          Serial1.println(satPass01.satelliteName);
          Serial1.println(satPass01.daysAOS);
          Serial1.println(satPass01.secsAOS);
          Serial1.println(satPass01.interval);          
          Serial1.println(satPass01.half_point_y);                    
          for (int i = 0; i < satPass01.ncoef_y; i++) {
            Serial1.println(satPass01.coef[i],7);
          };
          Serial1.println(satPass01.half_point_z);     
          for (int i = satPass01.ncoef_y; i < satPass01.ncoef_z; i++) {
            Serial1.println(satPass01.coef[i],7);
          };          
          Serial1.println(calculateAzimuth(&satPass01,0));
          Serial1.println(calculateAzimuth(&satPass01,satPass01.half_point_y));
          Serial1.println(calculateAzimuth(&satPass01,satPass01.interval-1));
          
          Serial1.println(calculateElevation(&satPass01,0));
          Serial1.println(calculateElevation(&satPass01,satPass01.half_point_z));
          Serial1.println(calculateElevation(&satPass01,satPass01.interval-1));
        } else {
          Serial1.println("Empty");
        }
        Serial1.println(ebs_free_ix);
        Serial1.println(ebs_next_seq);
        Serial1.println(ebs_error_code);
        Serial1.println(ebs_current_ix);        
        Serial1.println(ebs_block_map);
        response = RSP_OK;
        break; 
        
// /Snn ttttt                      - Simulate Azimuth and Elevation calculation results        
//                    nn is the entry number, ttttt is offset in the interval
        
      case 'S':       
        response = RSP_ERR;                             // Assume the request is invalid 
        if (strlen(cmd) >= 10 && cmd[4] == ' ') {
          cmd[4] = 0;
          wi = atoi(&cmd[2]);
          if (EBS_ReadRandom(&satPass01, item2block(wi)) && satPass01.half_point_y > 0) {  // half_point_y > 0 means an entry has valid data
            wi = atoi(&cmd[5]);
            Serial1.println(calculateAzimuth(&satPass01,wi));
            Serial1.println(calculateElevation(&satPass01,wi));
            response = RSP_OK;
          };
        };
        break;      
        
      case 'M':
        Serial1.println(freeRam());
        Serial1.println(sizeof(struct PassDescription_T));
        break;
               
// /D yy mm dd HH MM SS       - set date and time          
               
      case 'D':
        if (strlen(cmd) >= 20 && cmd[2] == ' ' && cmd[5] == ' ' && cmd[8] == ' ' && cmd[11] == ' ' && cmd[14] == ' ' && cmd[17] == ' ') {
          cmd[5] = 0;
          wi = atoi(&cmd[3]);
          currentDateTime.setYear(wi);
          cmd[8] = 0;
          wi = atoi(&cmd[6]);
          currentDateTime.setMonth(wi);
          cmd[11] = 0;
          wi = atoi(&cmd[9]);          
          currentDateTime.setDay(wi);
          cmd[14] = 0;
          wi = atoi(&cmd[12]);
          currentDateTime.setHours(wi);
          cmd[17] = 0;
          wi = atoi(&cmd[15]);
          currentDateTime.setMinutes(wi);
          wi = atoi(&cmd[18]);
          currentDateTime.setSeconds(wi);
          RTC.adjust(currentDateTime);
          syncWithRTC();
          response = RSP_OK;          
        } else {
          response = RSP_ERR;
        };
        break;
               
      default:
        response = RSP_ERR;
        break;
    }
  } else if (strlen(cmd) >= 1 && Mode == MODE_AUTO) {
    response = RSP_NONE;
    char p[16]; int a, b;
    switch(cmd[0]) {
      case 'C':                                          // Process C / C2 commands
      case 'c':
        if (strlen(cmd) == 2 && cmd[1] == '2') {
          sprintf(p, "AZ=%.3d EL=%.3d\r\n", Horizontal_Position, Vertical_Position);
          p[15] = 0;
          Serial1.print(p);
        } else if (strlen(cmd) == 1) { 
          sprintf(p, "AZ=%.3d\r\n", Horizontal_Position);
          p[8] = 0;
          Serial1.print(p);
        } else {
          Serial1.print("?>\r\n");
        };
        break;
      case 'B':                                          // Process B command
      case 'b':        
        sprintf(p, "EL=%.3d\r\n", Vertical_Position);
        p[8] = 0;
        Serial1.print(p);        
        break;
      case 'M':                                          // Process M
      case 'm':      
        if (strlen(cmd) == 4) {
          if (cmd[1] == '0' && cmd[2] == '0' && cmd[3] == '0') {
            a = 0;
          } else {
            a = atoi(&cmd[1]);
            if (a <= 0) {
              a = -1;
              Serial1.print("?>\r\n");
            };
          };            
          if (a >= 0) {
            if (a != Horizontal_Position && Horizontal_Target_Offset == HORIZONTAL_TARGET_NOT_SET) {
              Horizontal_Target_Offset  = HORIZONTAL_TARGET_AZIMUTH;
              Horizontal_Target_Azimuth = a;          
            };
          };
        } else {
          Serial1.print("?>\r\n");
        };
        break;
      case 'W':                                          // Process M
      case 'w':      
        if (strlen(cmd) == 8 && cmd[4] == ' ') {
          cmd[4] = 0;
          if (cmd[1] == '0' && cmd[2] == '0' && cmd[3] == '0') {
            a = 0;
          } else {
            a = atoi(&cmd[1]);
            if (a <= 0) {
              a = -1;
              Serial1.print("?>\r\n");
            };
          };
          if (cmd[5] == '0' && cmd[6] == '0' && cmd[7] == '0') {
            b = 0;
          } else {
            b = atoi(&cmd[5]);
            if (b <= 0) {
              b = -1;
              Serial1.print("?>\r\n");
            };
          };            
          if (a >= 0 && b >= 0) {
            if (a != Horizontal_Position && Horizontal_Target_Offset == HORIZONTAL_TARGET_NOT_SET) {
              Horizontal_Target_Offset  = HORIZONTAL_TARGET_AZIMUTH;
              Horizontal_Target_Azimuth = a;          
            };            
            if (b != Vertical_Position) {            
              Vertical_Position = b;
              updateStatLine = true;
            };
          };
        } else {
          Serial1.print("?>\r\n");
        };
        break;        
      case 'X':                                          // Simulate as NOOP
      case 'x':
        Serial1.print("\r\n");
        break;      
      default:                                           // Everything else is not recognized
        Serial1.print("?>\r\n");
        break;
    };
    last_comm_activity = millis();
  } else {
    Serial1.println(cmd);    
    response = RSP_ERR;
  }
  
  if (response == RSP_OK) {
    Serial1.println("OK");
  } else if (response == RSP_ERR) {
    Serial1.println("ERR");
  }
}

