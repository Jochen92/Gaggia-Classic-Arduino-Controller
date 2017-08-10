/*
TODO: see if "if" statements can be replaced by "if, else if, else" statements
TODO: checkService.
TODO: OnSwitch, wake-up from sleep mode via interrupt pin( 2, 3, 18, 19, 20, 21)
TODO: implement RTC alarm to wake up from sleep mode via interrupt (pin 2, 3, 18, 19, 20, 21)
TODO: machineOnLight, on when machine is wake mode, off when in sleep mode.
TODO: flashing an item in the menu once it's selected to indicate it's selected and possible to change.
TODO: implement sleep-mode and wakeup via interrupt.
TODO: implement brew scale. 
TODO: implement resevoir level sensor with warning on display.
TODO: new menu item: change PID parameters, boiler 1 and 2.
      - create more submenu layers.
      - rethink menu design and select/control method.
TODO: Decide how to set time for RTC.
TODO: rethink displaying methods
TODO: implement Fahrenheit
TODO: comments.
TODO: LCD brightness by PWM
TODO: check for INPUT_PULLUP for rotary encoder without external 5V logic and without external resistors
TODO: change int to byte or short if possible.
*/

/*
  -------------- ERRORS ----------------
  

  -------------- ERRORS ----------------
*/


//load libraries.
#include "max6675.h"
#include "LiquidCrystal.h"
#include "PID_v1.h"
#include "EEPROM.h"
#include "Wire.h"
#include "RTClib.h"

////// Temperature processing //////
//Thermocouple offset in Celsius
double ktcOffsetC1 = 8;
double ktcOffsetC2 = 6;
// Low-pass filter parameter sensor 1
double tempC1;    //input to the lp-filter
double lowPassTempC1; //output from the filter
double cutoff1 = 0.97;  //value to determine the amount of filtering.
// Low-pass filter parameter sensor 2
double tempC2;    //input to the lp-filter
double lowPassTempC2; //ouput from the filter
double cutoff2 = 0.97;  //value to determine the amnout of filtering.
//temperature rounding parameters sensor 1
int intPartC1;    //integer part of the temperature
double deciPartC1;  //decimal part of the temperature
double roundedTempC1; //rounded temperature
//temperature rounding parameters sensor 2
int intPartC2;    //integerpart of the temperature
double deciPartC2;  //decimalpart of the temperature
double roundedTempC2; //rounded temperature


////// PID parameters //////
//PID 1 parameters
double setpointC1;    //the desired temperature.
double setpoint1, input1, output1;  //values used by the PID library
double idleKp1 = 17, idleKi1 = 0.1, idleKd1 = 60; //PID values for idle mode
double midKp1 = 25, midKi1 = 0.17, midKd1 = 50;   //PID values for medium mode.
double aggrKp1 = 100, aggrKi1 = 0, aggrKd1 = 20;  //PID values for aggressive mode.
//PID 2 parameters
double setpointC2;  //desired temperature.
double setpoint2, input2, output2;    //values used by the PID library
double idleKp2 = 17, idleKi2 = 0.09, idleKd2 = 60;  //PID values for idle mode
double midKp2 = 25, midKi2 = 0.15, midKd2 = 50;     //PID values for medium mode
double aggrKp2 = 100, aggrKi2 = 0, aggrKd2 = 20;    //PID values for aggressive mode.
//PID relay window size (artificial PWM)
int windowSize = 500;   //artificial PWM window size to determine the output to the relays.
unsigned long windowStartTime;
//Steam parameters.
bool readyToSteam;       //wether the boiler is up to temperature for steaming.
double setpointCSteam; //Desired steam temperature in Celcius.
bool steamState;        //Wether the system is in steam mode or not.
bool idleSteam1;
bool idleSteam2;
byte steamLogo[8] = {
  0b01100,
  0b10000,
  0b01000,
  0b00100,
  0b11000,
  0b00111,
  0b00010,
  0b00010
};


////// Brewing //////
// Brewing Timer
unsigned long brewBegin;  //A time marker for when the brewing begins
double shotTimer; //How long the shot is lasting
double shotWeight; //How much the shot currently weighs
double finalShotTime; //storage for the final shottime value
double finalShotWeight; //storage for the final shotweight value.
bool holdShotParametersState;  //State to hold shottime printed on the lcd screen after brewing has stopped
int holdShotParametersLength = 10000;  //The time to keep shottime printed on the lcd screen after brewing has stopped.
unsigned long holdShotParametersBegin; //A time marker for when the shot time is being held on the lcd screen.
//Settings for brewing
bool idleBrew1; //Wether boiler 1 is at the right temperature.
bool idleBrew2; //wether boiler 2 is at the right temperature.
bool currentIdleBrew1;
bool currentIdle2;
bool lastIdleBrew1;
bool lastIdle2;
bool readyToBrew1;
bool readyToBrew2;
bool brewState;     //Wether the system is brewing coffee or not.
volatile bool brewMethod;    //Which brew method is used. Automatic or manual.
volatile int piMethod;       //If pre-infusion is used. Off, boiler pressure or pump pressure.
double piTime;         //the pre-infusion length; if used.
volatile double maxShotTime;    //maximum time a shot may take in automatic brewing mode.
volatile double maxShotWeight;  //maximum weight a shot may have in automatic brewing mode.
byte piLogo[8] = {  //Pre-Infusion logo
  0b11100,
  0b10100,
  0b11100,
  0b10000,
  0b10111,
  0b00010,
  0b00010,
  0b00111
};


//buttonstates.
bool currentSelectState = LOW;
bool lastSelectState = LOW;
bool currentBrewSwitchState = LOW;
bool lastBrewSwitchState = LOW;
bool currentSteamSwitchState = LOW;
bool lastSteamSwitchState = LOW;
bool currentOnSwitchState = LOW;
bool lastOnSwitchState = LOW;


//Systems' unit (Metric or Imperial).
bool units;


//fading leds on PWM
int brightness = 0;
int fadeStep = 5;


//WakeUp Alarm parameters. Machine will be ready to use at the wake up time.
int wakeUpHour;
int wakeUpMinute;
volatile int wakeUpTime; //A value from 0 to 95. Divided by 4, this represents the time with a resolution of 15 minutes (written to EEPROM)
volatile bool wakeUp;    //Wether the alarm is set or not.
byte wakeUpLogo[8] = { //alarmbell logo
  0b00100,
  0b01110,
  0b01110,
  0b11111,
  0b10001,
  0b11111,
  0b01110,
  0b00000
};


//Service parameters
volatile bool requestUpdateService;  //request the Service log to be updated.
volatile int serviceValue;           //value for the menu to determine which type of service to handle.
int backFlushErrorCode;     //Errorcode when backflusing is recommended.
int descaleErrorCode;       //Errorcode when descaling is recommended.
int grinderCleanErrorCode;  //Errorcode when the grinder neads to be cleaned.
int lastBackFlushYear;
int lastBackFlushMonth;
int lastBackFlushDay;
int lastBackFlushDate;
int lastGrinderCleanYear;
int lastGrinderCleanMonth;
int lastGrinderCleanDay;
int lastGrinderCleanDate;
int lastDescaleYear;
int lastDescaleMonth;
int lastDescaleDay;
int lastDescaleDate;


//RTC
int nowDate;
int nowTime;

//Scale
double currentWeight;
double beginWeight;

//// EEPROM ////
//Factory Reset.
volatile bool requestFactoryReset = false; //request a factory reset of all settings and logs.

//EEPROM addresses.
int addrBrewTempInt = 0;
int addrBrewTempDec = 1;
int addrSteamTempInt = 2;
int addrSteamTempDec = 3;
int addrBrewMethod = 4;
int addrShotWeight = 5;
int addrShotTime = 6;
int addrPiMethod = 7;
int addrPiTime = 8;
int addrWakeUp = 9;
int addrWakeUpTime = 10;
int addrUnits = 11;
int addrLastDescaleYear = 12;
int addrLastDescaleMonth = 13;
int addrLastDescaleDay = 14;
int addrLastBackFlushYear = 15;
int addrLastBackFlushMonth = 16;
int addrLastBackFlushDay = 17;
int addrLastGrinderCleanYear = 18;
int addrLastGrinderCleanMonth = 19;
int addrLastGrinderCleanDay = 20;
int addrFirstStart = 21;  //EEPROM might be reset if you cahnge this address
int addrIdleKp1 = 22;
int addrIdleKi1 = 23;
int addrIdleKd1 = 24;
int addrIdleKp2 = 25;
int addrIdleKi2 = 26;
int addrIdleKd2 = 27;
int addrMidKp1 = 28;
int addrMidKi1 = 29;
int addrMidKd1 = 30;
int addrMidKp2 = 31;
int addrMidKi2 = 32;
int addrMidKd2 = 33;
int addrAggrKp1 = 34;
int addrAggrKi1 = 35;
int addrAggrKd1 = 36;
int addrAggrKp2 = 37;
int addrAggrKi2 = 38;
int addrAggrKd2 = 39;


////// MENU //////
//menuStates to indicate in which level of menu we are.
volatile bool menuState = false;       //Menu is open or not.
volatile bool subMenuState = false;    //An item in the menu is opened for change, or not
bool menuExit = false;        //Menu is exited.
volatile bool requestMenuExit = false; //request for menu exit.

//Varialbles used by the menu.
const int menuLength = 15;      //number of items in the menu.
volatile int menuIndex = 0;        //index to indicate which menu item is selected.
volatile unsigned long menuTimer;  //A timer for auto exiting the menu if the menu buttons arent operated long enough.
int menuTimeOut = 10000;  //The time-out time for auto exiting the menu.


//the settings menu
char *mainMenu[] = {
  "Brew Temp.      ",
  "Steam Temp.     ",
  "PID Parameters  ",
  "Brew Method     ",
  "Shot Weight     ",
  "Shot Time       ",
  "Pre-Inf Method  ",
  "Pre-Inf Time    ",
  "Wake Up         ",
  "Wake Up Time    ",
  "Units           ",
  "Service log     ",
  "Update Service  ",
  "Factory Reset   ",
  "Exit            ",
};


//pins
const int select = 2;  //Interrupt
const int rotaryA = 3;  //Interrupt
const int rotaryB = 4;
const int ktcSO = 5;
const int ktcCLK = 6;
const int ktc1SC = 7;
const int ktc2SC = 8;
const int ssrBoiler1 = 9;
const int ssrBoiler2 = 10;
const int mrPump = 11;
const int mrSolenoid = 12;
const int onSwitch = 18; //Interrupt
const int brewSwitch = 19; //Interrupt
const int steamSwitch = 20; //Interrupt
const int RS = 22;
const int enable = 23;
const int D4 = 24;
const int D5 = 25;
const int D6 = 26;
const int D7 = 27;
const int onSwitchLed = 44;     //PWM: 2 to 13 and 44 to 46
const int brewSwitchLed = 45;   //PWM: 2 to 13 and 44 to 46
const int steamSwitchLed = 46;  //PWM: 2 to 13 and 44 to 46


//declare systems.
//temperature sensors.
MAX6675 ktc1(ktcCLK, ktc1SC, ktcSO);
MAX6675 ktc2(ktcCLK, ktc2SC, ktcSO);
//lcd screen
LiquidCrystal lcd(RS, enable, D4, D5, D6, D7);
//PIDs
PID pidBoiler1(&input1, &output1, &setpoint1, idleKp1, idleKi1, idleKd1, DIRECT);
PID pidBoiler2(&input2, &output2, &setpoint2, idleKp2, idleKi2, idleKd2, DIRECT);
//Real Time Clock
RTC_DS3231 rtc;

void IsrRotary() {
  menuTimer = millis();
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  // If interrupts come faster than 50ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 100) {
    if (menuState == true && subMenuState == false) {
      if (digitalRead(rotaryB) == LOW) {
        menuIndex--;
        if (menuIndex <= 0) {
          menuIndex += menuLength;
        }
      }
      else if (digitalRead(rotaryB) == HIGH) {
        menuIndex++;
        if (menuIndex >= menuLength) {
          menuIndex -= menuLength;
        }
      }
      Serial.println(menuIndex);
      // Keep track of when we were here last (no more than every 5ms) 
    }
    else if (menuState == true && subMenuState == true) {
      Serial.print(menuIndex); Serial.println("*");
      switch (menuIndex) {
        case 0:
          if (digitalRead(rotaryB) == HIGH && setpointC1 < 114.5) {
            setpointC1 += 0.5;
            setpointC2 += 0.5;
          }
          else if (digitalRead(rotaryB) == LOW && setpointC1 > 0.5) {
            setpointC1 -= 0.5;
            setpointC2 -= 0.5;
          }
          break;
        case 1:
          if (digitalRead(rotaryB) == HIGH && setpointCSteam < 169.5) {
            setpointCSteam += 0.5;
          }
          else if (digitalRead(rotaryB) == LOW && setpointCSteam > 100) {
            setpointCSteam -= 0.5;
          }
          break;
        case 2:
          break;
        case 3:
          if (brewMethod == 0) {
            brewMethod = 1;
          }
          else {
            brewMethod = 0;
          }
          break;
        case 4:
          if (digitalRead(rotaryB) == HIGH && maxShotWeight < 99) {
            maxShotWeight += 0.5;
          }
          else if (digitalRead(rotaryB) == LOW && maxShotWeight > 0.5) {
            maxShotWeight -= 0.5;
          }
          break;
        case 5:
          if (digitalRead(rotaryB) == HIGH && maxShotTime <= 99) {
            maxShotTime += 0.5;
          }
          else if (digitalRead(rotaryB) == LOW && maxShotTime > piTime + 1) {
            maxShotTime -= 0.5;
          }
          break;
        case 6:
          if (digitalRead(rotaryB) == HIGH && piMethod == 0) {
            piMethod = 1;
          }
          else if (digitalRead(rotaryB) == LOW && piMethod == 0) {
            piMethod = 2;
          }
          else if (digitalRead(rotaryB) == HIGH && piMethod == 1) {
            piMethod = 2;
          }
          else if (digitalRead(rotaryB) == LOW && piMethod == 1) {
            piMethod = 0;
          }
          else if (digitalRead(rotaryB) == HIGH && piMethod == 2) {
            piMethod = 0;
          }
          else if (digitalRead(rotaryB) == LOW && piMethod == 2) {
            piMethod = 1;
          }
          Serial.print("Pre-inf Method = "); Serial.println(piMethod);
          break;
        case 7:
          if (digitalRead(rotaryB) == HIGH && piTime < maxShotTime) {
            piTime += 0.1;
          }
          else if (digitalRead(rotaryB) == LOW && piTime > 0.1) {
            piTime -= 0.1;
          }
          break;
        case 8:
          if (wakeUp == 0) {
            wakeUp = 1;
          }
          else {
            wakeUp = 0;
          }
          Serial.print("Wake Up: "); Serial.println(wakeUp);
          break;
        case 9:
          if (digitalRead(rotaryB) == HIGH) {
            wakeUpTime += 1;
            if (wakeUpTime >= 96){
              wakeUpTime -= 96;
            }
          }
          else if (digitalRead(rotaryB) == LOW) {
            wakeUpTime -= 1;
            if (wakeUpTime < 0) {
              wakeUpTime += 96;
            }
          }
          break;
        case 10:
          if (units == 0) {
            units = 1;
          }
          else {
            units = 0;
          }
          Serial.print("Units: "); Serial.println(units);
          break;
        case 11:
          if (digitalRead(rotaryB) == HIGH && serviceValue == 0) {
            serviceValue = 1;
          }
          else if (digitalRead(rotaryB) == LOW && serviceValue == 0) {
            serviceValue = 2;
          }
          else if (digitalRead(rotaryB) == HIGH && serviceValue == 1) {
            serviceValue = 2;
          }
          else if (digitalRead(rotaryB) == LOW && serviceValue == 1) {
            serviceValue = 0;
          }
          else if (digitalRead(rotaryB) == HIGH && serviceValue == 2) {
            serviceValue = 0;
          }
          else if (digitalRead(rotaryB) == LOW && serviceValue == 2) {
            serviceValue = 1;
          }
          Serial.print("Service Value = "); Serial.println(serviceValue);
          break;
        case 12:
          if (digitalRead(rotaryB) == HIGH && serviceValue == 1) {
            serviceValue = 2;
          }
          else if (digitalRead(rotaryB) == LOW && serviceValue == 1) {
            serviceValue = 3;
          }
          else if (digitalRead(rotaryB) == HIGH && serviceValue == 2) {
            serviceValue = 3;
          }
          else if (digitalRead(rotaryB) == LOW && serviceValue == 2) {
            serviceValue = 1;
          }
          else if (digitalRead(rotaryB) == HIGH && serviceValue == 3) {
            serviceValue = 1;
          }
          else if (digitalRead(rotaryB) == LOW && serviceValue == 3) {
            serviceValue = 2;
          }
          Serial.print("Service Value = "); Serial.println(serviceValue);
          break;
        case 13:
          if (requestFactoryReset == false) {
            requestFactoryReset = true;
          }
          else {
            requestFactoryReset = false;
          }
          Serial.print("requestFactoryReset: "); Serial.println(requestFactoryReset);
          break;
        case 14:
          if (digitalRead(rotaryB) == HIGH && requestMenuExit == false) {
            requestMenuExit = true;
          }
          else {
            requestMenuExit = true;
          }
          Serial.print("requestMenuExit: "); Serial.println(requestMenuExit);
          break;
       
      }
    }
    lastInterruptTime = interruptTime;
  }
}


void IsrSelect() {
  menuTimer = millis();
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 100) {
    if (menuState == false) {
      menuState = true;
      Serial.print("menuState: "); Serial.print(menuState); Serial.print("\t");
      Serial.print("subMenuState: "); Serial.print(subMenuState); Serial.print("\t");
      Serial.println("Menu Opened");
    }
    else if (subMenuState == false) {
      subMenuState = true;
      Serial.print("menuState: "); Serial.print(menuState); Serial.print("\t");
      Serial.print("subMenuState: "); Serial.print(subMenuState); Serial.print("\t");
      Serial.println("Submenu Opened");
    }
    else if (subMenuState == true) {
      subMenuState = false;
      Serial.print("menuState: "); Serial.print(menuState); Serial.print("\t");
      Serial.print("subMenuState: "); Serial.print(subMenuState); Serial.print("\t");
      Serial.println("Submenu Closed");
      //If a factory reset is requested, preform it.
      if (requestFactoryReset == true) {
        factoryReset();
        loadEEPROM();
      }
      //If a menu exit is requested, exit it.
      if (requestMenuExit == true) {
        menuExit = true;
        Serial.print("Exiting Menu...");
      }
      //If an update of the last backflush date is requested, update it.
      if (requestUpdateService = 1) {
        lastBackFlushDate = nowDate;
      }
      //if an update of the last descaling dat is requested, update it.
      if (requestUpdateService = 2) {
        lastDescaleDate = nowDate;
      }
      //if an update of the last grinder cleaning is requested, update it.
      if (requestUpdateService = 3) {
        lastGrinderCleanDate = nowDate;
      }
    }
    lastInterruptTime = interruptTime;
  }
}


void setup() {
  //Start a serial connection at baut-rate 9600.
  Serial.begin(9600);

  //start Real time Clock.
  rtc.begin();
  Serial.println("Clock Started");

  //Initialize pins
  pinMode(select, INPUT);
  pinMode(rotaryA, INPUT);
  pinMode(rotaryB, INPUT);
  pinMode(onSwitch, INPUT);
  pinMode(brewSwitch, INPUT);
  pinMode(steamSwitch, INPUT);
  pinMode(ssrBoiler1, OUTPUT);
  pinMode(ssrBoiler2, OUTPUT);
  pinMode(mrPump, OUTPUT);
  pinMode(mrSolenoid, OUTPUT);
  pinMode(onSwitchLed, OUTPUT);
  pinMode(brewSwitchLed, OUTPUT);
  pinMode(steamSwitchLed, OUTPUT);
  Serial.println("Pins initialized");

  //Initialize interrupts
  attachInterrupt(digitalPinToInterrupt(rotaryA), IsrRotary, FALLING);
  attachInterrupt(digitalPinToInterrupt(select), IsrSelect, FALLING);
//  attachInterrupt(digitalPinToInterrupt(onSwitch, IsrOn, RISING);
//  attachInterrupt(digitalPinToInterrupt(brewSwitch, IsrBrew, RISING);
//  attachInterrupt(digitalPinToInterrupt(steamSwitch, IsrSteam, RISING);
  Serial.println("Interrupts initialized");
  
  //Start LCD screen and print loading screen.
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Gaggia Classic ");
  lcd.setCursor(0, 1);
  lcd.print("   Loading...   ");
  Serial.println("Loading Screen Done");

  lcd.createChar(0, steamLogo);
  lcd.createChar(1, piLogo);
  lcd.createChar(2, wakeUpLogo);

  //If code runs for the first time on a new Arduino, the EEPROM is loaded with factory settings.
  //The EEPROM value to recognize this is then changed.
  if (EEPROM.read(addrFirstStart) == 255) { //standard (new) EEPROM value is 0xFF = 255;
    factoryReset();
    EEPROM.write(addrFirstStart, 0);
  }
  delay(500);
  //loading EEPROM values into global variables.
  loadEEPROM();
  setpointC2 = setpointC1 - 3;
  delay(500);
  Serial.println("EEPROM is Loaded");

  //Set window start time for PID loop
  windowStartTime = millis();

  //Set range between 0 and windowsize
  pidBoiler1.SetOutputLimits(0, windowSize);
  pidBoiler2.SetOutputLimits(0, windowSize);

  //Turn PIDs on
  pidBoiler1.SetMode(AUTOMATIC);
  pidBoiler2.SetMode(AUTOMATIC);
  Serial.println("PIDs are set");

  //let the on-button led ring fade on and of at start-up.
  fadeLed(onSwitchLed, brewSwitchLed, steamSwitchLed);
  //Turn button led ring on.
  digitalWrite(onSwitchLed, HIGH);
  Serial.println("Switch animations are done");

  //Check if maintenance is needed.
//  checkService();
  
  //Show that the setup has been completed.
  lcd.setCursor(0, 0);
  lcd.print(" Gaggia Classic ");
  lcd.setCursor(0, 1);
  lcd.print("Is ready to use!");
  delay(5000);
  lcd.clear();
  Serial.println("Gaggia Classic is Ready for use!");
}


void loop() {
  if (menuState == true) {
    menuScreen();
  }
  else {
    mainScreen();
  }
  if (((millis() - menuTimer >= menuTimeOut) || (menuExit == true)) && menuState == true) {
    Serial.print("menuExit = "); Serial.println(menuExit);
    menuState = false;
    subMenuState = false;
    requestMenuExit = false;
    menuExit = false;
    requestFactoryReset = false;
    updateEEPROM();
    menuIndex = 0;
    Serial.print("menuState: "); Serial.print(menuState); Serial.print("\t");
    Serial.println("Menu Closed");
  }
  
//  lowPassTemp();
//  controlBrew();
//  controlSteam();
//  if (steamState == true) {
//    pidSteam();
//  }
//  else {
//    pidBrew();
//  }
//  if (brewState == true) {
//    brewing();
//  }
//  //the thermocouples need time to settle. (approx. 200ms)
  delay(170);
}

void brewing() {
  currentWeight = analogRead(A0); //read the current weight from the scale
  if (brewMethod == 0) {
    if (piMethod == 0) {
      digitalWrite(mrSolenoid, HIGH);   //Solenoid valve closes.
      delay(50);  //wait for the valve to be fully closed.
      digitalWrite(mrPump, HIGH); //turn on pump.
    }
    else if (piMethod == 1) {
      digitalWrite(mrSolenoid, HIGH);    //Solenoid valve closes
      if (millis() - brewBegin >= piTime) { //wait for piTime to pass, then turn on pump.
        digitalWrite(mrPump, HIGH);
      }
    }
    else {
      digitalWrite(mrSolenoid, HIGH);   //Solenoid valve closes.
      delay(50);  //wait for valve to be fully closed.
      for (int i = 0; i < 3; i++)  {  //turn the pump on and off three times, 300 milliseconds on, 700 milliseconds off.
        digitalWrite(mrPump, HIGH);
        delay(300);
        digitalWrite(mrPump, LOW);
        delay(700);
      }
      if (millis() - brewBegin >= piTime) { //when piTime has passed, start the pump
        digitalWrite(mrPump, HIGH);
      }
    }
  }
  else {
    if (piMethod == 0) {
      digitalWrite(mrSolenoid, HIGH);   //Solenoid valve closes.
      delay(50);  //wait for the valve to be fully closed.
      digitalWrite(mrPump, HIGH); //turn on pump.
      if (millis() - brewBegin >= maxShotTime || currentWeight - beginWeight >= maxShotWeight) {  //if the current weight or shottime surpasses their max value, stop brewing
        brewState = false;
        digitalWrite(mrPump, LOW);
        delay(50);
        digitalWrite(mrSolenoid, LOW);
        holdShotParametersState = true;         //A boolean to indicate that shottime must be held.
        holdShotParametersBegin = millis();     //A timer to keep the time that the shot time is displayed.
      }
    }
    else if (piMethod == 1) {
      digitalWrite(mrSolenoid, HIGH);    //Solenoid valve closes
      if (millis() - brewBegin >= piTime) { //wait for piTime to pass, then turn on pump.
        digitalWrite(mrPump, HIGH);
      }
      if (millis() - brewBegin >= maxShotTime || currentWeight - beginWeight >= maxShotWeight) { //if the current weight or shottime surpasses their max value, stop brewing
        brewState = false;
        digitalWrite(mrPump, LOW);
        delay(50);
        digitalWrite(mrSolenoid, LOW);
        holdShotParametersState = true;         //A boolean to indicate that shottime must be held.
        holdShotParametersBegin = millis();     //A timer to keep the time that the shot time is displayed.
      }
    }
    else {
      digitalWrite(mrSolenoid, HIGH);   //Solenoid valve closes.
      delay(50);  //wait for valve to be fully closed.
      for (int i = 0; i < 3; i++)  {  //turn the pump on and off three times, 300 milliseconds on, 700 milliseconds off.
        digitalWrite(mrPump, HIGH);
        delay(300);
        digitalWrite(mrPump, LOW);
        delay(700);
      }
      if (millis() - brewBegin >= piTime) {
        digitalWrite(mrPump, HIGH);
      }
      if (millis() - brewBegin >= maxShotTime || currentWeight - beginWeight >= maxShotWeight) { //if the current weight or shottime surpasses their max value, stop brewing
        brewState = false;
        digitalWrite(mrPump, LOW);
        delay(50);
        digitalWrite(mrSolenoid, LOW);
        holdShotParametersState = true;         //A boolean to indicate that shottime must be held.
        holdShotParametersBegin = millis();     //A timer to keep the time that the shot time is displayed.
      }
    }
  }
}

void mainScreen() {
  /* In the bottom right quatre, the boiler2 temperature or shot time or final shot time
     is printed depending on which condition is met.
     On the bottom left quatre, the boiler1 temperature is printed.
     On the top left quatre, the desired temperature for boiler1 is printed.
     On the top right quatre, the logo's indicating;
     Pre-Infusion, WakeUpAlarm, Auto or manual brewing, service message and steam on, are printed.
  */
  //if brewing, show the current shot time.
  if (brewState == true) {
    shotWeight = (currentWeight - beginWeight);
    lcd.setCursor(0,1);
    if (shotWeight < 10) {
      lcd.print("  ");
      lcd.print(shotWeight, 1);
      lcd.print(" g"); 
    }
    else {
      lcd.print(" ");
      lcd.print(shotWeight, 1);
      lcd.print(" g");
    }
    finalShotWeight = shotWeight;
    shotTimer = (millis() - brewBegin) / 1000.0;
    lcd.setCursor(8, 1);
    if (shotTimer < 10) {
      lcd.print("    ");
      lcd.print(shotTimer, 1);
      lcd.print(" s");
    }
    else {
      lcd.print("   ");
      lcd.print(shotTimer, 1);
      lcd.print(" s");
    }
    finalShotTime = shotTimer;
  }
  //after brewing is finished, hold the final shot time on the screen for 10 seconds
  else if (holdShotParametersState == true) {
    lcd.setCursor(0,1);
    if (finalShotWeight < 10) {
      lcd.print("  ");
      lcd.print(finalShotWeight, 1);
      lcd.print(" g");
    }
    else {
      lcd.print(" ");
      lcd.print(finalShotWeight, 1);
      lcd.print(" g");
    }
    lcd.setCursor(8, 1);
    if (finalShotTime < 10) {
      lcd.print("    ");
      lcd.print(finalShotTime, 1);
      lcd.print(" s");
    }
    else {
      lcd.print("   ");
      lcd.print(finalShotTime, 1);
      lcd.print(" s");
    }
  }
  else {
    //Print temperature boiler 2, making sure the decimal dot is always on the same postion.
    lcd.setCursor(7, 1);
    if (roundedTempC2 < 10) {
      lcd.print("    ");
      lcd.print(roundedTempC2, 1);
    }
    else if (roundedTempC2 >= 10 && roundedTempC1 < 100) {
      lcd.print("   ");
      lcd.print(roundedTempC2, 1);
    }
    else {
      lcd.print("  ");
      lcd.print(roundedTempC2, 1);
    }
    lcd.print((char)223);
    lcd.print("C");
  }
  //Print measured temperature in Celsius
  lcd.setCursor(0, 1);
  if (roundedTempC1 < 10) {
    lcd.print("  ");
    lcd.print(roundedTempC1, 1);
  }
  else if (roundedTempC1 >= 10 && roundedTempC1 < 100) {
    lcd.print(" ");
    lcd.print(roundedTempC1, 1);
  }
  else {
    lcd.print(roundedTempC1, 1);
  }
  lcd.print((char)223);
  lcd.print("C   ");
  
  //print setpoint boiler 1.
  lcd.setCursor(0, 0);
  if (setpointC1 < 10) {
    lcd.print("  ");
    lcd.print(setpointC1, 1);
  }
  else if (setpointC2 >= 10 && setpointC1 < 100) {
    lcd.print(" ");
    lcd.print(setpointC1, 1);
  }
  else {
    lcd.print(setpointC1, 1);
  }
  lcd.print((char)223);
  lcd.print("C");

  //Print logo's on the display.
  lcd.setCursor(7,0);
  lcd.print("  ");
  lcd.setCursor(9,0);
  if (brewMethod == 0){
    lcd.print("M");
  }
  else {
    lcd.print("A");
  }
  lcd.setCursor(10,0);
  if (piMethod == 1 || piMethod == 2){
    lcd.write((uint8_t)1);
  }
  else {
    lcd.print(" ");
  }
  lcd.setCursor(11,0);
  if (steamState == true) {
    lcd.write((uint8_t)0);
  }
  else {
    lcd.print(" ");
  }
  lcd.setCursor(12, 0);
  lcd.print(" ");
  lcd.setCursor(13,0);
  if (backFlushErrorCode == true || grinderCleanErrorCode == true || descaleErrorCode == true) {
    lcd.print("S"); 
  }
  else {
    lcd.print(" ");
  }
  lcd.setCursor(15,0);
  if (wakeUp == 1) {
    lcd.write((uint8_t)2);
  }
  else {
    lcd.print(" ");
  }

  //button led rings fading.
  //SteamSwitch
  if (steamState == true) {
    if (readyToSteam == true) {
      digitalWrite(steamSwitchLed, HIGH);
    }
    else {
      brightness += fadeStep;
      analogWrite(steamSwitchLed, brightness);
      if (brightness <= 0 || brightness >= 255) {
        fadeStep = -fadeStep;
      }
      delay(10);
    }
  }
  else {
    //brewSwitch
    if (readyToBrew1 == true && readyToBrew2 == true) {
      digitalWrite(brewSwitchLed, HIGH);
    }
    else {
      brightness += fadeStep;
      analogWrite(brewSwitchLed, brightness);
      if (brightness <= 0 || brightness >= 255) {
        fadeStep = -fadeStep;
      }
      delay(10);
    }
  }
}

void lowPassTemp() {

  //temperature sensor readings in Celsius with an offset.
  tempC1 = ktc1.readCelsius() - ktcOffsetC1;
  tempC2 = ktc2.readCelsius() - ktcOffsetC2;

  //Low-pass filter.

  lowPassTempC1 = cutoff1 * lowPassTempC1 + (1 - cutoff1) * tempC1;
  lowPassTempC2 = cutoff2 * lowPassTempC2 + (1 - cutoff2) * tempC2;

  //Round temperature from boiler 1 to 0.5 for lcd display
  //Removing the integer part and multiplying the decimal part by 100.
  //Then sort, round and devide back by 100 and add to the integer part again.
  intPartC1 = (int)(lowPassTempC1);
  deciPartC1 = 100 * (lowPassTempC1 - intPartC1);
  if (deciPartC1 < 25) {
    roundedTempC1 = intPartC1;
  }
  if (deciPartC1 >= 25 && deciPartC1 < 75) {
    roundedTempC1 = intPartC1 + 0.5;
  }
  if (deciPartC1 >= 75) {
    roundedTempC1 = intPartC1 + 1;
  }
}

void pidSteam() {
  //if more time has gone by between the start of one PID cycle keep
  //adding the windowsize to it untill it's within the windowsize.
  if (millis() - windowStartTime > windowSize) {
    windowStartTime += windowSize;
  }

  //If the direct measured/unfiltered temperature is lower than the setpoint minus a boundry, give a HIGH to the boilers SSR.
  if (tempC1 < (setpointCSteam - 5)) {
    digitalWrite(ssrBoiler1, HIGH);
    Serial.print(setpointCSteam + 5); Serial.print(" ");
    idleSteam1 = false;
  }
  //If the direct measured/unfiltered temperature is above the setpoint minus a boundry, start the PID cycle.
  if (tempC1 >= (setpointCSteam - 5)) {
    //if the filtered temperature is within one degree from the setpoint, set PID parameters to "idle".
    if ((setpointCSteam - 1) < lowPassTempC1 < (setpointCSteam + 1)) {
      pidBoiler1.SetTunings(idleKp1, idleKi1, idleKd1);
      idleSteam1 = true;
    }
    //Else set the PID parameters to "medium".
    else {
      pidBoiler1.SetTunings(midKp1, midKi1, midKd1);
      idleSteam1 = false;
    }
    //Set the Setpoint en Input and start the PID calculations.
    setpoint1 = setpointCSteam;
    input1 = lowPassTempC1;
    pidBoiler1.Compute();

    //Given a PID output, Check if it is higher then the time past since the windowStartTime.
    //If so, write HIGH to the boiler SSR, until the time past since the last windowStartTime
    // is higher then the output, then write LOW to the SSR.
    if (output1 < millis() - windowStartTime) {
      digitalWrite(ssrBoiler1, LOW);
      Serial.print(setpointC1 - 5); Serial.print(" ");
    }
    else {
      digitalWrite(ssrBoiler1, HIGH);
      Serial.print(setpointC1 + 5); Serial.print(" ");
    }
  }

  //If the direct measured/unfiltered temperature is lower than the setpoint minus a boundry, give a HIGH to the boilers SSR.
  if (tempC2 < (setpointC2 - 5)) {
    digitalWrite(ssrBoiler2, HIGH);
    Serial.print(setpointC2 + 5); Serial.print(" ");
    idleBrew2 = false;
  }
  //If the direct measured/unfiltered temperature is above the setpoint minus a boundry, start the PID cycle.
  if (tempC2 >= (setpointC2 - 5)) {
    //if the filtered temperature is within one degree from the setpoint, set PID parameters to "idle".
    if ((setpointC2 - 1) < lowPassTempC2 < (setpointC2 + 1)) {
      pidBoiler2.SetTunings(idleKp2, idleKi2, idleKd2);
      idleBrew2 = true;
    }
    //if the machine is brewing coffee, set PID paramaters to "aggresive".
    else if (brewState == true) {
      pidBoiler2.SetTunings(aggrKp2, aggrKi2, aggrKd2);
      idleBrew2 = false;
    }
    //Else set the PID parameters to "medium".
    else {
      pidBoiler2.SetTunings(midKp2, midKi2, midKd2);
      idleBrew2 = false;
    }
    //Set the Setpoint en Input and start the PID calculations.
    setpoint2 = setpointC2;
    input2 = lowPassTempC2;
    pidBoiler2.Compute();

    //Given a PID output, Check if it is higher then the time past since the windowStartTime.
    //If so, write HIGH to the boiler SSR, until the time past since the last windowStartTime
    // is higher then the output, then write LOW to the SSR.
    if (output2 < millis() - windowStartTime) {
      digitalWrite(ssrBoiler2, LOW);
      Serial.print(setpointC2 - 5); Serial.print(" ");
    }
    else {
      digitalWrite(ssrBoiler2, HIGH);
      Serial.print(setpointC2 + 5); Serial.print(" ");
    }
  }

  //print parameters for analysing the PID loop
  Serial.print(tempC1); Serial.print(" ");
  Serial.print(lowPassTempC1); Serial.print(" ");
  Serial.print(setpoint1); Serial.print(" ");
  Serial.print(setpoint1 - 0.5); Serial.print(" ");
  Serial.print(setpoint1 + 0.5); Serial.print(" ");

  Serial.print(tempC2); Serial.print(" ");
  Serial.print(lowPassTempC2); Serial.print(" ");
  Serial.print(setpoint2); Serial.print(" ");
  Serial.print(setpoint2 - 0.5); Serial.print(" ");
  Serial.println(setpoint2 + 0.5); Serial.print(" ");

}

//void checkReadyToBrew() {
//  currentIdleBrew1 = idleBrew1;
//  currentIdle2 = idleBrew2;
//  if (currentIdleBrew1 == true && lastIdleBrew1 == false) {
//    readyToBrewTimer = millis();
//    lastIdleBrew1 = currentIdleBrew1;
//  }
//  if (millis() - readyToBrewTimer >= 60000) {
//    readyToBrew1 = true;    
//  }
//  if (currentIdleBrew1 == false && readyToBrew == true) {
//    readyToBrew = false;
//  }
//  if (currentIdle2 == true && lastIdle2 == false) {
//    readyToBrew2 = true;
//    lastidleBrew2 = currentIdle2;
//  }
//  if (millis() - readyToBrewTimer >= 60000) {
//    readyToBrew2 = true;
//  }
//  if (currentIdle2 == false && readyToBrew == true) {
//    readyToBrew = false;
//  }
//}
//
//void checkReadyToSteam() {
//  currentIdleSteam = idleSteam;
//  if (currentIdleSteam == true && lastIdleSteam == false) {
//    readyToSteamTimer = millis();
//    lastIdleSteam = currentIdleSteam;
//  }
//  if (millis() - readyToSteamTimer >= 60000) {
//    readyToSteam= true;    
//  }
//  if (currentIdleSteam == false && readyToSteam == true) {
//    readyToSteam = false;
//  }
//}

void pidBrew() {
  //if more time has gone by between the start of one PID cycle keep
  //adding the windowsize to it untill it's within the windowsize.
  if (millis() - windowStartTime > windowSize) {
    windowStartTime += windowSize;
  }

  //If the direct measured/unfiltered temperature is lower than the setpoint minus a boundry, give a HIGH to the boilers SSR.
  if (tempC1 < (setpointC1 - 5)) {
    digitalWrite(ssrBoiler1, HIGH);
    Serial.print(setpointC1 + 5); Serial.print(" ");
    idleBrew1 = false;
  }
  //If the direct measured/unfiltered temperature is above the setpoint minus a boundry, start the PID cycle.
  if (tempC1 >= (setpointC1 - 5)) {
    //if the filtered temperature is within one degree from the setpoint, set PID parameters to "idle".
    if ((setpointC1 - 1) < lowPassTempC1 < (setpointC1 + 1)) {
      pidBoiler1.SetTunings(idleKp1, idleKi1, idleKd1);
      idleBrew1 = true;
    }
    //if the machine is brewing coffee, set PID paramaters to "aggresive".
    else if (brewState == true) {
      pidBoiler1.SetTunings(aggrKp1, aggrKi1, aggrKd1);
      idleBrew1 = false;
    }
    //Else set the PID parameters to "medium".
    else {
      pidBoiler1.SetTunings(midKp1, midKi1, midKd1);
      idleBrew1 = false;
    }
    //Set the Setpoint en Input and start the PID calculations.
    setpoint1 = setpointC1;
    input1 = lowPassTempC1;
    pidBoiler1.Compute();

    //Given a PID output, Check if it is higher then the time past since the windowStartTime.
    //If so, write HIGH to the boiler SSR, until the time past since the last windowStartTime
    // is higher then the output, then write LOW to the SSR.
    if (output1 < millis() - windowStartTime) {
      digitalWrite(ssrBoiler1, LOW);
      Serial.print(setpointC1 - 5); Serial.print(" ");
    }
    else {
      digitalWrite(ssrBoiler1, HIGH);
      Serial.print(setpointC1 + 5); Serial.print(" ");
    }
  }

  //If the direct measured/unfiltered temperature is lower than the setpoint minus a boundry, give a HIGH to the boilers SSR.
  if (tempC2 < (setpointC2 - 5)) {
    digitalWrite(ssrBoiler2, HIGH);
    Serial.print(setpointC2 + 5); Serial.print(" ");
    idleBrew2 = false;
  }
  //If the direct measured/unfiltered temperature is above the setpoint minus a boundry, start the PID cycle.
  if (tempC2 >= (setpointC2 - 5)) {
    //if the filtered temperature is within one degree from the setpoint, set PID parameters to "idle".
    if ((setpointC2 - 1) < lowPassTempC2 < (setpointC2 + 1)) {
      pidBoiler2.SetTunings(idleKp2, idleKi2, idleKd2);
      idleBrew2 = true;
    }
    //if the machine is brewing coffee, set PID paramaters to "aggresive".
    else if (brewState == true) {
      pidBoiler2.SetTunings(aggrKp2, aggrKi2, aggrKd2);
      idleBrew2 = false;
    }
    //Else set the PID parameters to "medium".
    else {
      pidBoiler2.SetTunings(midKp2, midKi2, midKd2);
      idleBrew2 = false;
    }
    //Set the Setpoint en Input and start the PID calculations.
    setpoint2 = setpointC2;
    input2 = lowPassTempC2;
    pidBoiler2.Compute();

    //Given a PID output, Check if it is higher then the time past since the windowStartTime.
    //If so, write HIGH to the boiler SSR, until the time past since the last windowStartTime
    // is higher then the output, then write LOW to the SSR.
    if (output2 < millis() - windowStartTime) {
      digitalWrite(ssrBoiler2, LOW);
      Serial.print(setpointC2 - 5); Serial.print(" ");
    }
    else {
      digitalWrite(ssrBoiler2, HIGH);
      Serial.print(setpointC2 + 5); Serial.print(" ");
    }
  }

  //print parameters for analysing the PID loop
  Serial.print(tempC1); Serial.print(" ");
  Serial.print(lowPassTempC1); Serial.print(" ");
  Serial.print(setpoint1); Serial.print(" ");
  Serial.print(setpoint1 - 0.5); Serial.print(" ");
  Serial.print(setpoint1 + 0.5); Serial.print(" ");

  Serial.print(tempC2); Serial.print(" ");
  Serial.print(lowPassTempC2); Serial.print(" ");
  Serial.print(setpoint2); Serial.print(" ");
  Serial.print(setpoint2 - 0.5); Serial.print(" ");
  Serial.println(setpoint2 + 0.5); Serial.print(" ");

}

void controlSteam() {
  currentSteamSwitchState = digitalRead(steamSwitch);
  if (currentSteamSwitchState == HIGH && lastSteamSwitchState == LOW && steamState == false && brewState == false) {
    steamState = true;
    lastSteamSwitchState = currentSteamSwitchState;
  }
  if (currentSteamSwitchState == HIGH && lastSteamSwitchState == LOW && steamState == true) {
    steamState = false;
  }
  lastSteamSwitchState = currentSteamSwitchState;
}

void controlBrew() {
  currentBrewSwitchState = digitalRead(brewSwitch);
  if (currentBrewSwitchState == HIGH && lastBrewSwitchState == LOW && brewState == false && steamState == false) {
    brewState = true;                 //boolean to indicate that brewing is on
    brewBegin = millis();             //Start a timer to time the shot time
    lastBrewSwitchState = currentBrewSwitchState;
    beginWeight = analogRead(A0); //read the begin weight from the scale to auot tare
  }
  else if (currentBrewSwitchState == HIGH && lastBrewSwitchState == LOW && brewState == true && brewMethod == 0) {  //Brewing is stopped once the button is pressed and then released, only if brewing was on.
    brewState = false;                //boolean to indicate brewing has stopped.
    holdShotParametersState = true;         //A boolean to indicate that shottime must be held.
    holdShotParametersBegin = millis();     //A timer to keep the time that the shot time is displayed.
  }
  lastBrewSwitchState = currentBrewSwitchState;
  if (millis() - holdShotParametersBegin >= holdShotParametersLength) { //if the hold the shot time has timed-out. The boolean can return to false to stop holding the shot time.
    holdShotParametersState = false;
  }
}


//void menuExit() {
//  /*  To open the Menu press the select button.
//      Once the menu is opened, you can scroll in the menu by rotating the wheel. It's a infinite scroller.
//      Once the menu is opened, a timer starts, if the select button is pressed or rotated, the timer is set back to 0. If the timer reaches the Time-Out time, the Menu auto-exits.
//      A item can be selected by scrolling to the item in the menu en pressing the select button. Once a item is selected it can be changed bij rotating the wheel. The item is deselected by pressing the slect button again.
//      The menu can be exited by selecting the exit item.
//      Once it's exited (both by time-out and manual exit) the EEPROM is updated with the changes made.
//  */
//
//  //if the menu timer has reached the timeout time, or the exit item in the menu is selected, exit the menu and update the EEPROM.
//  
//}

void menuScreen() {
  /*  This function only runs if menuState == true. This is achieved within menuSelect.

      If menuIndex is changed (by selecting a different item in the menu, see menuSelect()) a different case is selected.
      This function is responsible for changing the variables and dispaying the menu, item and values on the LCD screen.
      Once the menu is selected, different cases can be selected representing different items in the menu.
      Once an item is selected, the submenu is opened and the values that it holds can be changed through this function
      Once the submenu is left, the value is saved.

      Menu:
      Brew Temp
      Steam Temp
      PID parameters
      Brew Method
      Shot Weight
      Shot Time
      Pre-Inf Method
      Pre-Inf Time
      Wake Up
      Wake Up Time
      Units
      Service Log
      Update Service
      Factory Reset
      Exit
  */
  switch (menuIndex) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print(mainMenu[menuIndex]);
      lcd.setCursor(0, 1);
      if (setpointC1 < 10) {
        lcd.print("T =  ");
        lcd.print(setpointC1, 1);
        lcd.print(" ");
        lcd.print((char)223);
        lcd.print("C     ");
      }
      else {
        lcd.print("T = ");
        lcd.print(setpointC1, 1);
        lcd.print(" ");
        lcd.print((char)223);
        lcd.print("C     ");
      }
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print(mainMenu[menuIndex]);
      lcd.setCursor(0, 1);
      lcd.print("T = ");
      lcd.print(setpointCSteam, 1);
      lcd.print(" ");
      lcd.print((char)223);
      lcd.print("C    ");
      break;
    case 2:
      lcd.setCursor(0,0);
      lcd.print(mainMenu[menuIndex]);
      lcd.setCursor(0,1);
      lcd.print("Not yet working ");
      break;
    case 3:
      if (brewMethod == 0) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Manual         ");
      }
      else if (brewMethod == 1) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Automatic      ");
      }
      break;
    case 4:
      lcd.setCursor(0, 0);
      lcd.print(mainMenu[menuIndex]);
      lcd.setCursor(0, 1);
      if (maxShotWeight < 10) {
        lcd.print("W =  ");
        lcd.print(maxShotWeight, 1);
        lcd.print(" g      ");        
      }
      else {
        lcd.print("W = ");
        lcd.print(maxShotWeight, 1);
        lcd.print(" g      ");
      }
      break;
    case 5:
      lcd.setCursor(0, 0);
      lcd.print(mainMenu[menuIndex]);
      lcd.setCursor(0, 1);
      if (maxShotTime < 10) {
        lcd.print("t =  ");
        lcd.print(maxShotTime, 1);
        lcd.print(" s      ");
      }
      else {
        lcd.print("t = ");
        lcd.print(maxShotTime, 1);
        lcd.print(" s      ");
      }
      break;
    case 6:
      if (piMethod == 0) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Off             ");
      }
      else if (piMethod == 1) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Boiler Pressure ");
      }
      else if (piMethod == 2) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Pump Pressure   ");
      }
      break;
    case 7:
      lcd.setCursor(0, 0);
      lcd.print(mainMenu[menuIndex]);
      lcd.setCursor(0, 1);
      if (piTime < 10) {
        lcd.print("t =  ");
        lcd.print(piTime, 1);
        lcd.print(" s      ");
      }
      else {
        lcd.print("t = ");
        lcd.print(piTime, 1);
        lcd.print(" s      ");
      }
      break;
    case 8:
      if (wakeUp == 0) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Off             ");
      }
      else if (wakeUp == 1) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("On              ");
      }
      break;
    case 9:
      wakeUpHour = wakeUpTime / 4;
      wakeUpMinute = (wakeUpTime % 4) * 15;
      lcd.setCursor(0, 0);
      lcd.print(mainMenu[menuIndex]);
      lcd.setCursor(0, 1);
      if (wakeUpHour < 10) {
        lcd.print("t =  0");
        lcd.print(wakeUpHour);
        lcd.print(":"); 
      }
      else {
        lcd.print("t =  ");
        lcd.print(wakeUpHour);
        lcd.print(":");
      }
      if (wakeUpMinute < 10) {
        lcd.print("0");
        lcd.print(wakeUpMinute);
        lcd.print("      ");
      }
      else {
        lcd.print(wakeUpMinute);
        lcd.print("      ");
      }
      break;
    case 10:
      if (units == 0) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Metric          ");
      }
      else if (units == 1) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Imperical       ");
      }
      break;
    case 11:
      if (serviceValue == 0) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("BkFl: ");
        lcd.print(lastBackFlushDate);
      }
      if (serviceValue == 1) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("DeSc: ");
        lcd.print(lastDescaleDate);
      }
      if (serviceValue == 2) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("GrCl: ");
        lcd.print(lastGrinderCleanDate);
      }
      break;
    case 12:
      if (serviceValue == 1) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Back-Flush Date ");
        requestUpdateService = 1;
      }
      if (serviceValue == 2) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Descale Date    ");
        requestUpdateService = 2;
      }
      if (serviceValue == 3) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Grndr Clean Date");
        requestUpdateService = 3;
      }
      break;
    case 13:
      if (requestFactoryReset == false) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Are you sure?  N");

      }
      else if (requestFactoryReset == true) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Are you sure?  Y");

      }
      break;
    case 14:
      if (requestMenuExit == false) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Are you sure?  N");

      }
      else if (requestMenuExit == true) {
        lcd.setCursor(0, 0);
        lcd.print(mainMenu[menuIndex]);
        lcd.setCursor(0, 1);
        lcd.print("Are you sure?  Y");
      }
      break;
  }
}


void updateEEPROM() {
  // A function that updates all the EEPROM values.
  EEPROM.write(addrBrewTempInt, (int)(setpointC1));
  EEPROM.write(addrBrewTempDec, 10 * (setpointC1 - (int)(setpointC1)));
  EEPROM.write(addrSteamTempInt, (int)(setpointCSteam));
  EEPROM.write(addrSteamTempDec, 10 * (setpointCSteam - (int)(setpointCSteam)));
  EEPROM.write(addrBrewMethod, brewMethod);                //0 = Manual, 1 = Automatic
  EEPROM.write(addrShotWeight, maxShotWeight);
  EEPROM.write(addrShotTime, maxShotTime);
  EEPROM.write(addrPiMethod, piMethod);                  //0 = PI off, 1 = PI Boiler Pressure, 2 = PI Pump Pressure
  EEPROM.write(addrPiTime, piTime);
  EEPROM.write(addrWakeUp, wakeUp);                    //0 = wakeup off, 1 = wakeup on
  EEPROM.write(addrWakeUpTime, wakeUpTime);
  EEPROM.write(addrLastDescaleYear, lastDescaleYear);
  EEPROM.write(addrLastDescaleMonth, lastDescaleMonth);
  EEPROM.write(addrLastDescaleDay, lastDescaleDay);
  EEPROM.write(addrLastBackFlushYear, lastBackFlushYear);
  EEPROM.write(addrLastBackFlushMonth, lastBackFlushMonth);
  EEPROM.write(addrLastBackFlushDay, lastBackFlushDay);
  EEPROM.write(addrLastGrinderCleanYear, lastGrinderCleanYear);
  EEPROM.write(addrLastGrinderCleanMonth, lastGrinderCleanMonth);
  EEPROM.write(addrLastGrinderCleanDay, lastGrinderCleanDay);
  EEPROM.write(addrUnits, units);                   //0 = Metric (gr/Celsius), 1 = Imperial(oz./Fahrenheit)
  EEPROM.write(addrIdleKp1, 17);
  EEPROM.write(addrIdleKi1, 0.1);
  EEPROM.write(addrIdleKd1, 60);
  EEPROM.write(addrIdleKp2, midKp1);
  EEPROM.write(addrIdleKi2, midKi1);
  EEPROM.write(addrIdleKd2, midKd1);
  EEPROM.write(addrMidKp1, midKp1);
  EEPROM.write(addrMidKi1, midKi1);
  EEPROM.write(addrMidKd1, midKd1);
  EEPROM.write(addrMidKp2, midKp2);
  EEPROM.write(addrMidKi2, midKi2);
  EEPROM.write(addrMidKd2, midKd2);
  EEPROM.write(addrAggrKp1, aggrKp1);
  EEPROM.write(addrAggrKi1, aggrKi1);
  EEPROM.write(addrAggrKd1, aggrKd1);
  EEPROM.write(addrAggrKp2, aggrKp2);
  EEPROM.write(addrAggrKi2, aggrKi2);
  EEPROM.write(addrAggrKd2, aggrKd2);
}

void factoryReset() {
  Serial.println("Factory Reset... ");
  EEPROM.write(addrBrewTempInt, 95);
  EEPROM.write(addrBrewTempDec, 0);
  EEPROM.write(addrSteamTempInt, 145);
  EEPROM.write(addrSteamTempDec, 0);
  EEPROM.write(addrBrewMethod, 0);                //0 = Manual, 1 = Automatic
  EEPROM.write(addrShotWeight, 45);
  EEPROM.write(addrShotTime, 30);
  EEPROM.write(addrPiMethod, 0);                  //0 = PI off, 1 = PI Boiler Pressure, 2 = Pump Pressure
  EEPROM.write(addrPiTime, 5);
  EEPROM.write(addrWakeUp, 0);                    //0 = wakeup off, 1 = wakeup on
  EEPROM.write(addrWakeUpTime, 28);
  EEPROM.write(addrLastDescaleYear, 17);
  EEPROM.write(addrLastDescaleMonth, 1);
  EEPROM.write(addrLastDescaleDay, 1);
  EEPROM.write(addrLastBackFlushYear, 17);
  EEPROM.write(addrLastBackFlushMonth, 1);
  EEPROM.write(addrLastBackFlushDay, 1);
  EEPROM.write(addrLastGrinderCleanYear, 17);
  EEPROM.write(addrLastGrinderCleanMonth, 1);
  EEPROM.write(addrLastGrinderCleanDay, 1);
  EEPROM.write(addrUnits, 0);                   //0 = Metric (gr/Celsius), 1 = Imperial(oz./Fahrenheit)
  EEPROM.write(addrIdleKp1, idleKp1);
  EEPROM.write(addrIdleKi1, idleKi1);
  EEPROM.write(addrIdleKd1, idleKd1);
  EEPROM.write(addrIdleKp2, midKp1);
  EEPROM.write(addrIdleKi2, midKi1);
  EEPROM.write(addrIdleKd2, midKd1);
  EEPROM.write(addrMidKp1, midKp1);
  EEPROM.write(addrMidKi1, midKi1);
  EEPROM.write(addrMidKd1, midKd1);
  EEPROM.write(addrMidKp2, midKp2);
  EEPROM.write(addrMidKi2, midKi2);
  EEPROM.write(addrMidKd2, midKd2);
  EEPROM.write(addrAggrKp1, aggrKp1);
  EEPROM.write(addrAggrKi1, aggrKi1);
  EEPROM.write(addrAggrKd1, aggrKd1);
  EEPROM.write(addrAggrKp2, aggrKp2);
  EEPROM.write(addrAggrKi2, aggrKi2);
  EEPROM.write(addrAggrKd2, aggrKd2);
  requestFactoryReset = false;
  Serial.println("Done");
}

void loadEEPROM() {
  //Values from EEPROM are loaded into global variables and booleans are coupled with their strings.
  setpointC1 = EEPROM.read(addrBrewTempInt) + (EEPROM.read(addrBrewTempDec) / 10.0);
  setpointCSteam = EEPROM.read(addrSteamTempInt) + (EEPROM.read(addrSteamTempDec) / 10.0);
  if (EEPROM.read(addrBrewMethod) == 0) {             //0 = Manual, 1 = Automatic
    brewMethod = 0;
  }
  else {
    brewMethod = 1;
  }
  maxShotWeight = EEPROM.read(addrShotWeight);
  maxShotTime = EEPROM.read(addrShotTime);
  if (EEPROM.read(addrPiMethod) == 0) {                  //0 = PI off, 1 = PI on
    piMethod = 0;
  }
  else if (EEPROM.read(addrPiMethod) == 1) {
    piMethod = 1;
  }
  else {
    piMethod = 2;
  }
  piTime = EEPROM.read(addrPiTime);
  if (EEPROM.read(addrWakeUp) == 0) {                    //0 = wakeup off, 1 = wakeup on
    wakeUp = 0;
  }
  else {
    wakeUp = 1;
  }
  wakeUpTime = EEPROM.read(addrWakeUpTime);
  lastDescaleYear = EEPROM.read(addrLastDescaleYear);
  lastDescaleMonth = EEPROM.read(addrLastDescaleMonth);
  lastDescaleDay = EEPROM.read(addrLastDescaleDay);
  lastBackFlushYear = EEPROM.read(addrLastBackFlushYear);
  lastBackFlushMonth = EEPROM.read(addrLastBackFlushMonth);
  lastBackFlushDay = EEPROM.read(addrLastBackFlushDay);
  lastGrinderCleanYear = EEPROM.read(addrLastGrinderCleanYear);
  lastGrinderCleanMonth = EEPROM.read(addrLastGrinderCleanMonth);
  lastGrinderCleanDay = EEPROM.read(addrLastGrinderCleanDay);
  if (EEPROM.read(addrUnits) == 0) {
    units = 0;
  }
  else {
    units = 1;
  }
}

//Needs to be finished
//void checkService() {
//  DateTime now = rtc.now();
//  lastDescaleDate = (now.year() + now.month() + now.day());
//  if (nowDate - lastDescaleDate >= 180) {           //if the machine hasn't been descaled for 6 months (180 days) or longer.
//    descaleErrorCode = true;
//  }
//  lastBackFlushDate = (year + month + day);
//  if (nowDate - lastBackFlushDate >= 14) {          //if the machine hasn't been deback-flushed for 2 weeks (14 days) or longer.
//    backFlushErrorCode = true;
//  }
//  lastGrinderCleanDate = (year + month + day);
//  if (nowDate - lastGrinderCleanDate >= 180) {    //if the grinder hasn't been cleaned for 6 months (180 days) or longer.
//    grinderCleanErrorCode = true;
//  }
//  if (backFlushErrorCode == true) {
//    lcd.setCursor(0, 0);
//    lcd.print("Back-flushing is");
//    lcd.setCursor(0, 1);
//    lcd.print("recommended!    ");
//    delay(5000);
//  }
//  if (descaleErrorCode == true) {
//    lcd.setCursor(0, 0);
//    lcd.print("Descaling is    ");
//    lcd.setCursor(0, 1);
//    lcd.print("recommended!    ");
//    delay(5000);
//  }
//  if (grinderCleanErrorCode == true) {
//    lcd.setCursor(0, 0);
//    lcd.print("Grinder needs to");
//    lcd.setCursor(0, 1);
//    lcd.print("be cleaned!     ");
//    delay(5000);
//  }
//}

void fadeLed (int led1, int led2, int led3) {
  for (int i = 0; i <= 8 * 51; i ++) {
    analogWrite(led1, brightness);
    analogWrite(led2, brightness);
    analogWrite(led3, brightness);
    brightness += fadeStep;
    if (brightness <= 0 || brightness >= 255) {
      fadeStep = -fadeStep;
    }
    delay(15);
  }
}

