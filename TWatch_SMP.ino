#include "config.h"
#include <soc/rtc.h>
#include <sstream>
#include <math.h>
#include <time.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <WiFi.h>
const char* ssid = "Redmi Note 8 Pro";
const char* password = "4e9df9fd60bc";

const char* ntpServer = "europe.pool.ntp.org";
const long  gmtOffset_sec = -3600;       //UTC+2
const int   daylightOffset_sec = 3600;

TTGOClass *ttgo;
AXP20X_Class *power;
TFT_eSPI *tft;
BMA *sensor;

BluetoothSerial SerialBT;

bool IRQ = false;
bool collapse;
uint32_t nr_of_steps = 0;
RTC_DATA_ATTR uint32_t steps_perm = 0;  //steps after sleep

uint32_t currentMillis;   //time until sleep
uint32_t startMillis;

uint32_t targetTime = 0;       //1 second update clock

uint8_t hh, mm, ss, mmonth, dday;
uint16_t yyear;


void displayTime()
{
    uint8_t xcolon = 0;
    uint8_t xpos = 10;
    uint8_t ypos = 90;

    RTC_Date tnow = ttgo->rtc->getDateTime();

    hh = tnow.hour;
    mm = tnow.minute;
    ss = tnow.second;
    dday = tnow.day;
    mmonth = tnow.month;
    yyear = tnow.year;

    ttgo->tft->setTextSize(1);
    ttgo->tft->setTextColor(TFT_SKYBLUE, TFT_BLACK);

    if (hh < 10) xpos += ttgo->tft->drawChar('0', xpos, ypos, 7);
    xpos += ttgo->tft->drawNumber(hh, xpos, ypos, 7);

    xcolon = xpos + 3;
    xpos += ttgo->tft->drawChar(':', xcolon, ypos, 7);

    if (mm < 10) xpos += ttgo->tft->drawChar('0', xpos, ypos, 7);
    xpos += ttgo->tft->drawNumber(mm, xpos, ypos, 7);
        
    xcolon = xpos + 3;
    xpos += ttgo->tft->drawChar(':', xcolon, ypos, 7);

    if (ss < 10) xpos += ttgo->tft->drawChar('0', xpos, ypos, 7);
    xpos += ttgo->tft->drawNumber(ss, xpos, ypos, 7);

    ttgo->tft->setTextSize(2);
    ttgo->tft->setCursor(2, 0);

    ttgo->tft->print(dday);
    ttgo->tft->print("-");
    ttgo->tft->print(mmonth);
    ttgo->tft->print("-");
    ttgo->tft->print(yyear);

    ttgo->tft->setCursor(139, 0);
    int per = ttgo->power->getBattPercentage();
    ttgo->tft->print("Per:");
    ttgo->tft->print(per);
    ttgo->tft->print("%");
    
}

const int maxApp = 5;
String appName[maxApp] = {"Clock", "Pedometer", "Set WiFi Time", "Sleep", "Simulate Alert"};

uint8_t modeMenu()
{
    int mSelect = 0;
    int16_t x, y;

    boolean exitMenu = false;

    setMenuDisplay(0);

    while (!exitMenu) {
        if (ttgo->getTouch(x, y)) {

            while (ttgo->getTouch(x, y)) {}

            if (y >= 160) {
                mSelect += 1;
                if (mSelect == maxApp) mSelect = 0;
                setMenuDisplay(mSelect);
            }

            if (y <= 80) {
                mSelect -= 1;
                if (mSelect < 0) mSelect = maxApp - 1;
                setMenuDisplay(mSelect);
            }
            if (y > 80 && y < 160) {
                exitMenu = true;
            }
        }
      checkIR();
      if(collapse) {ttgo->tft->fillScreen(TFT_BLACK); return 0;}
    }
    ttgo->tft->fillScreen(TFT_BLACK);
    return mSelect;
}

void setMenuDisplay(int mSel)
{
    int curSel = 0;

    ttgo->tft->fillScreen(0xAFE5);
    ttgo->tft->fillRect(0, 80, 240, 80, TFT_BLACK);

    if (mSel == 0) curSel = maxApp - 1;
    else curSel = mSel - 1;
    
    ttgo->tft->setTextSize(2);
    ttgo->tft->setTextColor(0x0011);
    ttgo->tft->setCursor(50, 30);
    ttgo->tft->println(appName[curSel]);

    ttgo->tft->setTextColor(TFT_PINK);
    ttgo->tft->setCursor(40, 110);
    ttgo->tft->println(appName[mSel]);

    if (mSel == maxApp - 1) curSel = 0;
    else curSel = mSel + 1;
    ttgo->tft->setTextColor(0x0011);
    ttgo->tft->setCursor(50, 190);
    ttgo->tft->print(appName[curSel]);
}

void appPedometer()
{
  int16_t x, y;
  uint32_t steps_prev = nr_of_steps;
  while(1){
    ttgo->tft->fillScreen(TFT_BLACK);
    ttgo->tft->setTextColor(TFT_GREENYELLOW, TFT_BLACK);
    ttgo->tft->drawString("NR OF STEPS", 40, 30, 2);

    ttgo->tft->setTextColor(TFT_SILVER);
    tft->setCursor(110, 70);
    tft->println(nr_of_steps);

    ttgo->tft->fillRect(0, 170, 105, 70, TFT_OLIVE);
    ttgo->tft->setTextSize(2);
    ttgo->tft->setCursor(21, 193);
    ttgo->tft->println("RESET");

    ttgo->tft->fillRect(135, 170, 105, 70, TFT_BLUE);
    ttgo->tft->setTextSize(2);
    ttgo->tft->setCursor(160, 185);
    ttgo->tft->println("SEND");
    ttgo->tft->setCursor(138, 205);
    ttgo->tft->println("TO PHONE");

    while (!ttgo->getTouch(x, y)) {
      checkIR();
      if(collapse) {ttgo->tft->fillScreen(TFT_BLACK); return;}
      if(nr_of_steps > steps_prev){
        ttgo->tft->fillRect(109, 69, 101, 21, TFT_BLACK);
        tft->setCursor(110, 70);
        tft->println(nr_of_steps);
        steps_prev = nr_of_steps;
      }
    }
    while (ttgo->getTouch(x, y)) {}

    if (y > 170) {
      if (x < 105){
        sensor->resetStepCounter();
        nr_of_steps = 0;
      }
      else if (x > 135){
        SerialBT.print("STEPS: ");
        SerialBT.println(nr_of_steps);
      }
    }
    else
      break;
  }
  ttgo->tft->fillScreen(TFT_BLACK);
}

void setup()
{
    ttgo = TTGOClass::getWatch();
    ttgo->begin();
    ttgo->openBL();
    SerialBT.begin("TWatch");
    
    sensor = ttgo->bma;
    tft = ttgo->tft;
    power = ttgo->power;
    nr_of_steps = steps_perm;

    ttgo->tft->setTextFont(1);
    ttgo->tft->fillScreen(TFT_BLACK);
    ttgo->tft->setTextColor(TFT_SKYBLUE, TFT_BLACK);

    //ttgo->rtc->check();
    ttgo->rtc->syncToSystem();

    displayTime();

    //Acfg cfg;
    //cfg.odr = BMA4_OUTPUT_DATA_RATE_50HZ;   //100Hz
    //cfg.range = BMA4_ACCEL_RANGE_4G;    //2G  
    //cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    //cfg.perf_mode = BMA4_CONTINUOUS_MODE;
    //sensor->accelConfig(cfg);
    sensor->enableAccel();

    sensor->enableFeature(BMA423_STEP_CNTR, true);
    sensor->enableFeature(BMA423_WAKEUP, true);
    sensor->enableFeature(BMA423_TILT, true);

    pinMode(BMA423_INT1, INPUT);
    attachInterrupt(BMA423_INT1, [] {IRQ = 1;}, RISING);

    sensor->enableStepCountInterrupt();
    sensor->enableWakeupInterrupt();
    sensor->enableTiltInterrupt();

    power->adc1Enable(AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1, true);
    power->setPowerOutPut(AXP202_LDO3, false);   //AUDIO
    power->setPowerOutPut(AXP202_EXTEN, false);
    power->setPowerOutPut(AXP202_DCDC2, false);
    power->setPowerOutPut(AXP202_LDO4, false);
    
    esp_sleep_enable_ext1_wakeup(GPIO_SEL_39, ESP_EXT1_WAKEUP_ANY_HIGH);

    startMillis = millis();  
}

void goToSleep()
{
    steps_perm = nr_of_steps;
    ttgo->displaySleep();
    //sensor->enableStepCountInterrupt(false);   //Se trezeste doar la Tilt
    esp_deep_sleep_start();
}

void getWifiTime() {
  uint32_t crtTime;
  uint32_t startTime = millis();

  WiFi.begin(ssid, password);
  startTime = millis();

  while (WiFi.status() != WL_CONNECTED){
      crtTime = millis() - startTime;
      if(crtTime > 10000){
        ttgo->tft->fillScreen(TFT_BLACK);
        return;
      }
  }
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  getLocalTime(&timeinfo);

  ttgo->rtc->setDateTime(timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  //ttgo->rtc->check();
  //ttgo->rtc->syncToSystem();

  ttgo->tft->fillScreen(TFT_BLACK);
}


void loop()
{
    if (targetTime < millis()) {
        targetTime = millis() + 1000;
        displayTime();
    }

    currentMillis = millis() - startMillis;
    if(currentMillis > 20000) {
      goToSleep();
    }

    int16_t x, y;
    if (ttgo->getTouch(x, y)) {
        while (ttgo->getTouch(x, y)) {}
        switch (modeMenu()) {
        case 0:
            break;
        case 1:
            appPedometer();
            break;
        case 2:
            getWifiTime();
            break;
        case 3:
            goToSleep();
            break;
        case 4:
            enterAlertState();
            break;
        }
        displayTime();
        startMillis = millis();
    }
    checkIR();
}

void bluetoothTime(String timeBT)
{
    RTC_Date tnow = ttgo->rtc->getDateTime();

    dday = tnow.day;
    mmonth = tnow.month;
    yyear = tnow.year;

    String word;

    int index = timeBT.indexOf(':');
    word = timeBT.substring(0, index);
    timeBT = timeBT.substring(index+1);
    hh = atoi(word.c_str());
    word = timeBT.substring(0, index);
    timeBT = timeBT.substring(index+1);
    mm = atoi(word.c_str());
    word = timeBT.substring(0, index);
    ss = atoi(word.c_str());

    if ( (hh < 24 && hh >=0) && (mm < 60 && mm >= 0) && (ss < 60 && ss >= 0)){
      ttgo->rtc->setDateTime(yyear, mmonth, dday, hh, mm, ss);
      SerialBT.println("Time set successfully");

    }
    else
      SerialBT.println("Wrong time format");
}

void emergency(){
  SerialBT.println("Dialing 911");
  startMillis = millis();
}

void enterAlertState() {
  uint32_t crtTime = 0;
  uint32_t startTime = millis();

  ttgo->tft->fillScreen(TFT_RED);
  ttgo->tft->setTextSize(2);
  ttgo->tft->setTextColor(TFT_WHITE, TFT_RED);
  ttgo->tft->setCursor(20, 20);
  ttgo->tft->print("Collapse Detected!");
  ttgo->tft->setCursor(20, 60);
  ttgo->tft->print("Double click or");
  ttgo->tft->setCursor(25, 80);
  ttgo->tft->print(" Tilt to exit");
  ttgo->tft->setCursor(30, 120);
  ttgo->tft->print("Time remaining");
  ttgo->tft->setCursor(25, 140);
  ttgo->tft->print("until emergency:");
  ttgo->tft->setTextSize(3);
  
  while(1){
    ttgo->tft->setCursor(110, 180);
    if (crtTime%999==0){
      ttgo->tft->fillRect(110, 180, 180, 220, TFT_RED);
      ttgo->tft->print((11000-crtTime)/1000+1);
    }
    
    crtTime = millis() - startTime;
    if(crtTime > 12000) {
      emergency();
      startMillis = millis();
      ttgo->tft->fillScreen(TFT_BLACK);
      return;    
    }

    if(IRQ){
        IRQ = false;
        bool  rlst;
        do {
            rlst =  sensor->readInterrupt();
        } while (!rlst);

        if (sensor->isTilt() || sensor->isDoubleClick()) {
          ttgo->tft->fillScreen(TFT_BLACK);
          startMillis = millis();
          return;
        }
    } 
  }
}

void checkIR(){
  Accel acc;
  uint16_t acceleration;
  bool res;
  collapse = false;
  
  if (IRQ) {
        IRQ = false;
        bool  rlst;
        do {
            rlst =  sensor->readInterrupt();
            res = sensor->getAccel(acc);
            if(res){
              acceleration = sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z);
              if(acceleration>1750){
                collapse = true;
                break;
              }
            }
        } while (!rlst);

        if (sensor->isStepCounter()) {
            nr_of_steps = sensor->getCounter() + steps_perm;
        }
  }

  if (collapse){
    uint32_t crtTime;
    uint32_t startTime = millis();
    while(1){
      crtTime = millis() - startTime;

      if(crtTime > 7000) {
        enterAlertState();
        ttgo->tft->fillScreen(TFT_BLACK);
        break;
      }

      if (IRQ) {
        IRQ = false;
        bool  rlst;
        do {
            rlst = sensor->readInterrupt();
        } while (!rlst);
        if (sensor->isStepCounter() || sensor->isTilt()){
          ttgo->tft->fillScreen(TFT_BLACK);
          startMillis = millis();
          return;
        }
      }
    }
  }

  if(SerialBT.available()){
    String timeBT=SerialBT.readString();
    String command="";

    int index = timeBT.indexOf(':');
    if (index != -1)
    {
      command = timeBT.substring(0, index);
      timeBT = timeBT.substring(index+1);
    }
  
    if(command == "Time")
      bluetoothTime(timeBT);
  }
}
