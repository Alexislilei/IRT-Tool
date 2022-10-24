#if !defined(ARDUINO_ARCH_RP2040)
  #error For RP2040 only
#endif

#if defined(ARDUINO_ARCH_MBED)
  
  #define PIN_SD_MOSI       PIN_SPI_MOSI
  #define PIN_SD_MISO       PIN_SPI_MISO
  #define PIN_SD_SCK        PIN_SPI_SCK
  #define PIN_SD_SS         PIN_SPI_SS

#else

  #define PIN_SD_MOSI       PIN_SPI0_MOSI
  #define PIN_SD_MISO       PIN_SPI0_MISO
  #define PIN_SD_SCK        PIN_SPI0_SCK
  #define PIN_SD_SS         PIN_SPI0_SS
  
#endif

#define _RP2040_SD_LOGLEVEL_       0

#include <RP2040_SD.h>

#define PIN_LED_POW 1
#define PIN_LED_REC 2


//=========================================================================
// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 22     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

//=========================================================================== 
// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include "RTClib.h"
RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//===========================================================================
#define USE_NO_LCD

#include <Arduino.h>

#include "PinDefinitionsAndMore.h"

//#include "pico/stdlib.h"
//#include "hardware/adc.h"

/*
 * Set input pin and output pin definitions etc.
 */
#define IRMP_PROTOCOL_NAMES              1 // Enable protocol number mapping to protocol strings - needs some program memory ~ 420 bytes here
#define IRMP_USE_COMPLETE_CALLBACK       1 // Enable callback functionality
//#define NO_LED_FEEDBACK_CODE   // Activate this if you want to suppress LED feedback or if you do not have a LED. This saves 14 bytes code and 2 clock cycles per interrupt.

#if __SIZEOF_INT__ == 4
#define F_INTERRUPTS                     20000 // Instead of default 15000 to support LEGO + RCMM protocols
#else
//#define F_INTERRUPTS                     20000 // Instead of default 15000 to support LEGO + RCMM protocols, but this in turn disables PENTAX and GREE protocols :-(
//#define IRMP_32_BIT                       1 // This enables MERLIN protocol, but decreases performance for AVR.
#endif

#include <irmpSelectAllProtocols.h>  // This enables all possible protocols
//#define IRMP_SUPPORT_SIEMENS_PROTOCOL 1

/*
 * After setting the definitions we can include the code and compile it.
 */
#include <irmp.hpp>

IRMP_DATA irmp_data;
IRMP_DATA irRevBuf[4];

uint32_t keytime[4] = {0,0,0,0};
bool irPasswordFlag = false;



#if defined(__AVR__) && !(defined(__AVR_ATmega4809__) || defined(__AVR_ATtiny1616__)  || defined(__AVR_ATtiny3216__) || defined(__AVR_ATtiny3217__))
// For cyclically display of VCC
#include "ADCUtils.hpp"
#define MILLIS_BETWEEN_VOLTAGE_PRINT 5000
#endif

void handleReceivedIRData();
//void irmp_result_print_LCD();

bool volatile sIRMPDataAvailable = false;

//===================
 const char *tProtocolStringPtr;
 uint8_t repeat_count = 0;

 

//----------------------------------------
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

//unsigned long targetTime = 0; // Used for testing draw times

//-----------------------------------------------
uint32_t targetTime = 0;       // for next 1 second timeout


//----------------------------------------------------
#define LOCAL_KEY_NOPRESS 0
#define LOCAL_KEY_OK 1
#define LOCAL_KEY_UP 2
#define LOCAL_KEY_DOWN 3
#define LOCAL_KEY_RIGHT 4
#define LOCAL_KEY_LEFT 5
#define LOCAL_KEY_OK_HOLD  0x81
#define LOCAL_KEY_UP_HOLD  0x82
#define LOCAL_KEY_DOWN_HOLD 0x83
#define LOCAL_KEY_RIGHT_HOLD 0x84
#define LOCAL_KEY_LEFT_HOLD 0x85


const int keyLevel[]={160,420,610,780,940};

uint16_t keyValue = 1024;
uint8_t localKeyEvent = LOCAL_KEY_NOPRESS;

//---------------------------------------------------------
//uint32_t 

byte omm = 99;
byte oss = 99;
bool initial = 1;
byte xcolon = 0;
unsigned int colour = 0;

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

uint8_t count500ms = 0;

uint8_t hh=conv2d(__TIME__), mm=conv2d(__TIME__+3), ss=conv2d(__TIME__+6);  // Get H, M, S from compile time
uint16_t now_year = 2022;
uint8_t now_month= 1,now_day = 1;
uint8_t sethour,setminute,setsecond,setdate,setmonth;
uint16_t setyear;
bool bUpdateDate = true;
String dateStamp("2022-10-03");
String irRecFilename("IRnoDate.txt");
String TempRecFilename("TnoDate.txt");

DateTime now(F(__DATE__), F(__TIME__));


//---RTC---------------------------------------------------------------

const byte interruptPin = 15;
void setup_1S_Timer_exInterrupt(void);
void RTC_1S_process(void);
//------------------------------------------------------------------------------------

//----UI-----------------
#define PAGE_TIME 0
#define PAGE_IR 1
#define PAGE_SETUP 2
#define PAGE_MAX 3

#define NO_ERR 0
#define ERR_RTC 1
#define ERR_DHT 2
#define ERR_TFT 3
#define ERR_SD 4
#define ERR_OTHERS 5
const String sErrorCode[6]={
  "NO ERROR",
  "RTC ERROR",
  "DHT ERROR",
  "TFT ERROR",
  "SD ERROR",
  "OTHERS ERROR"
};

uint8_t error_code = 0;

byte display_page = PAGE_TIME;
byte display_last_page = PAGE_TIME;
String irDataQueue[5]={"-","-","-","-","-"};
String sAddress="",sCommand="";
bool display_fresh = true;
uint8_t irPageDelay = 0;

void display_time(byte xpos,byte ypos,byte font);
void display_ir_page(void);
void display_time_page(void);
void display_setup_page(void);

//float temp=0;

uint8_t setuppage1_focus = 0;
void set_item_color(uint8_t item_number,uint8_t focus_number);
void datetime_adjust(bool bIncrease,uint8_t item);

//-------------------------------------
void DataLogger(String sFilename,String sData,bool timeOn);
void printDirectory(RP2040_SDLib::File dir, int numTabs);
void printFromSDtoSerial(void);

RP2040_SDLib::File root;

//------------------------------------------------------------------------------------
const int setupKeyPin = 26;     // the number of the pushbutton pin
const int printButtonPin = 14;  //press this button to print data to serial

// variables will change:
//uint8_t buttonState = 0;         // variable for reading the pushbutton status
int setupButtonPressCount = 0;
int printButtonPressCount = 0;
void init_button(void);

//------------------------------------------------------------------------------------
float DHT_Temperature = 0;
float DHT_Humidity = 0;

const unsigned int TRecInterval[] ={0xffff,5,10,30,60,120,300,600};//interval of temperature record. if 0xff means off
#define TREC_ITEMS 8
unsigned int trecChoose = 0;  
uint16_t nextTimeToRecTemp = 0;

void init_DHT11(void)
{
    
    dht.begin();
    Serial.println(F("DHTxx Unified Sensor Example"));
    // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
    Serial.println(F("------------------------------------"));
    // Set delay between sensor readings based on sensor details.
    delayMS = sensor.min_delay / 1000;
}
void read_DHT(void)
{
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    }
    else {
      DHT_Temperature = event.temperature;  //add
//      Serial.print(F("Temperature: "));
//      Serial.print(event.temperature);
//      Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
    else {
      DHT_Humidity = event.relative_humidity; //add
//      Serial.print(F("Humidity: "));
//      Serial.print(event.relative_humidity);
//      Serial.println(F("%"));
    }
}

void init_DS1307(void)
{
    Wire.setSDA(20);
    Wire.setSCL(21);
    
    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    //Serial.flush();
    //while (1) delay(10);
    error_code = ERR_RTC;
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
  Serial.print("time:");
  Serial.println(F(__TIME__));
  rtc.writeSqwPinMode(DS1307_SquareWave1HZ);
  
  /*
   * enum   Ds1307SqwPinMode {
  DS1307_OFF = 0x00, DS1307_ON = 0x80, DS1307_SquareWave1HZ = 0x10, DS1307_SquareWave4kHz = 0x11,
  DS1307_SquareWave8kHz = 0x12, DS1307_SquareWave32kHz = 0x13
}
  */
}




void setup_1S_Timer_exInterrupt() 
{
    
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), RTC_1S_process, CHANGE); //FALLING,RISING,LOW,HIGH
}

void RTC_1S_process()
{
    if(count500ms&0x01)
    {
      ss++;              // Advance second
      if (ss==60) {
        ss=0;
        omm = mm;
        mm++;            // Advance minute
        if(mm>59) {
          mm=0;
          hh++;          // Advance hour
          if (hh>23) {
            hh=0;
          }
        }
      }
      if(nextTimeToRecTemp)
      {
          nextTimeToRecTemp--;
          if(nextTimeToRecTemp == 0)
            nextTimeToRecTemp=TRecInterval[trecChoose];
      }      
    }
    count500ms++;  
    if(hh==0&&mm==0&&ss==0)
      bUpdateDate = true;
//    else
//      bUpdateDate = false;


}

void update_date_from1307(void)
{
    DateTime newDate = rtc.now();
    now_year = newDate.year();
    now_month = newDate.month();
    now_day = newDate.day();
    Serial.println("update date:"+String(now_year,DEC)+"-"+String(now_month,DEC)+"-"+String(now_day,DEC));
    dateStamp = newDate.timestamp(newDate.TIMESTAMP_DATE);
    irRecFilename = String("IR")+ dateStamp + String(".txt");    //to get new filename everyday. example:IR221323.txt
    irRecFilename.remove(9,1);
    irRecFilename.remove(6,1);
    irRecFilename.remove(2,2);    
    TempRecFilename = irRecFilename;
    TempRecFilename.remove(0,2);
    TempRecFilename = "T-" + TempRecFilename;
}

//--------------------------------------------------------------
void init_button(void)
{
    pinMode(setupKeyPin, INPUT_PULLUP);
    pinMode(printButtonPin,INPUT_PULLUP);
}
void init_IRMP(void)
{
    irmp_init();
    irmp_irsnd_LEDFeedback(true); // Enable receive signal feedback at LED_BUILTIN
    irmp_register_complete_callback_function(&handleReceivedIRData);
  
    Serial.print(F("Ready to receive IR signals of protocols: "));
    irmp_print_active_protocols (&Serial);
    Serial.println(F("at pin " STR(IRMP_INPUT_PIN)));
}

void init_TFT(void)
{
    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
}

void init_SDCard(void)
{
    #if defined(ARDUINO_ARCH_MBED)
    Serial.print("Starting SD Card ReadWrite on MBED ");
  #else
    Serial.print("Starting SD Card ReadWrite on ");
  #endif
    
    Serial.println(BOARD_NAME);
    Serial.println(RP2040_SD_VERSION);
    
    Serial.print("Initializing SD card with SS = ");  Serial.println(PIN_SD_SS);
    Serial.print("SCK = ");   Serial.println(PIN_SD_SCK);
    Serial.print("MOSI = ");  Serial.println(PIN_SD_MOSI);
    Serial.print("MISO = ");  Serial.println(PIN_SD_MISO);

    if (!SD.begin(PIN_SD_SS)) 
    {
      Serial.println("Initialization failed!");
      return;
    }
//    root = SD.open("/");
//    //printDirectory(root, 0);
//    //root.close();
//    PrintFilesInDirectory(root,0);
    Serial.println("Initialization done.");
}
void setup()
{
    pinMode(PIN_LED_REC, OUTPUT);
    digitalWrite(PIN_LED_REC,HIGH );
    pinMode(PIN_LED_POW, OUTPUT);
    digitalWrite(PIN_LED_POW,HIGH );
    
    
    Serial.begin(115200);
    
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217) \
    || defined(__AVR_ATtiny1616__)  || defined(__AVR_ATtiny3216__) || defined(__AVR_ATtiny3217__)
    delay(4000); // To be able to connect Serial monitor after reset or power on and before first printout
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRMP));

    //======rp2040 built-in temperature sensor================
//    adc_init();
//    adc_set_temp_sensor_enabled(true);
//    adc_select_input(4);
    //===button==========================================
    init_button();
    //=======IR===========================================    
    init_IRMP();
    
    /*
    irmp_init();
    irmp_irsnd_LEDFeedback(true); // Enable receive signal feedback at LED_BUILTIN
    irmp_register_complete_callback_function(&handleReceivedIRData);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    irmp_print_active_protocols (&Serial);
    Serial.println(F("at pin " STR(IRMP_INPUT_PIN)));*/
    //======DHT11============================================
    init_DHT11();
    //=======TFT===========================================================
    init_TFT();
    //------------------------------------
    
    init_DS1307();
    //DateTime 
    now = rtc.now();    
    hh = now.hour();
    mm = now.minute();
    ss = now.second();
    now_year = now.year();
    now_month = now.month();
    now_day = now.day();
    
    setup_1S_Timer_exInterrupt();
    
    //=======SD CARD===========================================================
    init_SDCard();
    pinMode(PIN_LED_REC, OUTPUT);
    digitalWrite(PIN_LED_REC,LOW );
    
}

void loop()
{
    //targetTime = millis() + 1000; 
    //if (digitalRead(setupKeyPin) == LOW)
    keyValue = analogRead(setupKeyPin);
    if(keyValue < keyLevel[4])
    {
      irPageDelay = 10;
      if(setupButtonPressCount < 0x7fff)
        setupButtonPressCount ++;  
      else
        setupButtonPressCount = 0xff;
    }
    else
    {
      setupButtonPressCount = 0;
      localKeyEvent = 0;
    }
    if(setupButtonPressCount >=50)
    {
      
      
      if(setupButtonPressCount==50) //first press
      {
          //Serial.println("first press times:"+String(setupButtonPressCount,DEC));
          //setupButtonPressCount = 1000;
          
          for(int i = 0;i<5;i++)
          {
            if(keyValue<keyLevel[i])
            {
              localKeyEvent = i+1;
              break;
            }
          }
      }
      if(setupButtonPressCount>127)
      {
//          Serial.println("hold press times:"+String(setupButtonPressCount,DEC));

          if(setupButtonPressCount&0b00000011 == 0)          
          {
              for(int i = 0;i<5;i++)
              {
                if(keyValue<keyLevel[i])
                {
                  localKeyEvent = i+1;
                  break;
                }
              }
          }
      }    
    }
    if(localKeyEvent)
    {
      if(localKeyEvent == LOCAL_KEY_OK)
      {    
          if(display_page == PAGE_TIME)
          {
            irPageDelay = 10;
            display_page = PAGE_IR;
            display_ir_page();
          }
          else if(display_page == PAGE_IR)
          {
            irPageDelay = 10;
            display_page = PAGE_SETUP;
            display_setup_page();
          }
          else if(display_page == PAGE_SETUP)
          {
            display_page = PAGE_TIME;
            display_time_page();
            irPageDelay = 0;
          } 
          //localKeyEvent = LOCAL_KEY_NOPRESS;
      }
      //============
      if(display_page ==PAGE_SETUP)
      {
        switch(localKeyEvent&0x0F)
        {
          case LOCAL_KEY_UP:  //up
            if(setuppage1_focus) 
              setuppage1_focus--;
              
              display_setup_page();
            break;
          case LOCAL_KEY_DOWN:  //down
            if(setuppage1_focus<7) 
              setuppage1_focus++;
              display_setup_page();
            break;
          case LOCAL_KEY_RIGHT:  //right
            datetime_adjust(true,setuppage1_focus); //true means increas,false means decreas
            display_setup_page();
            //bUpdateDate=true;
            break;
          case LOCAL_KEY_LEFT:  //left
            datetime_adjust(false,setuppage1_focus);
            display_setup_page();
            //bUpdateDate=true;
            break;
        }
      }
      //===============
      localKeyEvent = LOCAL_KEY_NOPRESS;
    }
    
    if (digitalRead(printButtonPin) == LOW)
    {
      if(printButtonPressCount < 0x7fff)
        printButtonPressCount ++;   
    }
    else
    {
      printButtonPressCount = 0;
    }
    if(printButtonPressCount >10)
    {
        if(printButtonPressCount<1000)
        {
            printButtonPressCount = 1000;
    //        irPageDelay = 10;
            printFromSDtoSerial();
//            root = SD.open("/");
//            tft.fillScreen(TFT_BLACK);
//            tft.setTextColor(TFT_YELLOW,TFT_BLACK);
//            tft.setCursor(5, 30,2);
//            tft.println("Printing files...");
//            tft.println("From SD to serial");
//            PrintFilesInDirectory(root,0);
//            tft.fillScreen(TFT_BLACK);
//            display_page = 99;
//            display_time_page();
//            display_page = PAGE_TIME;          
        }

    }

      if(bUpdateDate)
      {
        Serial.println("date update from 1307");
        update_date_from1307();
        bUpdateDate = false;
      }

      
      if (ss!=oss || initial) 
      {
        initial = 0;
        oss = ss;
        //read temperature every 1s
//        uint16_t raw = adc_read();
//        const float conversion_factor = 3.3f / (1<<12);
//        float result = raw * conversion_factor;
//        temp = 27 - (result -0.706)/0.001721;

        read_DHT();
        if(trecChoose)
          digitalWrite(PIN_LED_REC,HIGH);
        else
          digitalWrite(PIN_LED_REC,LOW);
        if(nextTimeToRecTemp == TRecInterval[trecChoose])
        {
            String d = "T: "+String(DHT_Temperature,1)+"C,\tH: "+String(DHT_Humidity,1)+"%";
            DataLogger(TempRecFilename,d,true);     
        }
        
        if(irPageDelay)
        {
          irPageDelay--;
          Serial.println(irPageDelay);
          if(!irPageDelay){
            display_page = PAGE_TIME;
            //tft.fillScreen(TFT_BLACK);
//            omm=99;
//            display_time(5,15,7);
          }

          
        }
        
        switch(display_page)
          {
            case PAGE_TIME:
              //tft.setTextColor(TFT_GREEN, TFT_BLACK);
              display_time_page();
              break;
            case PAGE_IR:
              tft.setTextColor(TFT_BLACK, TFT_WHITE);
              display_time(80,5,2);
              break;
            case PAGE_SETUP:
              display_setup_page();
              break;
            default:
              break;
          } 
      }
      
//    }    

    if (sIRMPDataAvailable)
    {
        sIRMPDataAvailable = false;

        digitalWrite(PIN_LED_POW,LOW);
        
        if(display_page !=PAGE_SETUP)
            display_page = PAGE_IR;
        irPageDelay = 10;
        /*
         * Serial output
         * takes 2 milliseconds at 115200
         */
        irmp_result_print(&irmp_data);
        
        sAddress = String(irmp_data.address,HEX);
        sAddress.toUpperCase();

        
        if(irmp_data.command<0x10)  {sCommand = "0"+String(irmp_data.command,HEX);}
        else  {sCommand = String(irmp_data.command,HEX);}

        sCommand.toUpperCase();
        

        if (irmp_data.flags & IRMP_FLAG_REPETITION)
        {
            if(repeat_count<255)
              repeat_count++;
            irDataQueue[0]="A:0x"+sAddress+" C:0x"+sCommand+" R:"+String(repeat_count)+" \t"+irmp_protocol_names[irmp_data.protocol]+"     ";
            //DataLogger("ir_rec.txt",irDataQueue[0],true);
        }
        else
        {
            repeat_count = 0;
            
            for(int i=0;i<4;i++)
            {
              irDataQueue[4-i]=irDataQueue[3-i];
            }
             
            irDataQueue[0]="A:0x"+sAddress +" C:0x"+sCommand+" R:0 \t"+irmp_protocol_names[irmp_data.protocol]+"     ";

            // to detect password to enter into settings menu
            for(int k=3;k>0;k--)
            {
              irRevBuf[k].address = irRevBuf[k-1].address;
              irRevBuf[k].command = irRevBuf[k-1].command;
            }
            irRevBuf[0].address = irmp_data.address;
            irRevBuf[0].command = irmp_data.command;
            
            if(irRevBuf[0].address == 0xbf00 && irRevBuf[0].command == 9 &&
                irRevBuf[1].address == 0xbf00 && irRevBuf[1].command == 6 &&
                irRevBuf[2].address == 0xbf00 && irRevBuf[2].command == 9 &&
                irRevBuf[3].address == 0xbf00 && irRevBuf[3].command == 1)
            {
              irPasswordFlag = true;
            }
            else
            {
              irPasswordFlag = false;
              
            }   
        }
        
        if(display_page == PAGE_TIME || display_page == PAGE_IR)
          DataLogger(irRecFilename,irDataQueue[0],true);
        
        if(display_page !=PAGE_SETUP)
        
          display_ir_page();
        
        if(irPasswordFlag)
        {
          irPasswordFlag = false;
          display_page = PAGE_SETUP;
          display_setup_page();
        }
        
        if(display_page ==PAGE_SETUP && irmp_data.address == 0xbf00 &&(!(repeat_count&0x3)))
        {
          
          //Serial.println(irmp_data.command);
          switch(irmp_data.command)
          {
            case 0x16:  //up
              if(setuppage1_focus) 
                setuppage1_focus--;
                
                display_setup_page();
              break;
            case 0x17:  //down
              if(setuppage1_focus<7) 
                setuppage1_focus++;
                display_setup_page();
              break;
            case 0x18:  //right
              datetime_adjust(true,setuppage1_focus); //true means increas,false means decreas
              display_setup_page();
              //bUpdateDate=true;
              break;
            case 0x19:  //left
              datetime_adjust(false,setuppage1_focus);
              display_setup_page();
              //bUpdateDate=true;
              break;
                         
          }
//          Serial.print("focus:");
//          Serial.println(setuppage1_focus);

        }
    }
    else{
      //repeat_count = 0;
      digitalWrite(PIN_LED_POW,HIGH);
    }
    
    

}



/*
 * Here we know, that data is available.
 * Since this function is executed in Interrupt handler context, make it short and do not use delay() etc.
 * In order to enable other interrupts you can call interrupts() (enable interrupt again) after getting data.
 */
#if defined(ESP8266) || defined(ESP32)
void IRAM_ATTR handleReceivedIRData()
#else
void handleReceivedIRData()
#endif
{

#if defined(__AVR__) && defined(ADATE)
    // reset voltage display timer
    sMillisOfLastVoltagePrint = millis();
#endif

    /*
     * Just print the data to Serial and LCD
     */
    irmp_get_data(&irmp_data);
    sIRMPDataAvailable = true;
}

void display_time(byte xpos,byte ypos,byte font)
{    
    if(font!=7){
        tft.setCursor (xpos, ypos, font);
        //tft.print(__DATE__); // This uses the standard ADAFruit small font
        tft.print("  ");
        if(hh<10)tft.print("0");
        tft.print(hh);
        tft.print(":");
        if(mm<10)tft.print("0");
        tft.print(mm);
        tft.print(":");
        if(ss<10)tft.print("0");
        tft.print(ss);
    }
    else{
        // Update digital time
        if (omm != mm) { // Only redraw every minute to minimise flicker
          // Uncomment ONE of the next 2 lines, using the ghost image demonstrates text overlay as time is drawn over it
          //tft.setTextColor(0x39C4, TFT_BLACK);  // Leave a 7 segment ghost image, comment out next line!
          tft.setTextColor(TFT_BLACK, TFT_BLACK); // Set font colour to black to wipe image
          // Font 7 is to show a pseudo 7 segment display.
          // Font 7 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : .
          tft.drawString("88:88",xpos,ypos,7); // Overwrite the text to clear it
          //tft.setTextColor(0xFBE0); // Orange
          tft.setTextColor(TFT_GREEN);
          omm = mm;
    
          if (hh<10) xpos+= tft.drawChar('0',xpos,ypos,7);
          xpos+= tft.drawNumber(hh,xpos,ypos,7);
          xcolon=xpos;
          xpos+= tft.drawChar(':',xpos,ypos,7);
          if (mm<10) xpos+= tft.drawChar('0',xpos,ypos,7);
          tft.drawNumber(mm,xpos,ypos,7);
        }
    
        if (ss%2) { // Flash the colon
          tft.setTextColor(0x39C4, TFT_BLACK);
          xpos+= tft.drawChar(':',xcolon,ypos,7);
          tft.setTextColor(0xFBE0, TFT_BLACK);
        }
        else {
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.drawChar(':',xcolon,ypos,7);
        }
    }

      //tft.setTextColor(TFT_BLUE, TFT_BLACK);
      //tft.drawCentreString("It is windy",120,48,2); // Next size up font 2

      //tft.setTextColor(0xF81F, TFT_BLACK); // Pink
      //tft.drawCentreString("12.34",80,100,6); // Large font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 . : a p m
      
    return;
}


void display_time_page(void)
{
    //tft.fillScreen(TFT_BLACK);
    if(display_page != display_last_page) 
    {
      tft.fillScreen(TFT_BLACK);
      //tft.fillRect(0, 0, 160, 70, TFT_BLACK);
      display_last_page = display_page;
      omm=99;
    }
    tft.setTextColor(TFT_BLACK,TFT_WHITE);  
    tft.setCursor(0, 0,1);
    tft.print("                          ");
    tft.setCursor(0, 2,1);
    tft.print("        "+ dateStamp + "        ");
    
    display_time(5,12,7);
    
    if(!trecChoose)
    {
        tft.setTextColor(TFT_YELLOW,TFT_BLACK);
        tft.setCursor(15, 65,6);
        tft.print(String(DHT_Temperature,1));
        tft.setTextFont(4);
        tft.println(" C");
        tft.setCursor(110, 65,1);
        tft.println("o");
        tft.setCursor(20, 105,4);
        tft.print(String(DHT_Humidity,1));
        tft.setTextFont(4);
        tft.println("%");      
    }
    else
    {
        tft.setTextColor(TFT_YELLOW,TFT_BLACK);  
        tft.setCursor(5, 65,2);
        tft.println("Record every "+String(TRecInterval[trecChoose])+ "S   ");
        
        tft.println(" Next rec in " + String(nextTimeToRecTemp) + "S   ");

        if(TRecInterval[trecChoose]-nextTimeToRecTemp)
          tft.setTextColor(TFT_YELLOW,TFT_BLACK);
        else
          tft.setTextColor(TFT_BLACK,TFT_YELLOW);
        
        tft.setCursor(5,100,4);
        tft.print(String(DHT_Temperature,1));
        tft.setTextFont(4);
        tft.print(" C ");
        tft.print(String(DHT_Humidity,1));
        tft.setTextFont(4);
        tft.println("%");
                
        tft.setCursor(55,100,1);
        tft.println("o");
        
  
    }
    
/*    
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setCursor(5, 70,1);
    tft.println(irDataQueue[0]);
    tft.setCursor(5, 80,1);
    tft.println(irDataQueue[1]);
    tft.setCursor(5, 90,1);
    tft.println(irDataQueue[2]);
    tft.setCursor(5, 100,1);
    tft.println(irDataQueue[3]);
    tft.setCursor(5, 110,1);
    tft.println(irDataQueue[4]);
*/
}
void display_ir_page(void)
{
        
        if(display_page != display_last_page) 
        {
          tft.fillScreen(TFT_BLACK);
          //tft.fillRect(0, 0, 160, 70, TFT_BLACK);
          display_last_page = display_page;
        }
        
        tft.setCursor(2,5,2);
        tft.setTextColor(TFT_BLACK,TFT_WHITE);
        tft.print(dateStamp);
        display_time(80,5,2);
 
        
        tft.setTextColor(TFT_YELLOW,TFT_BLACK);
        tft.setCursor(10, 30,4);
        tft.print("                            ");
        tft.setCursor(10, 30,4);
        //tft.setTextSize(3);
        tft.print(sAddress);
        tft.print(" 0x"+sCommand);
        //tft.setTextSize(1);

        tft.setTextColor(TFT_WHITE,TFT_BLACK);
        for(int j=0;j<5;j++)
        {
          tft.setCursor(2, 70+j*10,1);
          tft.println(irDataQueue[j].substring(0,26));
        }

}

void set_item_color(uint8_t item_number,uint8_t focus_number)
{
    if(item_number == focus_number)
      tft.setTextColor(TFT_YELLOW,TFT_BLACK);
    else
      tft.setTextColor(TFT_GREEN,TFT_BLACK);
}

void datetime_adjust(bool bIncrease,uint8_t item)
{
    if(item==0) //year
    {
//    switch(item)
//    {
//      case 0:   //year
        uint16_t tempY;
        tempY = now_year;
        if(bIncrease)
        {
          tempY++;
        }
        else
        {
          if(tempY>2022)
            tempY--;
        }
        DateTime checkTime(tempY,now_month,now_day,hh,mm,ss);
        if( checkTime.isValid())
          now_year = tempY;
//        break;
//        bUpdateDate=true;
    } 
    if(item == 1)
    {
//      case 1: //month
        uint8_t tempM;
        tempM = now_month;
        if(bIncrease)
        {
          tempM++;
          if(tempM>12) tempM -= 12;
        }    
        else
        {
          tempM--;
          if(tempM==0) tempM = 12;
        }
        DateTime checkTime(now_year,tempM,now_day,hh,mm,ss);
        if( checkTime.isValid())
          now_month = tempM;
//        bUpdateDate=true;
    }
//        break;
    if(item==2)
    {
//      case 2: //day
        uint8_t tempD;
        tempD = now_day;
        if(bIncrease)
        {
          tempD++;
          //if(temp>31) temp -= 31;
        }    
        else
        {
          tempD--;
          //if(temp==0) temp = 31;
        }
        DateTime checkTime(now_year,now_month,tempD,hh,mm,ss);
        if( checkTime.isValid())
          now_day = tempD;
     //   break;
    }
    if(item==3)
    {
//      case 3: //hour
        if(bIncrease)
        {
          hh++;
          if(hh>23) hh = 0;
        }    
        else
        {
          if(hh==0) hh = 23;
          else hh--;      
        }
//        break;
    }
    if(item == 4)
    {
//      case 4: //miute
        if(bIncrease)
        {
          mm++;
          if(mm>59) mm = 0;
        }    
        else
        {
          if(mm==0) mm = 59;
          else mm--;      
        }
//        break;
    }
    if(item ==5)
    {
//      case 5: //second
        if(bIncrease)
        {
          ss++;
          if(ss>59) ss = 0;
        }    
        else
        {
          if(ss==0) ss = 59;
          else ss--;      
        }
//        break;
    }
    if(item < 6)
       bUpdateDate=true;
       
    if(item ==6)
    {
        if(bIncrease)
        {
          trecChoose++;
          if(trecChoose>(TREC_ITEMS-1)) trecChoose = 0;
        }    
        else
        {
          if(trecChoose==0) trecChoose = TREC_ITEMS-1;
          else trecChoose--;      
        }
        nextTimeToRecTemp = TRecInterval[trecChoose]+1;
    }    
    if(item ==7)
    {
        if(bIncrease)
        {
          printFromSDtoSerial();
        }    

    }
    
//    }
    rtc.adjust(DateTime(now_year, now_month,now_day,hh,mm,ss));
//    {
//      Serial.println("write RTC1307");
//    }
}
void display_setup_page(void)
{
      if(display_page != display_last_page) 
      {
        tft.fillScreen(TFT_BLACK);
        //tft.fillRect(0, 0, 160, 70, TFT_BLACK);
        display_last_page = display_page;
        tft.setTextColor(TFT_BLACK,TFT_WHITE);
        tft.setCursor(25, 2,2);
        tft.print(" SETTINGS ");
//        tft.setCursor(0, 20,1);
//        tft.println("*MENU key to next page");
//        tft.println("*Exit key to time page");
//        tft.println("*Direction key to select and change");
//        Serial.println("Enter in settings page");
      }
        
      tft.setTextColor(TFT_GREEN,TFT_BLACK);
//      tft.setTextColor(TFT_YELLOW,TFT_BLACK); 
      set_item_color(0,setuppage1_focus);
      tft.setCursor(0, 15,1);
      tft.println();
      
      tft.print(" Year  :   ");
      tft.println(now_year);
      
      
      set_item_color(1,setuppage1_focus);
      tft.print(" Month :   ");
      tft.print(now_month);
      tft.println("  ");
      
      set_item_color(2,setuppage1_focus);
      tft.print(" Day   :   ");
      tft.print(now_day);
      tft.println("  ");

      //tft.setCursor(10, 50,4);
      set_item_color(3,setuppage1_focus);
      tft.print(" Hour  :   ");
      tft.print(hh);
      tft.println("  ");
      
      set_item_color(4,setuppage1_focus);
      tft.print(" Minute:   ");
      tft.print(mm);
      tft.println("  ");
      
      set_item_color(5,setuppage1_focus);
      tft.print(" Second:   ");
      tft.print(ss);
      tft.println("  ");

      tft.println();
      
      //tft.setCursor(0, 100,2);
      set_item_color(6,setuppage1_focus);
      tft.print(" Temp REC :  ");
      if(trecChoose)
         tft.print(String(TRecInterval[trecChoose])+" S");
      else
        tft.print("OFF  ");
      tft.println("      ");

      tft.println();
      
      set_item_color(7,setuppage1_focus);
      tft.print(" Print to Serial:  ->");
      //if(trecChoose)

      
}



void DataLogger(String sFilename,String sData,bool timeOn)
{
    digitalWrite(PIN_LED_REC,LOW);
  
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    RP2040_SDLib::File dataFile = SD.open(sFilename, FILE_WRITE);
  
    // if the file is available, write to it:
    if (dataFile) 
    {
      if(timeOn){

        dataFile.print(dateStamp+" ");

//        dataFile.print(" ");
//        dataFile.print(hh);
//        dataFile.print(":");
//        dataFile.print(mm);
//        dataFile.print(":");
//        dataFile.print(ss);
//        dataFile.print("->");
        dataFile.print(timeStamp(hh,mm,ss));
      }
      dataFile.println(sData);
      dataFile.close();
      
      // print to the serial port too:
      Serial.print("rec to file:");
      Serial.println(sFilename);
    }
    // if the file isn't open, pop up an error:
    else 
    {
      Serial.print("Error opening "); Serial.println(sFilename);
    }
    digitalWrite(PIN_LED_REC,HIGH);
}


void printDirectory(RP2040_SDLib::File dir, int numTabs) 
{
  while (true) 
  {
    RP2040_SDLib::File entry =  dir.openNextFile();

    if (! entry) 
    {
      // no more files
      break;
    }
    
    for (uint8_t i = 0; i < numTabs; i++) 
    {
      Serial.print('\t');
    }
    
    Serial.print(entry.name());
    
    if (entry.isDirectory()) 
    {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } 
    else 
    {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    
    entry.close();
  }
}

void PrintFilesInDirectory(RP2040_SDLib::File dir, int numTabs) 
{
  while (true) 
  {
    RP2040_SDLib::File entry =  dir.openNextFile();

    if (! entry) 
    {
      // no more files
      break;
    }
    
    for (uint8_t i = 0; i < numTabs; i++) 
    {
      Serial.print('\t');
    }
        Serial.print(entry.name());
    
    if (entry.isDirectory()) 
    {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } 
    else 
    {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    while (entry.available()) {
      Serial.write(entry.read());
    }
    
    entry.close();
  }
}

//String datatimeStamp(uint16_t y,uint8_t mon,uint8_t d,uint8_t h,uint8_t m,uint8_t s)
String timeStamp(uint8_t h,uint8_t m,uint8_t s)
{
  String stamp;
//  stamp=String(y) + "-" ;
//  if(mon<10)
//    stamp += "0" + String(mon) + "-";
//  else
//    stamp +=  String(mon) + "-";  
//    
//  if(d<10)
//    stamp += "0" + String(d) + " ";
//  else
//    stamp +=  String(d) + " ";  
    
  if(h<10)
    stamp += "0" + String(h) + ":";
  else
    stamp +=  String(h) + ":"; 

  if(m<10)
    stamp += "0" + String(m) + ":";
  else
    stamp +=  String(m) + ":";

  if(s<10)
    stamp += "0" + String(s) + "->\t";
  else
    stamp +=  String(s) + "->\t";
  return stamp;
}

void printFromSDtoSerial(void)
{
    root = SD.open("/");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW,TFT_BLACK);
    tft.setCursor(5, 30,2);
    tft.println("Printing files...");
    tft.println("From SD to serial");
    PrintFilesInDirectory(root,0);
    tft.fillScreen(TFT_BLACK);
    //display_page = 99;
    //display_time_page();
    //display_page = PAGE_TIME;  
    //tft.fillScreen(TFT_YELLOW);
    Serial.print("print done");
    sIRMPDataAvailable = false;
}
  
