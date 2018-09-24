
/*   Speedometer Nixie Display
 *    NixieKeith - http://www.glowtubeglow.com/
 *    
 *    This leverages base code from Arduinix http://www.arduinix.com/index.html
 *    and based on various authors' open source code shown in the libraries. 
 *
 *  fading transitions sketch for 4-tube board with default connections.
 *  based on 6-tube sketch by Emblazed
 *  4-tube-itized by Dave B. 16 June 2011
 *  
 *  This shows 0-99 MPH only
 *  
 *  DEBUGON -> Turns on all serial print output and allows testing nixie code without a GPS connection
 *  OLDEON -> Turns on OLED display (small OLED for redundant and testing display. This will require using analog port (0-1?) for OLED OUTPUT
 *  
 *  
*/
#define MPHOPT true   // Set this to "false" for KPH, "true" for MPH
//#define DEBUGON
//#define OLDEON


#include <GPSfix_cfg.h>
#include <NeoGPS_cfg.h>
#include <NMEAGPS_cfg.h>

// #include <SPI.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NeoSWSerial.h>

#include <Bounce2.h>

#define BUTTON_PIN 0
#define LED_PIN 13
#define SLEEPTIME 120000 // how long before the defice sleeps in millis  10000=10 seconds   120000=2 minutes

// Instantiate a Bounce object
Bounce debouncer = Bounce(); 

boolean mphon=MPHOPT;  // set to false for kph 
#define TOGGLE_MPH if (mphon) mphon=false; else mphon=true;

NeoSWSerial gps_port(19,18);  // was default to 4, 3

#include "NMEAGPS.h"

static NMEAGPS  gps; // This parses the GPS characters

#ifdef DEBUGON
int incr=0;
#endif


#define OLED_RESET 4

#ifdef OLEDON
Adafruit_SSD1306 display(OLED_RESET);
#endif

#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 


// Globals

float mph = 0; float lastmph = 0;
boolean sleeping = false;
uint32_t sleepytime = 0;
int sleepchar = 0;
unsigned long buttonPressTimeStamp;
int value = LOW;
uint32_t timer;
/* SN74141 : Truth Table
* D C B A = #
* L,L,L,L = 0
* L,L,L,H = 1
* L,L,H,L = 2
* L,L,H,H = 3
* L,H,L,L = 4
* L,H,L,H = 5
* H,H,H,L = 6   Note: Inverted with 7 on the NixieKeith IN-17 header
* L,H,H,L = 7         Inverted with the 6 on NixieKeith IN-17 header
* H,L,L,L = 8
* H,L,L,H = 9
*/

// SN74141 (1)
int ledPin_0_a = 2;                
int ledPin_0_b = 3;
int ledPin_0_c = 4;
int ledPin_0_d = 5;

// SN74141 (2)
int ledPin_1_a = 6;                
int ledPin_1_b = 7;
int ledPin_1_c = 8;
int ledPin_1_d = 9;

// anode pins
// NOTE: V1 board has this order flipped
int ledPin_a_1 = 10;
int ledPin_a_2 = 11;
int ledPin_a_3 = 12;
int ledPin_a_4 = 13;


/* Added for fade and better control of Nixies  */
float rawRange = 1024; // 3.3v
float logRange = 5.0; // 3.3v = 10^5 lux

const int numReadings = 30; //smoothing
int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

float fadeIn = 0.0f;
float fadeOut = 8.0f;
float fadeMax = 8.0f;
float fadeStep = 1.0f;
int NumberArray[8]={0 , 0, 0, 0, 0,0,0,0};
int currNumberArray[8]={0,0,0,0,0 ,0,0,0};
float NumberArrayFadeInValue[8]={9.0f,9.0f,9.0f,9.0f,9.0f,9.0f,9.0f,9.0f};
float NumberArrayFadeOutValue[8]={9.0f,9.0f,9.0f,9.0f,9.0f,9.0f,9.0f,9.0f};

void SetSN74141Chips( int num2, int num1 )
{
    int a,b,c,d;
  
    // set defaults.
    a=0;b=0;c=0;d=0; // will display a zero.
  
    // Load the a,b,c,d.. to send to the SN74141 IC (1)
    switch( num1 )
    {
        case 0: a=0;b=0;c=0;d=0;break;
        case 1: a=1;b=0;c=0;d=0;break;
        case 2: a=0;b=1;c=0;d=0;break;
        case 3: a=1;b=1;c=0;d=0;break;
        case 4: a=0;b=0;c=1;d=0;break;
        case 5: a=1;b=0;c=1;d=0;break;
//        case 6: a=1;b=1;c=1;d=0;break;  // For some reason these digits 7 & 6 are switched on the header
        case 6: a=0;b=1;c=1;d=0;break;
//        case 7: a=0;b=1;c=1;d=0;break;
        case 7: a=1;b=1;c=1;d=0;break;  // For some reason these digits 7 & 6 are switched on the header
        case 8: a=0;b=0;c=0;d=1;break;
        case 9: a=1;b=0;c=0;d=1;break;
        default: a=1;b=1;c=1;d=1;
        break;
    }  
  
    // Write to output pins.
    digitalWrite(ledPin_0_d, d);
    digitalWrite(ledPin_0_c, c);
    digitalWrite(ledPin_0_b, b);
    digitalWrite(ledPin_0_a, a);

    // Load the a,b,c,d.. to send to the SN74141 IC (2)
    switch( num2 )
    {
        case 0: a=0;b=0;c=0;d=0;break;
        case 1: a=1;b=0;c=0;d=0;break;
        case 2: a=0;b=1;c=0;d=0;break;
        case 3: a=1;b=1;c=0;d=0;break;
        case 4: a=0;b=0;c=1;d=0;break;
        case 5: a=1;b=0;c=1;d=0;break;
        case 6: a=1;b=1;c=1;d=0;break;
// case 6: a=0;b=1;c=1;d=0;break;  // For some reason these digits 7 & 6 are switched on the header
        case 7: a=0;b=1;c=1;d=0;break;
//  case 7: a=1;b=1;c=1;d=0;break;  // For some reason these digits 7 & 6 are switched on the header
        case 8: a=0;b=0;c=0;d=1;break;
        case 9: a=1;b=0;c=0;d=1;break;
        default: a=1;b=1;c=1;d=1;
        break;
    }
  
    // Write to output pins
    digitalWrite(ledPin_1_d, d);
    digitalWrite(ledPin_1_c, c);
    digitalWrite(ledPin_1_b, b);
    digitalWrite(ledPin_1_a, a);
}
////////////////////////////////////////////////////////////////////////
//
// DisplayNumberString
// Use: passing an array that is 4 elements long will display numbers
//      on a 4 nixie setup.
//
////////////////////////////////////////////////////////////////////////


void DisplayFadeNumberString()
{
  // Nixie setup..
  
  // NOTE: If any of the bulbs need to blend then it will
  // be in time with the seconds bulbs. because any change only happens
  // on a one second interval. 

  // For IN-12+IN-15 we shift digits to the left and use the low order for special character displays. 
  // IN-15a/b is used in the low order nixie slot (right-most). 
  // IN-12 is used for the numerals in the left 3 slots
  
     // Anode channel 1 - numerals 1,3
        SetSN74141Chips(currNumberArray[3],currNumberArray[2]);   
        digitalWrite(ledPin_a_1, HIGH);   
        delay(NumberArrayFadeOutValue[1]);
        SetSN74141Chips(NumberArray[3],NumberArray[2]);   
        delay(NumberArrayFadeInValue[1]);
        digitalWrite(ledPin_a_1, LOW);
  
      // Anode channel 2 - numerals 2,4
        SetSN74141Chips(currNumberArray[0],currNumberArray[1]);   
        digitalWrite(ledPin_a_2, HIGH);   
        delay(NumberArrayFadeOutValue[2]);
        SetSN74141Chips(NumberArray[0],NumberArray[1]);   
        delay(NumberArrayFadeInValue[2]);
        digitalWrite(ledPin_a_2, LOW);


      // Anode channel 2 - Upper and lower INS-1 lamps 
      // forced off 
        SetSN74141Chips(1,1);   
        digitalWrite(ledPin_a_3, LOW);   
        digitalWrite(ledPin_a_4, LOW);
        delay(NumberArrayFadeOutValue[2]);
        SetSN74141Chips(1,1);   
        delay(NumberArrayFadeInValue[2]);
        digitalWrite(ledPin_a_3, LOW);
        digitalWrite(ledPin_a_4, LOW);

        for( int i = 0 ; i < 8 ; i ++ ) //equal to & of digits
        {
            //if( NumberArray[i] != currNumberArray[i] )
            {
                NumberArrayFadeInValue[i] += fadeStep;
                NumberArrayFadeOutValue[i] -= fadeStep;
  
               if( NumberArrayFadeInValue[i] >= fadeMax )
               {
                   NumberArrayFadeInValue[i] = fadeStep;
                   NumberArrayFadeOutValue[i] = fadeMax; //affects the refresh cycle
                   currNumberArray[i] = NumberArray[i];
                }
            }   
        }
      
}

void Update_Nixie_Speed(float speed){
   // Convert to digits
  int mphnow = speed; // Already rounded 
#ifdef DEBUGON
  Serial.print("\tMPH Rounded="); Serial.print(mphnow);Serial.print("\t");
#endif
    int x = mphnow;
    size_t i, size = 0;
 if (x == 0) size = 1; // patched to allow for single value zero. Otherwise next state skips with a valid zero value  
 while ( x ) {
       x=x/10;
       size++;
    }
 int temp[size];
 if (mphnow != 0){   // zero is a special case that makes it look like it is an empty set when the set = zero. all other cases below work
    for ( i = size - 1, x = mphnow ; x ; x = x/10, i-- ) {
        temp[i] = x % 10;
#ifdef DEBUGON
        Serial.print("i="); Serial.print(i);
        Serial.print(" x=");Serial.print(x);
        Serial.print(" temp[i]=");Serial.print(temp[i]); Serial.print("\t");
#endif
    }
 } else {
  temp[0]=0;  // If mph = 0 then put zero in digit 1. This is a work around to the size = zero patch
 }
  // Just for clenliness put everything to zeros
  for (i = 0; i < 8; ++i){
    NumberArray[i] = 0;
  };

  // The digits come in inverse order from the NumberArray. e,g, temp[highest value] is the rightmost digit and goes into the LOWEST index of NumberArray[lowest] 
  if (mphnow < 10){
//      NumberArray[1] = 0;     // tens
      NumberArray[0] = temp[0]; // lowest order 
  }  
  else {
    if (mphnow > 99){   // it is 3 digits
      NumberArray[2] = temp[0];      // tens
      NumberArray[1] = temp[1];     //  lowest
      NumberArray[0] = temp[2];     //  lowest
    }  
    else{   //  Must be 2 digits
      NumberArray[1] = temp[0];      // tens
      NumberArray[0] = temp[1];     //  lowest
    }
  }
  if (mphon)
    NumberArray[3]=6;   // number 6 maps to pin 6 "m" on IN-15a
  else 
    NumberArray[3]=4;   // number 4 maps to pin 8 "k" on IN-15a
    
 
  /* 
   *  Header
   *  /----------------------------/
   *  / 0    1    4    3   Top         /
   *  /----------------------------/
   */
  // Display.
#ifdef DEBUGON
  if(mphon) Serial.print("MPH > "); else Serial.print("KPH > ");
  Serial.print("\tNumberArray >"); 
  Serial.print(NumberArray[0]); Serial.print(":");
  Serial.print(NumberArray[1]); Serial.print(":");
  Serial.print(NumberArray[2]); Serial.print(":");
  Serial.print(NumberArray[3]); Serial.print(":");
  Serial.print(NumberArray[4]); Serial.print(":");
  Serial.print(NumberArray[5]); Serial.print(":");
  Serial.print(NumberArray[6]); Serial.print(":");
  Serial.print(NumberArray[7]); Serial.println(""); 
#endif
}
void Update_Sleep_Display(){
   // Convert to digits
  if (++sleepchar > 9) {sleepchar = 0;}; 

  for (int i = 0; i < 8; ++i){
    NumberArray[i] = sleepchar;
  };
     
  /* 
   *  Header
   *  /----------------------------/
   *  / 0    1    4    3   Top         /
   *  /----------------------------/
   */
  // Display.
#ifdef DEBUGON
  if(mphon) Serial.print("MPH > "); else Serial.print("KPH > ");
  Serial.print("\tNumberArray >"); 
  Serial.print(NumberArray[0]); Serial.print(":");
  Serial.print(NumberArray[1]); Serial.print(":");
  Serial.print(NumberArray[2]); Serial.print(":");
  Serial.print(NumberArray[3]); Serial.print(":");
  Serial.print(NumberArray[4]); Serial.print(":");
  Serial.print(NumberArray[5]); Serial.print(":");
  Serial.print(NumberArray[6]); Serial.print(":");
  Serial.print(NumberArray[7]); Serial.println(""); 
#endif
}

void setup() {

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  // Setup the button with an internal pull-up :
  
  //pinMode(BUTTON_PIN,INPUT_PULLUP);

  // After setting up the button, setup the Bounce instance :
  debouncer.attach(BUTTON_PIN,INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  debouncer.interval(25); // Use a debounce interval of 25 milliseconds

 #ifdef OLEDON
 display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
  
  // Show splash image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  /* display.display();
  delay(2000);
*/
#ifdef DEBIGON
// Seed to repeatable random list for testing only. 
  randomSeed(1); // repeatable pseudo-random numbers 

#endif
  // Clear the buffer.
  display.clearDisplay();
  // draw a single pixel
  display.drawPixel(10, 10, WHITE);
  // Show the display buffer on the hardware.
  // NOTE: You _must_ call display after making any drawing commands
  // to make them visible on the display hardware!
  display.display();
//  delay(2000);
  display.clearDisplay();

  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  // display.println("Hello, world!");
#endif

  Serial.println("Setup GPS..." );
  gps_port.begin(9600);

  // request RMC and GGA only
  gps_port.println( F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28") );

// now set up nixies

  pinMode(ledPin_0_a, OUTPUT);      
  pinMode(ledPin_0_b, OUTPUT);      
  pinMode(ledPin_0_c, OUTPUT);      
  pinMode(ledPin_0_d, OUTPUT);    
  
  pinMode(ledPin_1_a, OUTPUT);      
  pinMode(ledPin_1_b, OUTPUT);      
  pinMode(ledPin_1_c, OUTPUT);      
  pinMode(ledPin_1_d, OUTPUT);      
  
  pinMode(ledPin_a_1, OUTPUT);      
  pinMode(ledPin_a_2, OUTPUT);      
  pinMode(ledPin_a_3, OUTPUT);      
  
  // NOTE:
  // You can add a input here to for setting the time
#ifdef OLEDON
// Now for display 
 // Done already 
 //display.begin(SSD1306_SWITCHCAPVCC);

  display.clearDisplay();
 // display.setTextSize(2.5);
  display.setTextColor(WHITE);
 // display.setCursor(15, 15);
 // display.println( F("Mini-GPS"));
  display.setTextSize(1.8);
  display.setCursor(5, 5);
  display.print( F("Speedometer"));
  display.display(); // SEND SPLASH SCREEN
  delay(5000);       //  WAIT 5 SECONDS
#endif
  timer = millis();
  sleepytime = timer;  // initialize the sleep timer to "0"
}

void loop()    // Main Driver loop
{
/* Logic: 
 *  1) Display the Nixie buffer to the Arduinix shield (Defailt is all zeros
 *    Nixie display is only the two rightmost digits even though this code drive 4 digits. 
 *    This is for future use. I have some ideas with this especially for IN-12/15 nixies. 
 *  
 *  2) Check the GPS device to see if it there and working
 *    - If not, then loop waiting for a connect to a satellite
 *    - If GPS is there: 
 *        1) Get time agian (again just for possible usage)
 *        2) Do a read on the GPS device and grab time and heading/speed
 *        3) Grab and convert speed to MPH (using the GPS calls)
 *        4) Format the time and the speed to be ready for the nixies. 
 *        5) Set up the buffer for the Nixie (Does not actually do the display to the nixies). 
 *        
 *        DEBUG define turns on a bunch of debug logic.
 *          It turns off real GPS MPH gather and replaces it with a random MPH value (0-99)
 *          It outputs to serial port showing the buffers and values, etc. 
 *        OLED define option turns on display onto an OLED screen. 
 *          OLED must be driven by the analog ports becuase no other pins are available when using Arduinix. 
 *          
 *  
 */
  
  
// was   DisplayNumberString( NumberArray ); 
  DisplayFadeNumberString(); 
 // Check Button  
   debouncer.update(); // Update the Bounce instance
   
   if ( debouncer.fell() ) {  // Call code if button transitions from HIGH to LOW
     TOGGLE_MPH;
   } 

   
  while (gps_port.available()) {

    timer = millis(); // reset the timer

    if (gps.decode( gps_port.read() ) == NMEAGPS::DECODE_COMPLETED) {

      if (gps.nmeaMessage == NMEAGPS::NMEA_RMC) {
        //  If your device emits a GGA then an RMC each second,
        //    change the above GGA to RMC.  Or if you don't get all
        //    the attributes (e.g., alt, heading, speed and satellites)
        //    try changing the above test.  NMEAorder.ino can be
        //    used to determine which one is last (should be used above).

        //  BTW, this is the safest place to do any time-consuming work,
        //    like updating the display.  Doing it elsewhere will cause GPS
        //    characters to be lost, and some fix data won't be available/valid.
        //    And if you take too long here, you could still lose characters.

        uint32_t displayTime = micros(); // use this later, to figure out how long the display process takes.
        const gps_fix & fix = gps.fix();

        //  construct data to be sent to the OLED *****************************
#ifdef OLEDON
        display.clearDisplay(); //CLEAR THE OLED BUFFER, THUS CLEARING THE SCREEN:  GOT IT!
        
        display.setTextSize(1);
        display.setTextColor(WHITE);

        display.setCursor(0, 0);
        display.print( F("Sat: "));

        if (fix.valid.satellites) {
          display.setCursor(30, 0);
          display.println( fix.satellites );
        }
#endif

    if (fix.valid.date && fix.valid.time) {
#ifdef OLEDON
          display.setCursor(95, 0);
#endif
          NeoGPS::clock_t seconds = (NeoGPS::clock_t) fix.dateTime;     // convert pieces to seconds
          seconds -= 4 * 60 * 60;                               // offset seconds
          NeoGPS::time_t localTime( seconds );                    // convert seconds back to pieces

#ifdef OLEDON
          if (localTime.hours < 10) {
            display.print( F("0"));
          }
          display.print( localTime.hours ); display.print( ':' ); // use pieces
          if (localTime.minutes < 10) {
            display.print( F("0"));
          }
          display.print( localTime.minutes);
#endif

        }
        if (fix.valid.speed) {
          if (mphon){
            mph = round(fix.speed_mph());
          }
          else {
            mph = round(fix.speed_kph());
          }
#ifdef OLEDON
          display.setTextSize(4);
          display.setCursor(35, 5);
          display.println( mph, 0);
#endif
#ifdef DEBUGON
//          if (++incr > 99) incr = 0;

          mph = random(0,142);
          Serial.print("Override Speed="); Serial.print(mph);
#endif


/*          if (lastmph >1 || mph > 1) {  
              sleeping = false;
              sleepytime = millis();
          } // Reset if the speedo comes to life 
*/      
           if (sleeping && mph > 1){
            // wake up!
            sleeping = false;
            sleepytime = millis();
           }    
           if(!sleeping)
            sleepytime = (millis() - sleepytime); // sleepytime is the difference
           if (sleeping && (sleepytime > SLEEPTIME)) 
            sleepytime = sleepytime - SLEEPTIME;
 Serial.print("last,mph,sleepytime="); Serial.print(lastmph); Serial.print(":");Serial.print(mph); Serial.print(":");Serial.print(sleepytime); Serial.println(".");

           if ((lastmph < 2) && (mph < 2) && (sleepytime >= SLEEPTIME) ){  // SLEEPTIME is the value for the idle timer 
              sleeping = true;
              Update_Sleep_Display();
              
//                          Serial.println("MPH/KMPH < 2 AND SLEEPTIME has been exceeded.");
            }
          else  {                       
//                  Serial.println("Updating Nixies, Sleeping flag off, and sleepytime left alone.");
                  Update_Nixie_Speed(mph);
                  sleeping = false;
                  lastmph = mph;

                }
            }


#ifdef OLEDON
        display.display();
#endif

//        Serial.print( F("dt = ") );
//        Serial.print( micros() - displayTime ); // How long did all that take?
 //       Serial.println( F(" us") );
      }
    }
  }

#ifdef OLEDON
  display.setCursor(10, 57);
  display.println( F("Acquiring a GPS signal...") );
#endif

  // Until we get sentences, print a dot every 2 seconds or so
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    // **************************************************
    Serial.print( '.' );
  }
}
