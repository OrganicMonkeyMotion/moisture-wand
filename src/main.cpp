/*
main.cpp - OpenMOIST ESP32 
Copyright (C) 2019 Asterion Daedalus (aka Bazmundi)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// ------------------------------------------------------------------------------------------ //
//                                                                                            //
//                                                                                            //
// Open Source libraries or ideas used (other than frameworks provided by PlatformIO):        //
//  - https://github.com/Imroy/pubsubclient                                                   //
//  - http://knolleary.net   (MIT licence)                                                    //
//  - https://www.instructables.com/ESP32-WiFi-SOIL-MOISTURE-SENSOR/                          //
//  - https://www.aranacorp.com/en/using-an-rfid-module-with-an-esp32/                        //
//                                                                                            //
// ------------------------------------------------------------------------------------------ //

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <MFRC522.h>
#include <PubSubClient.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"


// Update these with values suitable for your network.
const char* ssid        = "xxxxxxxx";  // Put your wifi router ssid here.
const char* password    = "yyyyyyyy";   // Put your wifi router password here.

// MQTT centric variables
IPAddress MQTTserver(192,168,1,130);  // Put your mqtt server IP address here.
WiFiClient wclient;
PubSubClient client(wclient, MQTTserver);  // using default port 1883


// root of topics
String OpenSprinklette = "/os";

// global listen on topics
String SoundOff        = OpenSprinklette + "/soundOff";
String AllStop         = OpenSprinklette + "/allStop";

// global talk on topics
String Debug           = OpenSprinklette + "/debug";
String LastWill        = OpenSprinklette + "/lwt";
String Herald          = OpenSprinklette + "/herald";


const int moistpin = 32;

float asoilmoist=analogRead(moistpin);
float gwc=exp(-0.0015*asoilmoist + 0.7072); //global variable to store the gravimetric soil water content

// OLED stuff

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
// On an ESP32:             21(SDA), 22(SCL) from: https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/ 

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// RC522 defines
// ------------------
// ESP32 WROOM-32
//      Signal	  Pin
#define RST	      16  // For the module, not actually involved in SPI - therefore, is not assotiated internally with SPI in ESP32.  
                      // Can be used as a power up/power down.
#define SS	      5
#define MOSI	    23
#define MISO	    19
#define SCK	      18
// ------------------

//Parameters
const int ipaddress[4] = {103, 97, 67, 25};

//Variables
byte nuidPICC[4] = {0, 0, 0, 0};
MFRC522::MIFARE_Key key;
MFRC522 rfid = MFRC522(SS, RST);



#define DHTPIN 22
#define DHTTYPE DHT11
#define TOUCHPIN 13
DHT dht(DHTPIN, DHTTYPE);
#define BUTTON_DOWN_THRESH 20
#define BUTTON_UP_THRESH 70

// get initial global values
float hum = 0.0;
float temp = 0.0;
float oldHum = 0.0;
float oldTemp = 0.0;

// chip id for the ESP32
String gadgetID (void) {

  uint32_t chipId = 0;
  for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}

  return String(chipId);
}


// topic to post on, differentiated by the gadget id
String moistureGadget ( void ) {
  
  return OpenSprinklette + "/gadget/sensor/moisture/" + gadgetID();
}


// debugging iff you want
void debugOut (String ouch) {

  client.publish(Debug, ouch);
}


// set type to sensor
String gadgetTypeIDVal (void) {

   return  String("4"); // sensor
}


// craft a json status packet
String gadgetMetaData ( String onlineFlag ) {

  return  "{ \"gadgetType\": " + gadgetTypeIDVal() + "," +
            "\"gadgetId\": "   + gadgetID()        + "," +
            "\"online\": "     + onlineFlag        + "}";
}

String myTag = String("");

// craft a sensor data packet
String sensorData ( float moisture, float gwc, float temperature, float humidity, String myTag) {

  return  "{ \"tag\": " + myTag    + "," +
            "\"moisture\": " + String(moisture)    + "," +
            "\"gwc\": "      + String(gwc)         + "," +
            "\"temp\": "     + String(temperature) + "," +
            "\"humidity\": " + String(humidity)    + "}";
}

// publish moisture sensor data
void moistureMessage ( float moisture, float gwc, float temperature, float humidity, String myTag ) {
  client.publish(moistureGadget(),sensorData(moisture, gwc, temperature, humidity, myTag));
}

// sensor header
String gadgetTypeVal (void) {

   return ( String("sensor: "));
}

// say hi, I'm here. Either on power up or in response to sound off request
void heraldMessage (void) {
  
  String heraldDebugString = gadgetTypeVal() + gadgetID() + String(" online!");


  // As per story with lwt.  If all is good, this gets through to node-red graph first, and
  // a gadget entry will be made in global list - with gadget online!
  String heraldString = gadgetMetaData("1");


  // Why two separate topics? well one is for comfort on the debug console. 
  // The other is for pumping into node-red graph for automation aspects. 
  // Yes, if you where really clever you could parse debug string.

  // Debug stream can be lifted out if there is a preference for a quiet debug channel.  The more 
  // important aspect is the lwt in any event.  That should as likely go to email or sms to warm
  // system owner that there is a problem.  Herald on debug is being kept until a distributed
  // key value store is set up, with automated assignment of gadgets to unit or group ID at
  // user end.
  client.publish(Debug, heraldDebugString);
  client.publish(Herald, heraldString);
}

// setup subscriptions
void registerGlobalListenOnTopics (void) {

  client.subscribe(SoundOff.c_str());
  client.subscribe(AllStop.c_str());
}

// response to sound off topic
int soundOffTopic (String topic) {

  if (SoundOff == topic) {
    
    heraldMessage(); 

    return true;
  } 

  return false;
}

// hold off mulitple display updates
bool waitingOnTag = true;
bool waitingOnButton = true;

// state of pin 13 (aka TOUCH4) as a phyical push button
enum buttonStates {buttonDown, buttonHold, buttonUp};
buttonStates buttonState = buttonUp;

// device states, either reading RFID or sending moisture topic
enum readStates {readTag, readSensor};
readStates readState = readTag;

// stop whatever you're doing
void allStop (void) {
  buttonState = buttonUp;
  readState = readTag;

  // show messages first time through loop,
  // but not subsequently
  waitingOnTag = true;  
  waitingOnButton = true;
  display.clearDisplay();
  display.display();
}


// respond to all stop topic
int allStopTopic (String topic) {

  if (AllStop == topic) {

    allStop();

    return true;
  } 

  return false;
}


// setup wifi
void setup_wifi() {

  delay(10);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    
  }

  randomSeed(micros());

}

// do something other than all stop or herald
void ActionTopic (const MQTT::Publish& pub) {  
  // for this app there is nothing else to catch
  int nop = 0;

  nop = nop;

};

void callback(const MQTT::Publish& pub) {

  if (not allStopTopic(pub.topic())) {
    if (not soundOffTopic(pub.topic())) {

      ActionTopic(pub); 
    }
  }
}

// connect or reconnect to mqtt server
void reconnect() {

  // loop until we're reconnected
  while (!client.connected()) {

    Serial.print("\nAttempting MQTT connection...");

    // create a client ID from chip id
    String clientId = "ESP32Client:";
    clientId += gadgetID();

    // last will and testament
    String finalWords = gadgetMetaData("0");      // MQTT server will likely blurt this out during wemos boot.
                                                  // Ignore message during wemos boot, just a quirk
                                                  // of a distributed system and the timing.
                                                  // Take heed post wemos boot that something is wrong.
                                                  // The idea, in any event, even if this comes through
                                                  // before herald, an entry for the device will be
                                                  // created - defaulting to offline.

    // attempt a connection
    if (client.connect(clientId.c_str(),LastWill.c_str(),2,true,finalWords.c_str())) {

      Serial.println("connected");

      // once connected, publish an announcement
      // to tell subscribers that you're up
      heraldMessage();

      // resubscribe

      registerGlobalListenOnTopics();

    }
    else {

      Serial.print("\nFailed! Try again in 5 seconds");
      delay(5000);
    }
  }
}

void buttonDownDetectInterrupt (void) {
  buttonState = buttonDown;
}

void displayUpdate(String message) {
  // So, I know right? Why?
  // I bought 10xSSD1306 128x32 displays from Aliexpress.
  // The second one I grabbed out of that batch seems to have
  // a problem with clearing the display.   
  display.clearDisplay();
  display.invertDisplay(true);
  display.display();
  display.clearDisplay();
  display.invertDisplay(false);
  display.display();
  display.clearDisplay();
  display.setCursor(0,0);             // Start at top-left corner
  display.print(message);
  display.display();
}


// setup wifi and callbacks
void setup() {

  // setupconnection to MQTT broker
  setup_wifi();
  client.set_callback(callback);

  Serial.begin(115200);

  // setup the display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS,true)) {
    Serial.println(F("SSD1306 allocation failed"));
    return;
  }
  display.clearDisplay();
  delay(200);
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); 
  display.display();

  // setup the RFID tag
  SPI.begin();
  rfid.PCD_Init();
  Serial.print(F("Reader :"));
  rfid.PCD_DumpVersionToSerial();
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.println(rfid.PICC_GetTypeName(piccType));

  // let user know the device is up
  displayUpdate("Setup complete");
  delay(1500);
}

// Helper routine to dump a byte array as dec values to Serial,
// and return a tag ID for our MQTT topic.

const String printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
  }

  String myTagtemp = String("");

  for (byte i = 0; i < bufferSize; i++) {
    myTagtemp = myTagtemp + String(buffer[i]) + ":";
  }
  
  // output is "DEC:DEC:DEC:DEC"
  return myTagtemp.substring(0,myTagtemp.length()-1);
}

bool readRFID(void ) { 

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  // Look for new 1 cards
  if ( ! rfid.PICC_IsNewCardPresent())
    return false;

  // Verify if the NUID has been readed
  if (  !rfid.PICC_ReadCardSerial())
    return false;

  // Store NUID into nuidPICC array
  for (byte i = 0; i < 4; i++) {
    nuidPICC[i] = rfid.uid.uidByte[i];
  }

  Serial.print(F("RFID In dec: "));
  myTag = printDec(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();

  return true;

}


// forever
void loop() {

  
  if (!client.connected()) {

    allStop();  // make sure state machines start properly
    displayUpdate("Waiting on broker");
    delay(1500);
    reconnect();
  }

  //exponential smoothing of soil moisture
  asoilmoist=0.95*asoilmoist+0.05*analogRead(moistpin);

  if (readState == readTag) {
    
    if (waitingOnTag) {  // once on first time through loop
      displayUpdate("Waiting on RFID tag");
      waitingOnTag = !waitingOnTag;
    }

    if (readRFID()) {
      readState = readSensor;
      touchAttachInterrupt(TOUCHPIN, buttonDownDetectInterrupt, BUTTON_DOWN_THRESH);
    }
    
  } else if (readState == readSensor) {

      if (waitingOnButton) { // once on first time through loop
        displayUpdate("Waiting on button");
        waitingOnButton = !waitingOnButton;
      }

      if (buttonState == buttonDown) { // hold button down
        
        buttonState = buttonHold;
        
        // stop reacting to button still down
        touchDetachInterrupt(TOUCHPIN);

        // give me one ping captain
        gwc=exp(-0.0015*asoilmoist + 0.7072); //global variable to store the gravimetric soil water content
        temp = dht.readTemperature();

        //deal with nan returns, as temp and humidity should not be swinging wildly, just lie
        if (isnan(temp)) {
          temp=oldTemp;
        } else {
          oldTemp=temp;
        }

        hum = dht.readHumidity();
        if (isnan(hum)) {
          hum=oldHum;
        } else {
          oldHum=hum;
        }

        // publish to MQTT topic
        moistureMessage ( asoilmoist, gwc, temp, hum, myTag );

      } else if (buttonState == buttonHold) {

        // keep pin down till it makes sense its up
        if (touchRead(TOUCHPIN) > BUTTON_UP_THRESH) {

          buttonState = buttonUp;
          readState = readTag;

          waitingOnTag = true;
          waitingOnButton = true;
        }
      }

  }

  client.loop();

}
