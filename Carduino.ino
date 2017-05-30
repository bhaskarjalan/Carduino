
/***************************************************
  Carduino - SIM808 Shield & Adafruit IO & CANBUS Shield
  Author: Maxime Lienard
  See the guide at:
 ****************************************************/
#include <Adafruit_SleepyDog.h>

#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"

#include <SPI.h>
#include <SD.h>
#include "Canbuss.h"

/*************************** DEBUGGING ***********************************/

#define DEBUGMODE 1						// Run in debugging mode?
#ifdef DEBUGMODE
	#define DPRINT(x)  Serial.println(x)
#else
	#define DPRINT(x)
#endif

/*************************** FONA Pins ***********************************/
#define FONA_PWR 		9							// SIM808 power pin
#define FONA_RST 		5							// SIM808 reset pin
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

/*************************** CAN Pins ***********************************/
#define CAN_CS 			10							// CAN MODULE CS pin
CAN myCAN(CAN_CS);
#define ENGINE_RPM 0x0C
#define VEHICLE_SPEED 0x0D

/*************************** uSD Pins ***********************************/
#define SD_CS_PIN 			4							// microSD CS pin
File logfile;
/************************* WiFi Access Point *********************************/

  // Optionally configure a GPRS APN, username, and password.
  // You might need to do this to access your network's GPRS/data
  // network.  Contact your provider for the exact APN, username,
  // and password values.  Username and password are optional and
  // can be removed, but APN is required.
#define FONA_APN       "internet.proximus.be"
#define FONA_USERNAME  ""
#define FONA_PASSWORD  ""

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "pivooo"
#define AIO_KEY         "85b620d571ad4ed8b780601708ce2f57"
#define LOGGING_PERIOD  20
int logCounter = 0;

/************ Global State (you don't need to change this!) ******************/

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish velocity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/speed");
Adafruit_MQTT_Publish location = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gps/csv");
Adafruit_MQTT_Publish engine = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/rpm");
// Setup a feed called 'onoff' for subscribing to changes.
//Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");



/****************************** Timer ***************************************/
SIGNAL(TIMER0_COMPA_vect) {
  // Decrease the count since last location log.
  if (logCounter > 0) {
    logCounter--;
  }
}

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;
#define MAXTXFAILURES 3

void setup() {
	#ifdef DEBUGMODE
	while (!Serial);
	#endif
	
	// Configure timer0 compare interrupt to run and decrease the log counter every millisecond.
	OCR0A = 0xAF;
	TIMSK0 |= _BV(OCIE0A);
	
	//Watchdog is optional!
	Watchdog.enable(10000);

	Serial.begin(115200);

	DPRINT(F("Carduino V2"));

	//Initialize SD card
	while(!SD.begin(SD_CS_PIN)){
	DPRINT(F("uSD card failed to initialize, or is not present"));
	}
	DPRINT(F("uSD card initialized."));


	// Create the next log file on the SD card.
	char filename[15];
	strcpy(filename, "LOG00.CSV");
	for (uint8_t i = 0; i < 100; i++) {
	filename[3] = '0' + i/10;
	filename[4] = '0' + i%10;
	// Create file if it does not exist.
	if (!SD.exists(filename)) {
	  break;
	}
	}
	DPRINT("Using log file: "); 
	DPRINT(filename);

	// Open the log file.
	logfile = SD.open(filename, FILE_WRITE);
	if(!logfile)
		DPRINT(F("Failed to open log file!"));
	Watchdog.reset();
	
	// Initialize the CANBUS module.
	while((!myCAN.begin(500))){
		DPRINT("CAN BUS Shield init fail");
		DPRINT(" Init CAN BUS Shield again");
		delay(2000);
	}
	DPRINT(F("CAN BUS Shield init ok!"));
	
	
	Watchdog.reset();
	delay(5000);  // wait a few seconds to stabilize connection
	Watchdog.reset();

	// Initialise the FONA module
	while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD))) {
	DPRINT("Retrying FONA");
	}

	DPRINT(F("Connected to Cellular!"));

	Watchdog.reset();
	delay(5000);  // wait a few seconds to stabilize connection
	Watchdog.reset();
}

uint32_t speed=0, rpm=0;
float latitude, longitude, gps_speed, heading, altitude;
char fonaInBuffer[64]; //for notifications from the FONA

void loop() {
	// Make sure to reset watchdog every loop iteration!
	Watchdog.reset();
	
	// ping the server to keep the mqtt connection alive, only needed if we're not publishing
	if(! mqtt.ping()) {
		DPRINT(F("mqtt ping failed."));
	}
	
	if (myCAN.messageAvailable() == 1) {
    // Read the last message received
		myCAN.getMessage(&myCAN.messageRx);
		
		switch(myCAN.messageRx.data[2]){
			case ENGINE_RPM:
								rpm = uint16_t( myCAN.messageRx.data[3]*256 + myCAN.messageRx.data[4] )/4;
								#if (DEBUGMODE == 1)
									Serial.print(F("RPM: "));
									Serial.println(rpm);
								#endif
								break;
			case VEHICLE_SPEED:
								speed = myCAN.messageRx.data[3];
								#if (DEBUGMODE == 1)
									Serial.print(F("Speed: "));
									Serial.println(speed);
								#endif
								break;
			
			
			default:
								break;		
		}
    }
	
	Watchdog.reset();
	
	// Grab a GPS/GPRS reading.
	int8_t stat = fona.GPSstatus();
	if(stat < 4){
		fona.getGSMLoc(&latitude,&longitude);
		DPRINT(F("Localisation GPRS"));
	}
	else{
		fona.getGPS(&latitude, &longitude, &gps_speed, &heading, &altitude);
		DPRINT(F("Localisation GPS"));
	}
	
	Watchdog.reset();
	
	// Periodically log data.
	if (logCounter == 0) {
		// Ensure the connection to the MQTT server is alive (this will make the first
		// connection and automatically reconnect when disconnected).  See the MQTT_connect
		// function definition further below.
		MQTT_connect();

		Watchdog.reset();
		// Now we can publish stuff!
		DPRINT(F("\nSending speed val "));
		if (! velocity.publish(speed++)) {
			DPRINT(F("Failed"));
			txfailures++;
		} else {
			DPRINT(F("OK!"));
			txfailures = 0;
		}
		Watchdog.reset();
		
		if (! engine.publish(rpm++)) {
			DPRINT(F("Failed"));
			txfailures++;
		} else {
			DPRINT(F("OK!"));
			txfailures = 0;
		}
		Watchdog.reset();
		

		logLocation();
		logCounter = LOGGING_PERIOD*1000;
	}
	
	Watchdog.reset();  

	if (fona.available()){
		DPRINT(F("Received SMS"));
		SMS_response();
	}
	/*else
    DPRINT(F("No SMS"));*/
	Watchdog.reset(); 

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

void logLocation(){
	// Initialize a string buffer to hold the data that will be published.
	char sendBuffer[120];
	memset(sendBuffer, 0, sizeof(sendBuffer));
	int index = 0;
	
	// Start with '0,' to set the feed value.  The value isn't really used so 0 is used as a placeholder.
	sendBuffer[index++] = '0';
	sendBuffer[index++] = ',';

	// Now set latitude, longitude, altitude separated by commas.
	dtostrf(latitude, 2, 6, &sendBuffer[index]);
	index += strlen(&sendBuffer[index]);
	sendBuffer[index++] = ',';
	dtostrf(longitude, 3, 6, &sendBuffer[index]);
	index += strlen(&sendBuffer[index]);
	
	// Finally publish the string to the feed.
	DPRINT(F("Publishing: "));
	DPRINT(sendBuffer);
	if (!location.publish(sendBuffer)) {
		DPRINT(F("Publish failed!"));
		txfailures++;
	}
	else {
		DPRINT(F("Publish succeeded!"));
		txfailures = 0;
	}
	
	DPRINT(F("Storing GPS Data on SD card"));
	logfile.println(sendBuffer);
}


void SMS_response() {
	char* bufPtr = fonaInBuffer;
	int slot = 0;            //this will be the slot number of the SMS
    int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = fona.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (++charCount < (sizeof(fonaInBuffer)-1)));
    
    //Add a terminal NULL to the notification string
    *bufPtr = 0;
    
    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(fonaInBuffer, "+CMTI: \"SM\",%d", &slot)) {
      Serial.print("slot: "); Serial.println(slot);
      
      char callerIDbuffer[32];  //we'll store the SMS sender number in here
      
      // Retrieve SMS sender address/phone number.
      if (! fona.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);
      
      //Send back an automatic response
      Serial.println("Sending reponse...");
      // Initialize a string buffer to hold the data that will be published.
      String strBuffer = "https://maps.google.com?saddr=Current+Location&daddr=";
      char sendBuffer[120];
      memset(sendBuffer, 0, sizeof(sendBuffer));
      int index = 0;


      strncpy(sendBuffer,strBuffer.c_str(),sizeof(sendBuffer));
    
      // Now set latitude, longitude, altitude separated by commas.
      index += strlen(&sendBuffer[index]);
      dtostrf(latitude, 2, 6, &sendBuffer[index]);
      index += strlen(&sendBuffer[index]);
      sendBuffer[index++] = ',';
      dtostrf(longitude, 3, 6, &sendBuffer[index]);
      index += strlen(&sendBuffer[index]);
  
      if (!fona.sendSMS(callerIDbuffer, sendBuffer)) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
      }
      
      // delete the original msg after it is processed
      //   otherwise, we will fill up all the slots
      //   and then we won't be able to receive SMS anymore
      if (fona.deleteSMS(slot)) {
        Serial.println(F("OK!"));
      } else {
        Serial.println(F("Couldn't delete"));
      }
  }

}
