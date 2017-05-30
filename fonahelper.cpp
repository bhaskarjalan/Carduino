#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"

#define halt(s) { DPRINT(F( s )); while(1);  }


#define DEBUGMODE 1            // Run in debugging mode?
#ifdef DEBUGMODE
  #define DPRINT(x)  Serial.println(x)
#else
  #define DPRINT(x)
#endif

extern Adafruit_FONA fona;
extern SoftwareSerial fonaSS;

boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password) {
  Watchdog.reset();

  DPRINT(F("Initializing FONA....(May take 3 seconds)"));
  
  Serial3.begin(4800); // if you're using software serial
  
  if (! fona.begin(Serial3)) {           // can also try fona.begin(Serial1) 
    DPRINT(F("Couldn't find FONA"));
    return false;
  }
  Serial3.println("AT+CMEE=2");
  DPRINT(F("FONA is OK"));
  Watchdog.reset();
  DPRINT(F("Checking for network..."));
  while (fona.getNetworkStatus() != 1) {
   delay(500);
  }

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
  
  DPRINT(F("Enabling GPS"));
  if(! fona.enableGPS(true))
	  DPRINT(F("Error during GPS init..."));
  
  Watchdog.reset();
  
  fona.setGPRSNetworkSettings(apn, username, password);

  DPRINT(F("Disabling GPRS"));
  fona.enableGPRS(false);
  
  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();

  DPRINT(F("Enabling GPRS"));
  if (!fona.enableGPRS(true)) {
    DPRINT(F("Failed to turn GPRS on"));  
    return false;
  }
  Watchdog.reset();

  return true;
}
