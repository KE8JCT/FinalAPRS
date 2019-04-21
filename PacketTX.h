/* Hamshield
 * Example: AFSK Serial Messenger
 * Serial glue to send messages over APRS. You will need a 
 * seperate AFSK receiver to test the output of this example.
 * Connect the HamShield to your Arduino. Screw the antenna 
 * into the HamShield RF jack. After uploading this program 
 * to your Arduino, open the Serial Monitor to monitor. Type 
 * a message under 254 characters into the bar at the top of 
 * the monitor. Click the "Send" button. Check for output on 
 * AFSK receiver.
 *
 * To send a message: connect to the Arduino over a Serial link.
 * Send the following over the serial link:
 * `from,to,:message
 * example: * KE8JCT,KE8JCT,:Hi there TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST`
 */



#include <HamShield.h>
#include <DDS.h>
#include <packet.h>
#include <avr/wdt.h> 

#define MIC_PIN 3
#define RESET_PIN A3
#define SWITCH_PIN 2
String messagebuff = "";

void setupRadio();

void prepMessage(String messagebuff) ;
