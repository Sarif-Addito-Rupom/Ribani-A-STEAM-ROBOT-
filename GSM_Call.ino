
#include <GSM.h>

// PIN Number
#define PINNUMBER "(TX=0,RX=1)"

// initialize the library instance
GSM gsmAccess; // include a 'true' parameter for debug enabled
GSMVoiceCall vcs;

String remoteNumber = "01870185367";  // the number you will call
char charbuffer[20];

void setup() {

  // initialize serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Make Voice Call");

  // connection state
  boolean notConnected = true;

  // Start GSM shield
  // If your SIM has PIN, pass it as a parameter of begin() in quotes
  while (notConnected) {
    if (gsmAccess.begin(PINNUMBER) == GSM_READY) {
      notConnected = false;
    } else {
      Serial.println("Not connected");
      delay(1000);
    }
  }

  Serial.println("GSM initialized.");
  Serial.println("Enter phone number to call.");

}

void loop() {

  // add any incoming characters to the String:
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    // if it's a newline, that means you should make the call:
    if (inChar == '\n') {
      // make sure the phone number is not too long:
      if (remoteNumber.length() < 20) {
        // let the user know you're calling:
        Serial.print("Calling to : ");
        Serial.println(remoteNumber);
        Serial.println();

        // Call the remote number
        remoteNumber.toCharArray(charbuffer, 20);


        // Check if the receiving end has picked up the call
        if (vcs.voiceCall(charbuffer)) {
          Serial.println("Call Established. Enter line to end");
          // Wait for some input from the line
          while (Serial.read() != '\n' && (vcs.getvoiceCallStatus() == TALKING));
          // And hang up
          vcs.hangCall();
        }
        Serial.println("Call Finished");
        remoteNumber = "";
        Serial.println("Enter phone number to call.");
      } else {
        Serial.println("That's too long for a phone number. I'm forgetting it");
        remoteNumber = "";
      }
    } else {
      // add the latest character to the message to send:
      if (inChar != '\r') {
        remoteNumber += inChar;
      }
    }
  }
}
