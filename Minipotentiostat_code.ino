
// Implements potentiostat code on microcontroller (ESP32C3 Dev Module for Xiao SEEED)
// 12-bit ADC input readings on A2 
// Implements ledc functions for changing PWM frequency from 1 kHz to 3.1 kHz

#include <ArduinoBLE.h>

// Full list of Arduino BLE commands: https://github.com/arduino-libraries/ArduinoBLE
BLEService myService("0000ffe0-0000-1000-8000-00805f9b34fb"); // BLExAR BLE service
BLECharacteristic myCharacteristic("FFE1", BLEWrite | BLENotify,0x10);
BLEByteCharacteristic switchCharacteristic("0000ffe0-0000-1000-8000-00805f9b34fb", BLERead | BLEWrite);

long previousMillis = 0;  // last time the BMP280 was read, in [millisec]
int TXdelay = 20; // delay between sends - should match scan rate

int start_pin = 5;

int start_value = 0; ////

int a = 10; // PWM pin
int val = 0;
int sensorValue;

// use 10 bit precision for LEDC timer
#define LEDC_TIMER_12_BIT 10

// use 31370 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 31370

#define LED_PIN 10

int adc_reading = 1;
int c = 0;
int numSamples = 100;
long sum = 0; // Variable to store the sum of samples
float c_pos; //positive step current
float c_neg; //negative step current
float current_diff = 0; // difference in c_pos and c_neg
int n = 0;
float Potstep = 0.0078; // fixed due to the DAC resolution
int vevals[] = {200};
//int vevals[] = {200,20,50,100,200,250,300}; //if using multiple scan rates values (mV/s)
int const count = 1;
long intervalos[count];

// Values for Square Wave Voltammetry
float voltage1 = -0.9;
float voltage2 = 0.9;
float amplitude = 0.1; 
float potentialstepsize = 0.005; // V
int pulsewidth = 50; // milliseconds
float voltage = 0;
float voltagepositive = 0;
float voltagenegative = 0;


void setup() {

  Serial.begin(115200); //was 9600

  // Setup timer with given frequency, resolution and attach it to a led pin with auto-selected channel
  ledcAttach(LED_PIN, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);

  // Initialize 0% duty cycle ( at 12-bit resolution)
  ledcWrite(LED_PIN, 0); // 50% would be 2048


  if (!BLE.begin()) {
    while (1); // wait for BLE
  }

  // Build scan response data packet
  myService.addCharacteristic(myCharacteristic); // add BLE characteristic
  BLE.addService(myService); // add BLE service
  BLEAdvertisingData scanData;
  scanData.setLocalName("Potstat"); // set name
  BLE.setDeviceName("Potstat"); // set name


  // add the characteristic to the service
  myService.addCharacteristic(switchCharacteristic); ////

  // add service
  BLE.addService(myService); ////

  // set the initial value for the characteristic:
  switchCharacteristic.writeValue(0); ////

  BLE.setScanResponseData(scanData);// set data for scanners (BLE apps)
  BLE.advertise(); // advertise BLE device

  Serial.println("BLE Potentiostat");
  Serial.println(myCharacteristic.canRead());
  Serial.println(myCharacteristic.canWrite());

}

void loop() {

  // Approach used to receive value for 'start' signal through Blexar from Console

  // wait for a BLE central connection
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    while (central.connected()) {
      long currentMillis = millis();

        // This line looks for ANY input to start the scan (any character will trigger)
        if (myCharacteristic.written()) { //// these were switchCharacteristic

            start_value = 1;
            Serial.println("Start Scan");////
            Serial.println(myCharacteristic.canRead());
            Serial.println(myCharacteristic.canWrite());
            delay(3000);


      // Potentiostat code 
      for(int pos = 0; pos < count; pos++)
        {
        intervalos[pos]=(1000000L/((vevals[pos])*128L));
        }
      
      for(int pos = 0; pos < count; pos++){
      
      n = 1;
      
      while(n <= 1)
        { 
          //Start the forward scan for square wave voltammetry
          for(voltage = voltage1; voltage <= voltage2; voltage = voltage + potentialstepsize){ // val corresponds to voltage (0-255 bits : 0-5 V)

          // Positive voltage
          voltagepositive = voltage + amplitude;
          val = (255/2)*(voltagepositive+1);
          
          //analogWrite(a,val); // For Arduino
          ledcWrite(LED_PIN, val*4); // For ESP
    
          delay(pulsewidth);

          for (int i = 0; i < numSamples; i++) {

            sum += analogRead(A2);
            delayMicroseconds(100); // Small delay between samples to allow for ADC stabilization
          }

          c_pos = sum / numSamples;
          sum = 0;

          Serial.print(voltage);

          // Negative voltage
          voltagenegative = voltage - amplitude;
          val = (255/2)*(voltagenegative+1);

          ledcWrite(LED_PIN, val*4); // For ESP
          
          delay(pulsewidth);

          for (int i = 0; i < numSamples; i++) {
          sum += analogRead(A2);
          delayMicroseconds(100); // Small delay between samples to allow for ADC stabilization
          }
          c_neg = sum / numSamples;
          sum = 0;

          Serial.print(" ");
          current_diff = c_pos-c_neg;
          Serial.println(current_diff); 

          currentMillis = millis();
          
          if ((currentMillis - previousMillis >= TXdelay) && start_value == 1) { // send after delay
            previousMillis = currentMillis;
            writePotentiostatStr(); // write data for plotting        
          }

          } // Closing voltage 'for' loop
        
        n=n+1;
        } // Closes white loop
      } // Closes for pos loop
      // End of potentiostat code


      } // End of BLE characteristic

  } // End of BLE central connection
} // End of BLE central
} // End of the main loop



void writePotentiostatStr(){
  float currentdata; // pre-alloc for measurements happening now (current)
  String strToPrint = "";
  currentdata = current_diff;
  strToPrint+= String(currentdata,16); strToPrint+="\n"; // uncomment to send altitude
  writeBLE(strToPrint); // send string over BLE
}

void writeBLE(String message){
  byte plain[message.length()]; // message buffer
  message.getBytes(plain, message.length()); // convert to bytes
  myCharacteristic.writeValue(plain,message.length()); // writing to BLE
}

void readBLE(String message){
  byte plain[message.length()]; // message buffer
  message.getBytes(plain, message.length()); // convert to bytes
  myCharacteristic.readValue(plain,message.length()); // writing to BLE
}
