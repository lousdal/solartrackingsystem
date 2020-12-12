// #include <avr/eeprom.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

// varibles for solar panels output
int solar_voltage_pin = 4;  // ADC pin 4 for the measure resistor
int solar_voltage_drop_pin = 5; // ADC pin 5 for the voltage drop 

float solar_voltage_adc_value = 0;          //solar voltage adc
float solar_voltage_drop_adc_value = 0;     //solar measure resistor voltage drop adc

float new_solar_voltage_adc_value = 0;      //solar voltage adc filtered
float new_solar_voltage_drop_adc_value = 0; //solar measure resistor voltage drop adc filtered

float new_solar_voltage_value = 0;          //convert adc to voltage
float new_solar_voltage_drop_value = 0;     //convert adc to voltage

float solar_amps = 0; 
float solar_watt = 0;
float new_solar_watt = 0;                  // solar watt filtered
int solar_watt_int = 0;

// Variables for the Servo
byte servoBroadcastID = 0xFE; // broadcast ID for communication to all the servos
byte servoTiltID0 = 0x00; //ID of servo0 - tilt servo - on top in our setup
byte servoPanID1 = 0x01; //ID of servo1 - pan servo - on bottom in our setup
int servoPos; // servo position to set the servo. 
byte bytePosLow; // Lowest byte of Goal Position
byte bytePosHigh; // Highest byte of Goal Position

int startServoTiltPos = 300; // start tilt position of the solar panels. also acts a variable for old tilt servo position. 300 could be start position in east // 200 = 0 degrees // 512 = 90 degrees
int servoTiltPos = 300; // old servo tilt position
int newServoTiltPos;  // new servo tilt position
int startServoPanPos = 700; // start pan position for the solar panels. also acts a variable for old pan position. 700 could be start position in west //200 west 512 south 824 east
int servoPanPos = 700;  // old servo tilt position
int newServoPanPos; // new servo tilt position

// Variables for the photodiodes
int tiltUp = 0; // photodiode ADC pin 0
int tiltDown = 1; // photodiode ADC pin 1
int panLeft = 2;  // photodiode ADC pin 2
int panRight = 3; // photodiode ADC pin 3

float tiltUpVal = 0;  // variable to store the value read
float tiltDownVal = 0;  // variable to store the value read
float panLeftVal = 0; // variable to store the value read
float panRightVal = 0;  // variable to store the value read

float newTiltUpVal = 0; // variable to store the new value
float newTiltDownVal = 0; // variable to store the new value
float newPanLeftVal = 0; // variable to store the new value
float newPanRightVal = 0; // variable to store the new value

// variables for storing in eeprom
// uint16_t eepromAddr = 0;
// int readSavedServoTiltPos = 0;
// int readSavedServoPanPos = 0;

//variables for converting the servo position to degrees
float panpos_degree = 0;
String panpos_direction = "intet";
int tiltpos_degree = 0;

// variables for calculating watt hour pr day
int sampleWatt;
int wattMinSum;
int wattHourSum;
int wattDaySum;
int meanWattHourDay;

// variables for counters
int servoCounter;
int displayCounter;
int sampleCounter;
int minCounter;
int hourCounter;
//int eepromCounter;

bool isItDay = false;

// Master to slave instruction packet to move the servo. 512 is the center value for both tilt and pan servo.
void moveServoToPos(byte servoID, int servoPos){
  byte bytePosHigh = servoPos >> 8; // throw the 8 least sigificant bits away, so only the 2 most significant bits are left.
  byte bytePosLow = servoPos % 256; // use modulus 256 to get the 8 bit remainder
  byte checkSum = ~lowByte(servoID + 0x05 + 0x03 + 0x1E + bytePosLow + bytePosHigh); // Check Sum = ~ ( ID + Length + Instruction + Parameter1 + … Parameter N) - lowByte is cause if number calculated is higher than 255 it will only choose the lowByte
  digitalWrite(3, HIGH); // set RS-485 into transmitting mode
  delay(10);
  Serial.write(0xFF); // Start byte - begining of packet
  Serial.write(0xFF); // Start byte - begining of packet
  Serial.write(servoID); // ID of RX-28. Can use 254 IDs from 0 to 253.
  Serial.write(0x05); // Length of the packet. Is calculated as “the number of Parameters (N) + 2”.
  Serial.write(0x03); // Instruction to RX-28. 0x01 Ping. 0x02 Read data. 0x03 Write data. 0x04 Reg write. 0x05 Action. 0x06 Reset. 0x83 Sync write.
  Serial.write(0x1E); // Parameter1 - start address of GoalPosition 0x1E
  Serial.write(bytePosLow); // Parameter2 - Lowest byte of Goal Position
  Serial.write(bytePosHigh); // Parameter3 - Highest byte of Goal Position
  Serial.write(checkSum); // Check sum
  Serial.flush(); // to be sure that all that is written to serial is send
  digitalWrite(3, LOW); // set RS-485 into receiving mode
  delay(10);
}

String panpos_convert(int panpos){
  panpos_direction = "error";
  
  panpos_degree = (panpos * -0.28846154) + 237.7; // formula for converting servo pan position to degrees.

  if (panpos_degree > -22.5 and panpos_degree < 22.5){ // if statement converting degrees to direction
  panpos_direction = "E";
  }
  if (panpos_degree > 22.5 and panpos_degree < 67.5){ // if statement converting degrees to direction
  panpos_direction = "SE";
  }
  if (panpos_degree > 67.5 and panpos_degree < 112.5){ // if statement converting degrees to direction
  panpos_direction = "S";
  }
  if (panpos_degree > 112.5 and panpos_degree < 157.5){ // if statement converting degrees to direction
  panpos_direction = "SW";
  }
  if (panpos_degree > 157.5 and panpos_degree < 202.5){ // if statement converting degrees to direction
  panpos_direction = "W";
  }
  return panpos_direction;  
  }
  
int tiltpos_convert(int tiltpos){
  tiltpos_degree = (tiltpos * 0.28846154) - 57.7; // formula for converting servo tilt position to degrees.
  return tiltpos_degree;  
  }

void setup() {
  pinMode(3, OUTPUT); // Sets pin2 to be RTS (Request To Send) pin. If HIGH transmit. If LOW receive.
  pinMode(LED_BUILTIN, OUTPUT); // set built in LED to an output. Used to debug the code.

  Serial.begin(57143); // Begins serial communication to stream to the RX-28's See manual pg. 23 about baudrate. Set lower if causing problems or to 57143.
  Serial.flush(); // to be sure that all that is written to serial is send

  moveServoToPos(servoTiltID0, startServoTiltPos); // move servo to pan start position
  moveServoToPos(servoPanID1, startServoPanPos);  // move servo to tilt start position
  
  lcd.begin(); // initialize the LCD
  lcd.backlight(); // turn on the backlight
  lcd.clear();  // clears the display
  lcd.print("System is "); // prints to the display
  lcd.setCursor(0,1); // column_index, row_index. go to start of 2nd line
  lcd.print("powering up!"); // prints to the display
}

void loop() {

  delay(10);
  
  // Increment counters
  //eepromCounter++;
  //serialCounter++;
  displayCounter++;
  
  // Read sensors
  tiltUpVal = analogRead(tiltUp);    // read the input pin 0
  tiltDownVal = analogRead(tiltDown);    // read the input pin 1
  panLeftVal = analogRead(panLeft);    // read the input pin 2
  panRightVal = analogRead(panRight);    // read the input pin 3

  solar_voltage_adc_value = analogRead(solar_voltage_pin);              // read the solar panel voltage output
  solar_voltage_drop_adc_value = analogRead(solar_voltage_drop_pin);    // read the solar panel voltage drop of the mesaure resistor output  // amplifered with 45 times

  // Filter to smoothen the sensor values
  newTiltUpVal = (0.95 * newTiltUpVal) + (0.05 * tiltUpVal); // low pass filter to smoothen the value
  newTiltDownVal = (0.95 * newTiltDownVal) + (0.05 * tiltDownVal); // low pass filter to smoothen the value
  newPanLeftVal = (0.95 * newPanLeftVal) + (0.05 * panLeftVal);  // low pass filter to smoothen the value
  newPanRightVal = (0.95 * newPanRightVal) + (0.05 * panRightVal); // low pass filter to smoothen the value

  //solar panels output
  new_solar_voltage_adc_value = (0.95 * new_solar_voltage_adc_value) + (0.05 * solar_voltage_adc_value); // low pass filter to smoothen the value
  new_solar_voltage_drop_adc_value = (0.95 * new_solar_voltage_drop_adc_value) + (0.05 * solar_voltage_drop_adc_value);  // low pass filter to smoothen the value

  new_solar_voltage_value = new_solar_voltage_adc_value * 0.0048828125;            //CALCULATE VOLTAGE from adc value. 5V / 1024 = 0.0048828125
  new_solar_voltage_drop_value = new_solar_voltage_drop_adc_value * 0.0048828125;   //CALCULATE VOLTAGE from adc value. 5V / 1024 = 0.0048828125

  solar_amps = ((new_solar_voltage_drop_value / 39) / 0.2); // 0.2 is the value of the measure resistor. 39 is the amplification rate
  solar_watt = solar_amps * new_solar_voltage_value * 1000; // the power is calculated using P = I * V
  new_solar_watt = (0.95 * new_solar_watt) + (0.05 * solar_watt); // low pass filter to smoothen the value
  solar_watt_int = new_solar_watt;   // converts new_solar_watt to an int to send to the display
  
  if(new_solar_watt > 0.2){ // if new_solar_watt is above 0.2
    isItDay = true;   // day is set to true
  }
  
  if(isItDay == true){  // if its day
    sampleCounter++;  // sampleCounter increments
    servoCounter++; // servoCounter increments
  }
  
  if(sampleCounter == 10){  // if sampleCounter is 10. then 100 ms has passed. 100 ms = 1 min IRL.
    sampleWatt = new_solar_watt;
    wattMinSum += sampleWatt; // sampleWatt is added everytime

    minCounter++; // minCounter is incrementing to count every minute
    sampleCounter = 0;  // sampleCounter is reset to make new samples

    if(minCounter == 60){ // if minCounter is 60 then an hour has passed 
      wattHourSum = wattMinSum / 60;  // calculating watt hour sum
      wattMinSum = 0; // reset counter
      minCounter = 0; // minCounter is reset to count to 60 mins again
      hourCounter++;  //hourCounter is incrementing to count the hours

      wattDaySum += wattHourSum;  // wattHourSum is added everytime
      meanWattHourDay = wattDaySum / hourCounter; // calculating the meanWattHourDay by taking the wattDaySum dividing it with the hourCounter

      if(wattHourSum < 0.2){  // if wattTimeSum is below 0.2
        isItDay = false;  // the sun has set and it is night
        wattMinSum = 0; // wattMinSum is reset
        wattDaySum = 0; // wattDaySum is reset
        hourCounter = 0;  // hourCounter is reset
        moveServoToPos(servoTiltID0, startServoTiltPos); // move the tilt servo to start position when day is ended
        moveServoToPos(servoPanID1, startServoPanPos);  // move the pan servo to start position when day is ended
        servoTiltPos = startServoTiltPos; // setting the servoTiltPos to the start value
        servoPanPos = startServoPanPos; // setting the servoPanPos to the start value
      }
    }
  }

  if(servoCounter == 6){
      if(newPanLeftVal-20 > newPanRightVal and servoPanPos < 950){  // if statement comparing the two pan values
          newServoPanPos = servoPanPos + 3;  // incrementing the position with 1
          moveServoToPos(servoPanID1, newServoPanPos); // move the pan servo to the new position
          servoPanPos = newServoPanPos;  // setting the newly set servo position as the old servo position
          }
      if(newPanRightVal-20 > newPanLeftVal and servoPanPos > 50){  // if statement comparing the two pan values
          newServoPanPos = servoPanPos - 3;  // decrementing the position with 1
          moveServoToPos(servoPanID1, newServoPanPos); // move the pan servo to the new position
          servoPanPos = newServoPanPos;  // setting the newly set servo position as the old servo position
          }
      if(newTiltUpVal-20 > newTiltDownVal and servoTiltPos < 650 ){ // if statement comparing the two tilt values
          newServoTiltPos = servoTiltPos + 3; // incrementing the position with 1
          moveServoToPos(servoTiltID0, newServoTiltPos); // move the tilt servo to the new position
          servoTiltPos = newServoTiltPos;  // setting the newly set servo position as the old servo position
          }
      if(newTiltDownVal-20 > newTiltUpVal and servoTiltPos > 300){ // if statement comparing the two tilt values
          newServoTiltPos = servoTiltPos - 3;  // decrementing the position with 
          moveServoToPos(servoTiltID0, newServoTiltPos); // move the tilt servo to the new position
          servoTiltPos = newServoTiltPos;  // setting the newly set servo position as the old servo position
          } 
      servoCounter = 0; // reset the counter
      }  
    
  if (displayCounter == 20){
      lcd.clear(); // clears the display
      lcd.print("LmW:"); // prints to the display
      lcd.print(solar_watt_int);  // prints to the display
      lcd.print(" TWD:"); // prints to the display
      lcd.print(wattDaySum);  // prints to the display
      lcd.print(" ");
      lcd.print(hourCounter); // prints to the display
      lcd.setCursor(0,1); // column_index, row_index. go to start of 2nd line
      lcd.print("Hr:"); // prints to the display
      lcd.print(wattHourSum);  // prints to the display
      lcd.print(" MD:"); // prints to the display
      lcd.print(meanWattHourDay);  // prints to the display
      //lcd.print("Pan:"); // servoPanPos
      //lcd.print(panpos_direction); // print the variable servoPanPos
      //lcd.print(" Tilt:"); // servoTiltPos
      //lcd.print(tiltpos_degree);  // prints the variable servoTiltPos
      displayCounter = 0;
      }

/* // used for debugging displaying variables in the serial monitor
    if(serialCounter > 100){

      panpos_convert(servoPanPos);
      tiltpos_convert(servoTiltPos);

      Serial.flush();
      Serial.begin(9600);
      
      Serial.println(" ");
      Serial.print("Servo Tilt Position: ");             // debug value
      Serial.println(tiltpos_degree);             // debug value
    
      Serial.print("Servo Pan Position: ");             // debug value
      Serial.println(panpos_direction);             // debug value
    
      Serial.print("Solar Amps Output: ");             // debug value
      Serial.println(solar_amps);             // debug value

      Serial.print("Solar Watt Output: ");             // debug value
      Serial.println(new_solar_watt);             // debug value

      Serial.print("Solar Voltage Output: ");             // debug value
      Serial.println(new_solar_voltage_value);             // debug value
      
      Serial.println(" ");
      Serial.println(" ");

      Serial.flush();
      Serial.begin(57143);
      serialCounter = 0;

    }
    
// started on saving a day to the eeprom. More work has to be done for using this part properly.
    if(eepromCounter == 100){
      while (!eeprom_is_ready()); // Wait for EEPROM to be ready
      cli(); // disable/clear interupts
      eeprom_write_word(eepromAddr, servoTiltPos);  // Write to eeprom
      sei(); // re-enable/set interupts
      
      while (!eeprom_is_ready()); // Wait for EEPROM to be ready
      cli(); // disable/clear interupts
      readSavedServoTiltPos = eeprom_read_word(eepromAddr); // Read from eeprom
      sei(); // re-enable/set interupts
      
      Serial.print("servoTiltPos: ");
      Serial.print(readSavedServoTiltPos);
      Serial.print(" saved to EEPROM address");
      Serial.println(eepromAddr);
      eepromAddr++;
      
      while (!eeprom_is_ready()); // Wait for EEPROM to be ready
      cli(); // disable/clear interupts
      eeprom_write_word(eepromAddr, servoPanPos); // Write to eeprom
      sei();// re-enable/set interupts

      while (!eeprom_is_ready()); // Wait for EEPROM to be ready
      cli(); // disable/clear interupts
      readSavedServoPanPos = eeprom_read_word(eepromAddr); // Read from eeprom
      sei(); // re-enable/set interupts
      
      Serial.print("servoPanPos: ");
      Serial.print(readSavedServoPanPos);
      Serial.print(" saved to EEPROM address");
      Serial.println(eepromAddr);
      eepromAddr++;
      eepromCounter = 0;
    }
    */
  }
