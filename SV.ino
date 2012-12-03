#include <LiquidCrystal.h>
#include <OneWire.h>
#include <PID_v1.h>

//DS18S20 Temperature chip i/o

bool firstRun;
bool deviceFound;
int setTemp = 60;
float  curTemp = 0;
float  lastTemp = 0;
bool settingTemp = false;
float startSettingTime;
byte addr[8];

float lastKnob = 0;
float curKnob = 0;

const int setButtonPin = 9;
const int outputPin = 7;

double SetPoint, Input, Output;
int WindowSize = 5000;
unsigned long windowStartTime;

bool waitingForTemp = false;
OneWire ds(13);  // on pin 13
LiquidCrystal lcd(12, 11, 5, 4, 3, 2); 
PID tempPID(&Input, &Output, &SetPoint,2,5,1, DIRECT);


void setup(void) {
  // initialize inputs/outputs
  // start serial port
  Serial.begin(9600);
  Serial.print("Initializing.\n");
  lcd.begin(16, 2);
  firstRun = true;
  deviceFound = false;
  pinMode(setButtonPin, INPUT);
  pinMode(outputPin, OUTPUT);


  windowStartTime = millis();
  //initialize the variables we're linked to
  SetPoint = 00;
  //tell the PID to range between 0 and the full window size
  tempPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  tempPID.SetMode(AUTOMATIC);
}


float dataToTemp(byte * data) {
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;
  int Sign = (SignBit ? -1 : 1);
  return Sign * (Whole + Fract/100.);
}


float readKnob() {
  int sensorPin = 0;
  int sensorValue = analogRead(sensorPin); 
  return 40 + 60. * sensorValue / 1024;
}

void printData() {
  lcd.setCursor(0, 1);
  lcd.print("SET:");
  lcd.print((int)setTemp);
  lcd.print(" CUR:");
  lcd.print((int)(curTemp+.5));


  lcd.setCursor(0,0);
  if(settingTemp) {
    if(curKnob>90) {
      lcd.print("IT RUINS IT");
    }else {
      lcd.print("SET: ");
      lcd.print(curKnob);
      lcd.print("    ");
    }
  } else
  {
    lcd.print("PWR:");
    lcd.print((int)(100.*Output/WindowSize));
    lcd.print(" TIME:");
    lcd.print(millis()/60000);
  }
}



void
findThermoDevice() {
  if ( !ds.search(addr)) {
    if(firstRun)
      Serial.print("C0.\n"); //no more addresses
    ds.reset_search();
    firstRun = false;
    return;
  }


  Serial.print("R=");
  for( int i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.print("C1\n"); //crc not valid
    return;
  }

  if ( addr[0] == 0x10) {
    Serial.print("C2\n"); //ds18s20
  }
  else if ( addr[0] == 0x28) {
    Serial.print("C2\n"); //ds18b20
  }
  else {
    Serial.print("C3\n"); //not recognized
    return;
  }
  deviceFound = true;
}


void readTemp() {
  byte i;
  byte present = 0;
  byte data[12];

  if(waitingForTemp) {
    if(ds.read()==0) //still waiting, go back
      return;
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad


    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
    }
    lastTemp = curTemp;
    curTemp = dataToTemp(data);
    waitingForTemp = false;
  }



    ds.reset();
    ds.select(addr);
    ds.write(0x44,0);         // start conversion, with parasite power OFF
    waitingForTemp = true;

}

void loop(void) {


  if(!deviceFound)  {
    findThermoDevice();
    return;
  }


  lastKnob = curKnob;
  curKnob = readKnob();

  if(abs(lastKnob-curKnob)>1.0)  {
    settingTemp = true;
    startSettingTime = millis();
  } else {
    if(millis()-startSettingTime>5000)
      settingTemp = false;
  }

  if(settingTemp) {
    bool down = !digitalRead(setButtonPin);
    if(down) {
      setTemp = curKnob; 
      settingTemp = false;
    }

  }

  readTemp();  

  Input = curTemp;
  SetPoint = setTemp;
  tempPID.Compute();
  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime) 
    digitalWrite(outputPin,HIGH);
  else 
    digitalWrite(outputPin,LOW);

  printData();    

}
