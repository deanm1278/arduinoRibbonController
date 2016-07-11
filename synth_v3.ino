/*
* Arduino Ribbon Synth MIDI controller
* ------------------------------------
* ©2015 Dean Miller
*/

#include <EEPROM.h>
#include <Wire.h>
#include "Adafruit_Trellis.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN_LED   13
#define NEO_PIN   11
#define N_PIXELS  2

#define S0        A0
#define S1        A1
#define S2        A11
#define S3        A6

#define T0        A2
#define T1        A3
#define T2        A4
#define T3        A5

#define JSX       A7
#define JSY       A8

#define JSSEL     7

#define THRESH    600
#define N_STR     4
#define N_FRET    20
#define N_KEYS    16
#define S_PAD     3
#define T_PAD     300

short fretDefs[N_STR][N_FRET];

short T_vals[N_STR];
bool T_active[] = {false, false, false, false}; //is it currently active
short T_hit[N_STR];                             //has it been hit on this loop
int T_pins[] = {T0, T1, T2, T3};

short S_vals[N_STR];                            //current sensor values
short S_old[N_STR];                             //old sensor values for comparison
int S_active[N_STR];                            //currently active notes
int S_pins[] = {S0, S1, S2, S3};

unsigned int fretTouched[N_STR];

bool lightsActive = false;

bool transposed = false;

//E A D G
int offsets_default[] = {40, 45, 50, 55};

//B E A D
int offsets_transposed[] = {35, 40, 45, 50};

//default offsets
int offsets[] = {40, 45, 50, 55};

Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet trellis =  Adafruit_TrellisSet(&matrix0);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(N_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

bool stickActive = false;
bool stickState = false;
bool btnState = false;
int stickZeroX;
int stickZeroY;

int channel = 0;
int channelButtons[] = {4, 5, 6, 7};

unsigned long last_read;

//map out all the trellis button functions here
void (*fns[]) (int i) = {
  edgeLightsBlue,    //0
  edgeLightsRed,     //1
  unset,             //2
  unset,             //3
  unset,             //4
  unset,             //5
  unset,             //6
  unset,             //7
  unset,             //8
  unset,             //9
  unset,             //10
  unset,             //11
  unset,             //12
  unset,             //13
  unset,             //14
  transpose,         //15
};

void setup() {
  //read fret definitions from EEPROM
  for (int i=0; i<N_STR; i++){
    for (int j=0; j<N_FRET; j++){
      fretDefs[i][j] = EEPROMReadShort(j * sizeof(short) + (N_FRET*i*sizeof(short)));
    }
  }
  Serial1.begin(31250);
  //Serial.begin(9600);
  for(int i=0; i<N_STR; i++){
    pinMode(T_pins[i], INPUT);
    pinMode(S_pins[i], INPUT);
  }

  pinMode(JSX, INPUT);
  pinMode(JSY, INPUT);
  pinMode(JSSEL, INPUT);
  digitalWrite(JSSEL, HIGH);

  pinMode(PIN_LED, OUTPUT);

  trellis.begin(0x70);
    // light up all the LEDs in order
  for (uint8_t i=0; i<N_KEYS; i++) {
    trellis.setLED(i);
    trellis.writeDisplay();    
    delay(50);
  }
  // then turn them off
  for (uint8_t i=0; i<N_KEYS; i++) {
    trellis.clrLED(i);
    trellis.writeDisplay();    
    delay(50);
  }
  pixels.begin();
  //calibrate joystick
  stickZeroX = analogRead(JSX);
  stickZeroY = analogRead(JSY);
}

void loop() {
  //we can only read the trellis every 30ms which is ok
  if(millis() - last_read >= 30){
    readTrellis();
    last_read = millis();
  }

  readControls();
  determineFrets();
  legatoTest();
  pickNotes();
  readJoystick();
  cleanUp();
  delay(5);
}

void pickNotes(){
  for (int i=0; i<N_STR; i++){
    if(T_hit[i]){
      if(S_active[i]){
        //turn off active note on this string
        noteOff(0x80 + channel, S_active[i]);
      }
      S_active[i] = fretTouched[i] + offsets[i];
      noteOn(0x90 + channel, S_active[i], 100);
    }
  }
}

void legatoTest(){
  for(int i=0; i<N_STR; i++){
    if(S_active[i]){
      int note = fretTouched[i] + offsets[i];
      if(note != S_active[i] && (fretTouched[i] || T_active[i])){
        
        noteOn(0x90 + channel, note, 100);
        noteOff(0x80 + channel, S_active[i]);

        S_active[i] = note;
      }
    }
  }
}

void cleanUp(){
  for (int i=0; i<N_STR; i++){
    if(S_active[i] && !fretTouched[i] && !T_active[i]){
        noteOff(0x80 + channel, S_active[i]);
        S_active[i] = 0;
    }
  }
}

void readControls(){
  //read the strings and the triggers
  for (int i=0; i<N_STR; i++){
    T_hit[i] = checkTriggered(i);
    S_vals[i] = analogRead(S_pins[i]);
  }
}

void determineFrets () {
   //---------Get Fret Numbers------
 for (int i=0; i< N_STR; i++) {
 
   short s_val = S_vals[i];
    
    //check for open strings
    if (s_val == 0) {
      S_old[i] = s_val;
      fretTouched[i]=0;
    }
    else if(s_val >= fretDefs[i][0] && abs((int)s_val-(int)S_old[i]) > S_PAD){
      S_old[i] = s_val;
      fretTouched[i] = 1;
    }
    else{
      //loop through the array of fret definitions
      for (int j=1; j<N_FRET; j++) {
        int k = j - 1;
        if (s_val >= fretDefs[i][j] && 
            s_val < fretDefs[i][k] &&
            abs((int)s_val-(int)S_old[i]) > S_PAD) {
              
              S_old[i] = s_val;
              fretTouched[i] = j + 1;
            }
      }
    }
  }
}

void readTrellis(){
  // If a button was just pressed or released...
  if (trellis.readSwitches()) {
    // go through every button
    for (uint8_t i=0; i<N_KEYS; i++) {
      fns[i](i);
    }
    // tell the trellis to set the LEDs we requested
    trellis.writeDisplay();
  }
}

void unset(int i){
  //this function doesn't even do anything!!
}

void light(int i){
  //light up the button 
  
  // if it was pressed, turn it on
  if (trellis.justPressed(i)) {
    trellis.setLED(i);
  } 
  // if it was released, turn it off
  if (trellis.justReleased(i)) {
    trellis.clrLED(i);
  }
}

void calibrate(int btn){
  if (trellis.justPressed(btn)) {
  //Serial.println("calibrating...");
  for (int i=0; i<N_STR; i++) {
    //Flash the LED too indicate calibration
    setLED(btn);
    delay(100);
    clrLED(btn);
    delay(100);
    setLED(btn);
    delay(100);
    clrLED(btn);
    delay(100);
    setLED(btn);
  
    short sensorMax = 0;
    short sensorMin = 1023;
    short val;
    
      //loop through the array of fret definitions
      for (int j=N_FRET - 1; j>=0; j--) {
      
        int response = false;
      
        //wait for response
        while (!response) {
          
           if (checkTriggered(i)) {
              val = short(analogRead(S_pins[i]));
              response = true;
          }
          delay(10);
        }
      
        //write to memory
        clrLED(btn);
        int addr = j * sizeof(short) + (N_FRET*i*sizeof(short));
        //Serial.print("Writing ");
        //Serial.print(val);
        //Serial.print(" to address: ");
        //Serial.println(addr);
        EEPROMWriteShort(addr, val);
        
        delay(100);
        setLED(btn);
      }
    for (int j=0; j<N_FRET; j++) {
      short v = EEPROMReadShort(j * sizeof(short) + (N_FRET*i*sizeof(short)));
      fretDefs[i][j] = v;
    }
  }
  
  clrLED(btn);
  }
}

void EEPROMWriteShort(int address, int value){
      //One = Most significant -> Two = Least significant byte
      byte two = (value & 0xFF);
      byte one = ((value >> 8) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, two);
      EEPROM.write(address + 1, one);
}

short EEPROMReadShort(int address){
      //Read the 2 bytes from the eeprom memory.
      long two = EEPROM.read(address);
      long one = EEPROM.read(address + 1);

      //Return the recomposed short by using bitshift.
      return ((two << 0) & 0xFF) + ((one << 8) & 0xFFFF);
}

void setLED(int i){
  trellis.setLED(i);
  trellis.writeDisplay();
}

void clrLED(int i){
  trellis.clrLED(i);
  trellis.writeDisplay();
}

//check if a trigger has been hit. Return 0 if not, the value of the trigger if it has
short checkTriggered(int i){
  short v = analogRead(T_pins[i]);
  T_vals[i] = v;
  short ret = 0;
  if(!T_active[i] && v > THRESH){
    T_active[i] = true;
    ret = v;
  }
  else if(T_active[i] && v < THRESH - T_PAD){
    T_active[i] = false;
  }
  return ret;
}

void edgeLightsBlue(int btn){
  if (trellis.justPressed(btn)) {
    edgeLight(0, 0, 255);
  }
}

void edgeLightsRed(int btn){
  if (trellis.justPressed(btn)) {
    edgeLight(255, 0, 0);
  }
}

void edgeLight(int red, int green, int blue){
  lightsActive = !lightsActive;
    if(lightsActive){
      for(int i=0;i<N_PIXELS;i++){
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        pixels.setPixelColor(i, pixels.Color(red, green, blue));
   }
   pixels.show();
    }
    else{
      for(int i=0;i<N_PIXELS;i++){
        pixels.setPixelColor(i, pixels.Color(0,0,0)); // turn off
      }
      pixels.show();
    }
}

void transpose(int btn){
  if (trellis.justPressed(btn)) {
  if(transposed){
    for (int i=0; i<N_STR; i++) {
      offsets[i] = offsets_default[i];
    }
    trellis.clrLED(btn);
  }
  else{
    for (int i=0; i<N_STR; i++) {
      offsets[i] = offsets_transposed[i];
    }
    trellis.setLED(btn);
  }
  transposed = !transposed;
  }
}

/*
void channel1(int btn){
  if (trellis.justPressed(btn)) {
    
  }
}

void setChannel(int c){
   for(int i=0; i<sizeof(channelButtons)/sizeof(int); i++){
    channel
  }
  channel = c;
}
*/


void readJoystick(){
   if (digitalRead(JSSEL) == LOW) {
    //activate joystick
    if (!btnState) {
      //make sure modwheel value is set to 0 when stick is off
      if (stickActive) controllerChange(1, 0);
      stickActive = !stickActive;
      Serial.println(stickActive);
    }
    btnState = true;
  }
  //reset once stick is no longer beingnote pressed
  if (digitalRead(JSSEL) == HIGH && btnState) btnState = false;
  
  if (stickActive) {
    //read positions from center
    float xPos = map(analogRead(JSX), stickZeroX, 1023, 0, 127);
    float yPos = map(analogRead(JSY), stickZeroY, 1023, 0, 127);
    
    //get absolute position from center
    float z = sqrt(sq(xPos) + sq(yPos));
    int stickVal = (int)constrain(z, 0, 127);
    
    if (stickVal > 0) {
      stickState = true;
      controllerChange(1, stickVal);
    }
    else if (stickState && stickVal == 0) {
      stickState = false;
      controllerChange(1, 0);
    }
  }
}

//note-on message
void noteOn(int cmd, int pitch, int velocity) {
  
  Serial1.write(byte(cmd));
  Serial1.write(byte(pitch));
  Serial1.write(byte(velocity));
  digitalWrite(PIN_LED, HIGH);
}
//note-off message
void noteOff(int cmd, int pitch) {
  
  Serial1.write(byte(cmd));
  Serial1.write(byte(pitch));
  Serial1.write(byte(0));
  digitalWrite(PIN_LED, LOW);
}

//Sends controller change to the specified controller
void controllerChange(int controller, int value) {
  Serial1.write(byte(0xb2));
  Serial1.write(byte(controller));
  Serial1.write(byte(value));
}

