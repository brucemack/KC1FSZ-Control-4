// Control board for the Peppermint II SSB Tranceiver
// Bruce MacKinnon KC1FSZ
// 16-June-2018
// 
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

// OLED display related
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// The Etherkit library used to control the Si5351/a
#include <si5351.h>

// KC1FSZ utilities
#include <DebouncedSwitch2.h>
#include <RotaryEncoder.h>
#include <ClickDetector.h>
#include <Utils.h>
#include <VSWRMeterState.h>
#include <IntervalTimer2.h>
#include <Averager.h>

// This is used to determine whether the EEPROM storage is valid
#define MAGIC_NUMBER 2727

Adafruit_SSD1306 display(4);

Si5351 si5351;

// ----- I/O Pin Setup ------------------------------------------------------------------------
//
// Tuning rotary encoder pins
#define ENCODER0_PHASE0_PIN     2
#define ENCODER0_PHASE1_PIN     3
#define ENCODER0_PUSH_PIN       4
// Red command buttons
#define COMMAND_BUTTON0_PIN     5
#define COMMAND_BUTTON1_PIN     6
// Digital pot control
#define AFGAIN_SS_PIN           7
#define SPI_MOSI_PIN            8
#define SPI_CLK_PIN             9
// T/R controls 
#define PTT_BUTTON_PIN          10
#define TR_RELAY_PIN            11

// Volume control input
#define AFGAIN_PIN              A0 // (pin 14)
// VSWR meter inputs 
#define VSWR_FWD_PIN            A1 // (pin 15)
#define VSWR_REV_PIN            A2 // (pin 16)
//
// ----------------------------------------------------------------------------------------------

enum Mode { VFO = 0, VFO_OFFSET, BFO, CAL, VFO_POWER, BFO_POWER, VOL };
const char* modeTitles[] = { "VFO", "VFO+", "BFO", "CAL", "VFOPwr", "BFOPwr", "VOL" };
Mode mode = VFO;
Mode savedMode = VFO;

const unsigned long steps[] = { 500, 100, 10, 1, 1000000, 100000, 10000, 1000 };
const char* stepTitles[] = { "500 Hz", "100 Hz", "10 Hz", "1 Hz", "1 MHz", "100 kHz", "10 kHz", "1 kHz" };
const uint8_t maxStepIndex = 7;

// 40m band limitations used for scanning (phone portion only)
const unsigned long minDisplayFreq = 7125000L;
const unsigned long maxDisplayFreq = 7300000L;

// Tuning 
DebouncedSwitch2 db2(5L);
DebouncedSwitch2 db3(5L);
RotaryEncoder vfoEnc(&db2,&db3,100L);
DebouncedSwitch2 db4(10L);
ClickDetector vfoClick(&db4);

DebouncedSwitch2 commandButton0(10L);
DebouncedSwitch2 commandButton1(10L);

unsigned long vfoFreq = 7000000;
long vfoOffsetFreq = 11998000;
unsigned long bfoFreq = 11998000;
long calPpm = 0;
uint8_t stepIndex = 0;
uint8_t vfoPower = 0;
uint8_t bfoPower = 0;

// Scanning related.
bool scanMode = false;
// This controls how fast we scan
IntervalTimer2 scanInterval(150);

// AF Gain Related

// The current value of the gain (0-127).  What is written to the digital pot.
unsigned int AFGain = 0;
// The number of steps the gain control can make
const unsigned int AFGainSteps = 32;
// The last value read from the gain control. Used to determine if a change has happened.
int previousAFGainControlSample = 0;
// Used for averaging the gain control value (LPF)
int AFGainSamples[32];
Averager AFGainAverager(AFGainSamples,32);

// Used to control the display of the volume setting
IntervalTimer2 AFGainChangeTimeout(2000UL);

// T/R control related
bool transmitMode = false;
DebouncedSwitch2 pttButton(10L);

// VSWR meter related
VSWRMeterState vswrState(VSWR_FWD_PIN,VSWR_REV_PIN);
// This controls how oftehn we refresh the display
IntervalTimer2 vswrDisplayInterval(1000);

unsigned long getMH(unsigned long f) {
  return f / 1000000L;
}

unsigned long getKH(unsigned long f) {
  return (f / 1000L) % 1000L;
}

unsigned long getH(unsigned long f) {
  return f % 1000L;
}

// ----- SPI Bus ----------------------------------------------------------------------

void spiWriteBit(int mosiPin,int clkPin,int bit) {
  // Setup the data
  digitalWrite(mosiPin,bit);
  // The slave samples when clock goes high
  digitalWrite(clkPin,HIGH);
  digitalWrite(clkPin,LOW);
}

void spiWriteByte(int mosiPin,int clkPin,int b) {
  int work = b;
  for (int i = 0; i < 8; i++) {
    // Always focus on the MSB 
    spiWriteBit(mosiPin,clkPin,(work & 0x80) ? 1 : 0);
    work = work << 1;
  }
}

// NOTE: There are no delays of any kind.  The assumption is that the chip 
// can keep up with the maxium data rate.
//
void spiWrite(int ssPin,int mosiPin,int clkPin,int address, int value) {
  // Take the SS pin low to select the chip:
  digitalWrite(ssPin,LOW);
  // Address
  spiWriteByte(mosiPin,clkPin,address);
  // Data
  spiWriteByte(mosiPin,clkPin,value);
  // Take the SS pin high to de-select the chip:
  digitalWrite(ssPin,HIGH); 
}

// ----- Display -----------------------------------------------------------------------

void updateDisplayReceive1() {

  int startX = 10;
  int y = 17;

  display.setTextSize(2);
  display.setTextColor(WHITE);
  char buf[4];

  // Render frequency
  unsigned long f = 0;
  boolean neg = false;  

  if (mode == VFO) {
    f = vfoFreq;
  } else if (mode == VFO_OFFSET) {
    f = vfoOffsetFreq;
  } else if (mode == BFO) {
    f = bfoFreq;
  } else if (mode == CAL) {
    f = abs(calPpm);
    neg = (calPpm < 0);
  } else if (mode == VFO_POWER) {
    f = vfoPower;
  } else if (mode == BFO_POWER) {
    f = bfoPower;
  } else if (mode == VOL) {
    f = AFGain;
  }

  // Sign
  if (neg) {
    display.drawLine(0,y+6,5,y+6,1);
  } else {
    display.drawLine(0,y+6,5,y+6,0);
  }

  // Number in kHz and Hz
  if (mode == VFO || mode == VFO_OFFSET || mode == BFO) {
    display.setCursor(startX,y);
    display.print(getMH(f)); 
  
    display.setCursor(startX + 30,y);
    sprintf(buf,"%03lu",getKH(f));
    display.print(buf);
    
    display.setCursor(startX + 70,y);
    sprintf(buf,"%03lu",getH(f));
    display.print(buf);
  } 
  // Regular number
  else {
    display.setCursor(startX,y);
    display.print(f); 
  }
  
  // Step
  if (mode == VFO || mode == VFO_OFFSET || mode == BFO || mode == CAL) {
    display.setTextSize(0);
    display.setCursor(startX,55);
    display.print(stepTitles[stepIndex]);
  }
}

void updateDisplayTransmit1() {

  display.setTextColor(WHITE);

  int y = 17;

  display.setTextSize(0);
  display.setCursor(0,y);
  display.print("VSWR");

  if (vswrState.isOutputValid()) {
    display.setTextSize(2);
    display.setCursor(20,y);
    display.print(vswrState.getVswr());
  }
  
  display.setTextSize(0);
  display.setCursor(0,y + 12);
  display.print("Power");

  if (vswrState.isOutputValid()) {
    display.setTextSize(2);
    display.setCursor(20,y + 12);
    display.print(vswrState.getPwrFwd());
  }
}

void updateDisplay() {

  // Logo information and line
  display.setCursor(0,0);
  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.println("KC1FSZ VFO4");
  display.drawLine(0,15,display.width(),15,WHITE);
  
  // Mode
  int modeX = 85;
  display.setCursor(modeX,0);
  display.print(modeTitles[(int)mode]);

  if (transmitMode) {
    updateDisplayTransmit1();
  } else {
    updateDisplayReceive1();
  }
}

// ----- AF Gain Control -------------------------------------------------------------------------

void updateAFGain() {
  // Address 0x00 on the MCP4131 is the potentiometer value. 
  spiWrite(AFGAIN_SS_PIN,SPI_MOSI_PIN,SPI_CLK_PIN,0,(AFGain * 128) / AFGainSteps);
}

// ----- Si5351 Controls -------------------------------------------------------------------------

void updateVFOFreq() {
  long f = vfoOffsetFreq - vfoFreq;
  si5351.set_freq((unsigned long long)f * 100ULL,SI5351_CLK0);
}

void updateBFOFreq() {
  long f = bfoFreq;
  si5351.set_freq((unsigned long long)f * 100ULL,SI5351_CLK2);
}

void updateVFOPower() {
  si5351.drive_strength(SI5351_CLK0,(si5351_drive)vfoPower);
}

void updateBFOPower() {
  si5351.drive_strength(SI5351_CLK2,(si5351_drive)bfoPower);
}

void updateCal() {
  si5351.set_correction(calPpm,SI5351_PLL_INPUT_XO);
}

void setup() {
  
  Serial.begin(9600);
  delay(500);

  pinMode(ENCODER0_PHASE0_PIN,INPUT_PULLUP);
  pinMode(ENCODER0_PHASE1_PIN,INPUT_PULLUP);
  pinMode(ENCODER0_PUSH_PIN,INPUT_PULLUP);
  pinMode(COMMAND_BUTTON0_PIN,INPUT_PULLUP);
  pinMode(COMMAND_BUTTON1_PIN,INPUT_PULLUP);
  pinMode(AFGAIN_SS_PIN,OUTPUT);
  digitalWrite(AFGAIN_SS_PIN,HIGH);
  pinMode(SPI_MOSI_PIN,OUTPUT);
  pinMode(SPI_CLK_PIN,OUTPUT);
  digitalWrite(SPI_CLK_PIN,LOW);
  pinMode(PTT_BUTTON_PIN,INPUT_PULLUP);
  pinMode(TR_RELAY_PIN,OUTPUT);
  digitalWrite(TR_RELAY_PIN,LOW);

  display.begin(SSD1306_SWITCHCAPVCC,SSD1306_I2C_ADDRESS);

  // Si5351 initialization
  si5351.init(SI5351_CRYSTAL_LOAD_8PF,0,0);
  // Boost up drive strength
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
 
  display.clearDisplay();

  // Pull values from EEPROM
  long magic = Utils::eepromReadLong(0);
  // Check to make sure that we have valid information in the EEPROM.  For instance,
  // if this is a new processor we might not have saved anything. 
  if (magic == MAGIC_NUMBER) {
    vfoFreq = Utils::eepromReadLong(4);
    vfoOffsetFreq = Utils::eepromReadLong(8);
    bfoFreq = Utils::eepromReadLong(12);
    calPpm = Utils::eepromReadLong(16);
    stepIndex = EEPROM.read(20);    
    vfoPower = EEPROM.read(21);
    bfoPower = EEPROM.read(22);
  } 
  
  if (stepIndex > maxStepIndex) {
    stepIndex = 0;
  }

  // Initial update of Si5351
  updateVFOFreq();
  updateBFOFreq();
  updateVFOPower();
  updateBFOPower();
  updateCal();

  // Initial update of AF Gain based on the setting of the control
  previousAFGainControlSample = analogRead(AFGAIN_PIN);
  // Scale down to the desired number of steps 
  AFGain = previousAFGainControlSample / AFGainSteps;  
  updateAFGain();

  // Initial display render
  updateDisplay();
  display.display();
}

void loop() {

  // Capture the current wall clock
  unsigned long now = millis();

  // Read all of the controls
  db2.loadSample(digitalRead(ENCODER0_PHASE0_PIN) == 0);
  db3.loadSample(digitalRead(ENCODER0_PHASE1_PIN) == 0);
  db4.loadSample(digitalRead(ENCODER0_PUSH_PIN) == 0);
  commandButton0.loadSample(digitalRead(COMMAND_BUTTON0_PIN) == 0);
  commandButton1.loadSample(digitalRead(COMMAND_BUTTON1_PIN) == 0);
  pttButton.loadSample(digitalRead(PTT_BUTTON_PIN) == 0);
  long mult = vfoEnc.getIncrement();
  long clickDuration = vfoClick.getClickDuration();
  
  // Read the volume knob, inclusive of scaling, etc.  Gain control is read from an 10 bit 
  // A/D (1024).  The digial pot has 7 bit resolution (128).
  AFGainAverager.sample(analogRead(AFGAIN_PIN));
  int AFGainControlSample = AFGainAverager.getAverage();
  
  boolean displayDirty = false;

  // Look for dial turning
  if (mult != 0) {
    // Immediately stop scanning
    scanMode = false;
    // Handle dial
    long step = mult * steps[stepIndex];
    if (mode == VFO) {
      vfoFreq += step;
      updateVFOFreq();
    } else if (mode == VFO_OFFSET) {
      vfoOffsetFreq += step;
      updateVFOFreq();
    } else if (mode == BFO) {
      bfoFreq += step;
      updateBFOFreq();
    } else if (mode == CAL) {
      calPpm += step;
      updateCal();
    } else if (mode == VFO_POWER) {
      vfoPower += 1;
      if (vfoPower > 3) {
        vfoPower = 0;
      }
      updateVFOPower();     
    } else if (mode == BFO_POWER) {
      bfoPower += 1;
      if (bfoPower > 3) {
        bfoPower = 0;
      }
      updateBFOPower();     
    }

    displayDirty = true;
  }
  // Save frequencies in EEPROM
  else if (clickDuration > 5000) {   
    Utils::eepromWriteLong(0,MAGIC_NUMBER);
    Utils::eepromWriteLong(4,vfoFreq);
    Utils::eepromWriteLong(8,vfoOffsetFreq);
    Utils::eepromWriteLong(12,bfoFreq);
    Utils::eepromWriteLong(16,calPpm);
    EEPROM.write(20,stepIndex);
    EEPROM.write(21,vfoPower);
    EEPROM.write(22,bfoPower);
  } else if (clickDuration > 0) {
    if (++stepIndex > maxStepIndex) {
      stepIndex = 0;
    } 
    displayDirty = true;
  }
  // Look for presses on the control button
  else if (commandButton1.getState() && commandButton1.isEdge()) {
    if (mode == VFO) {
      mode = VFO_OFFSET;
    } else if (mode == VFO_OFFSET) {
      mode = BFO;
    } else if (mode == BFO) {
      mode = CAL;
    } else if (mode == CAL) {
      mode = VFO_POWER;
    } else if (mode == VFO_POWER) {
      mode = BFO_POWER;
    } else if (mode == BFO_POWER) {
      mode = VOL;
    } else {
      mode = VFO;
    } 
    displayDirty = true;    
  } else if (commandButton0.getState() && commandButton0.isEdge()) {
    if (mode == VFO) {
      scanMode = !scanMode;     
      // Start the timer
      scanInterval.start(now);
    }
  } 
  // Look for changes to the volume knob that are significant.
  else if (abs(AFGainControlSample - previousAFGainControlSample) > (1024/32)) {
    if (mode != VOL) {
      savedMode = mode;
    }
    mode = VOL;
    // Latch the value for comparison with the next one
    previousAFGainControlSample = AFGainControlSample;
    // Scal down to the desired number of steps
    AFGain = AFGainControlSample / AFGainSteps;
    updateAFGain();
    AFGainChangeTimeout.start(now);
    displayDirty = true;    
  }
  // Look for PTT action
  else if (pttButton.isEdge()) {
    // Immediately stop scanning
    scanMode = false;    
    // Read the PTT button
    transmitMode = pttButton.getState();
    // Activate the relays accordingly
    digitalWrite(TR_RELAY_PIN,(transmitMode) ? HIGH : LOW); 
    // Get ready to start displaying VSWR
    vswrState.clear();
    vswrDisplayInterval.start(now);
  }
 
  // Handle scanning.  If we are in VFO mode and scanning is enabled and the scan interval
  // has expired then step the VFO frequency.
  //  
  if (mode == VFO && scanMode && scanInterval.isExpired(now)) {
    // Bump the frequency by the configured step amount
    vfoFreq += steps[stepIndex];
    // Look for wrap-around
    if (vfoFreq > maxDisplayFreq) {
      vfoFreq = minDisplayFreq;
    }
    updateVFOFreq();
    displayDirty = true;
    // Start the timer to prepare for the next step
    scanInterval.start(now);
  }

  // Handle timeout of volume control
  if (mode == VOL && AFGainChangeTimeout.isExpired(now)) {
    mode = savedMode;
    displayDirty = true;
  }

  // Sample data for VSWR meter and determine if a display update is needed
  if (transmitMode) {
    vswrState.sampleIfNecessary(now);
    if (vswrDisplayInterval.isExpired(now)) {
      vswrDisplayInterval.start(now);
      displayDirty = true;
    }
  }  

  if (displayDirty) {
    display.clearDisplay();
    updateDisplay();
    display.display();
  } 
}

