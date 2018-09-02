// Area51 FPV Experiment by redfive410
// Based on Delta 5 Race Timer by Scott Chin
// SPI driver based on fs_skyrf_58g-main.c Written by Simon Chambers
//
// MIT License
//
// Copyright (c) 2018
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

const int slaveSelectPin = 10; // Setup data pins for rx5808 comms
const int spiDataPin = 11;
const int spiClockPin = 13;

struct {
	uint16_t volatile vtxFreq = 5740;
	uint8_t volatile filterRatio = 10;
	float volatile filterRatioFloat = 0.0f;
} settings;

struct {
	// Current unsmoothed rssi
	uint16_t volatile rssiRaw = 0;
	// Smoothed rssi value, needs to be a float for smoothing to work
	float volatile rssiSmoothed = 0;
	// int representation of the smoothed rssi value
	uint16_t volatile rssi = 0;
 // int array of last 10 rssi values
	uint16_t volatile rssiHistory[10] = {0,0,0,0,0,0,0,0,0,0};
  uint16_t volatile rssiHistoryIndex = 0;

  // True when gate is calibrated and after quad passes through the gate
  bool volatile steady_state = false;
  // True when the quad is going through the gate
  bool volatile crossing_state = false;
} state;

// Defines for fast ADC reads
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Initialize program
void setup() {
	Serial.begin(115200); // Start serial for output/debugging

	pinMode (slaveSelectPin, OUTPUT); // RX5808 comms
	pinMode (spiDataPin, OUTPUT);
	pinMode (spiClockPin, OUTPUT);
	digitalWrite(slaveSelectPin, HIGH);

	while (!Serial) {
	}; // Wait for the Serial port to initialise

	// set ADC prescaler to 16 to speedup ADC readings
    sbi(ADCSRA,ADPS2);
    cbi(ADCSRA,ADPS1);
    cbi(ADCSRA,ADPS0);

	// Initialize lastPass defaults
	settings.filterRatioFloat = settings.filterRatio / 1000.0f;

	setRxModule(settings.vtxFreq); // Setup rx module to default frequency
}

// Functions for the rx5808 module
void SERIAL_SENDBIT1() {
	digitalWrite(spiClockPin, LOW);
	delayMicroseconds(300);
	digitalWrite(spiDataPin, HIGH);
	delayMicroseconds(300);
	digitalWrite(spiClockPin, HIGH);
	delayMicroseconds(300);
	digitalWrite(spiClockPin, LOW);
	delayMicroseconds(300);
}
void SERIAL_SENDBIT0() {
	digitalWrite(spiClockPin, LOW);
	delayMicroseconds(300);
	digitalWrite(spiDataPin, LOW);
	delayMicroseconds(300);
	digitalWrite(spiClockPin, HIGH);
	delayMicroseconds(300);
	digitalWrite(spiClockPin, LOW);
	delayMicroseconds(300);
}
void SERIAL_ENABLE_LOW() {
	delayMicroseconds(100);
	digitalWrite(slaveSelectPin,LOW);
	delayMicroseconds(100);
}
void SERIAL_ENABLE_HIGH() {
	delayMicroseconds(100);
	digitalWrite(slaveSelectPin,HIGH);
	delayMicroseconds(100);
}

// Calculate rx5808 register hex value for given frequency in MHz
uint16_t freqMhzToRegVal(uint16_t freqInMhz) {
  uint16_t tf, N, A;
  tf = (freqInMhz - 479) / 2;
  N = tf / 32;
  A = tf % 32;
  return (N<<7) + A;
}

// Set the frequency given on the rx5808 module
void setRxModule(int frequency) {
	uint8_t i; // Used in the for loops

	// Get the hex value to send to the rx module
	uint16_t vtxHex = freqMhzToRegVal(frequency);

	// bit bash out 25 bits of data / Order: A0-3, !R/W, D0-D19 / A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
	SERIAL_ENABLE_HIGH();
	delay(2);
	SERIAL_ENABLE_LOW();
	SERIAL_SENDBIT0();
	SERIAL_SENDBIT0();
	SERIAL_SENDBIT0();
	SERIAL_SENDBIT1();
	SERIAL_SENDBIT0();

	for (i = 20; i > 0; i--) SERIAL_SENDBIT0(); // Remaining zeros

	SERIAL_ENABLE_HIGH(); // Clock the data in
	delay(2);
	SERIAL_ENABLE_LOW();

	// Second is the channel data from the lookup table, 20 bytes of register data are sent, but the
	// MSB 4 bits are zeros register address = 0x1, write, data0-15=vtxHex data15-19=0x0
	SERIAL_ENABLE_HIGH();
	SERIAL_ENABLE_LOW();

	SERIAL_SENDBIT1(); // Register 0x1
	SERIAL_SENDBIT0();
	SERIAL_SENDBIT0();
	SERIAL_SENDBIT0();

	SERIAL_SENDBIT1(); // Write to register

	// D0-D15, note: loop runs backwards as more efficent on AVR
	for (i = 16; i > 0; i--) {
		if (vtxHex & 0x1) { // Is bit high or low?
			SERIAL_SENDBIT1();
		}
		else {
			SERIAL_SENDBIT0();
		}
		vtxHex >>= 1; // Shift bits along to check the next one
	}

	for (i = 4; i > 0; i--) // Remaining D16-D19
		SERIAL_SENDBIT0();

	SERIAL_ENABLE_HIGH(); // Finished clocking data in
	delay(2);

	digitalWrite(slaveSelectPin,LOW);
	digitalWrite(spiClockPin, LOW);
	digitalWrite(spiDataPin, LOW);
}


// Read the RSSI value for the current channel
int rssiRead() {
	return analogRead(0);
}

// Main loop
void loop() {
  delay(150);
  state.rssiRaw = rssiRead();
	state.rssiSmoothed = (settings.filterRatioFloat * (float)state.rssiRaw) + ((1.0f-settings.filterRatioFloat) * state.rssiSmoothed);
	state.rssi = (int)state.rssiSmoothed;
  
  state.rssiHistory[0] = state.rssiHistory[1];
  state.rssiHistory[1] = state.rssiHistory[2];
  state.rssiHistory[2] = state.rssiHistory[3];
  state.rssiHistory[3] = state.rssiHistory[4];
  state.rssiHistory[4] = state.rssiHistory[5];
  state.rssiHistory[5] = state.rssiHistory[6];
  state.rssiHistory[6] = state.rssiHistory[7];
  state.rssiHistory[7] = state.rssiHistory[8];
  state.rssiHistory[8] = state.rssiHistory[9];
  state.rssiHistory[9] = state.rssi;

  uint32_t rssiHistorySum = 0;
  for (int i=0; i < 10; i++) {
    rssiHistorySum += state.rssiHistory[i];
  }

  if (state.rssi * 10 < rssiHistorySum + 3) {
    Serial.println("Steady State");
    state.steady_state = true;
    if (state.crossing_state == true){
      state.crossing_state = !state.crossing_state;  
    }
  }

  if (state.steady_state == true && state.rssiHistory[0] + 3 < state.rssiHistory[9]) {
    Serial.println("Crossing Pass Start");
    state.steady_state = false;
    state.crossing_state = true;
  }
}
