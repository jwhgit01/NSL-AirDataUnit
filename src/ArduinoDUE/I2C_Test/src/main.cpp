#include <Arduino.h>
#include <SoftWire.h>
#include <AsyncDelay.h>

/* Debugging Mode */
#define DEBUG true

/* I2C Addresses */
#define I2C_ADDRESS_ADU 0x70
#define I2C_ADDRESS_MS4525 0x28

/* Register address */
#define ADDR_READ_MR 0x00 // write to this address to start conversion 

/* Arduino Pins */
#define ALPHA_PIN 2
#define BETA_PIN  3
#define SDA_MS4525 20
#define SCL_MS4525 21

/* SoftWire and AsyncDelay Objects */
SoftWire MS4525(SDA_MS4525, SCL_MS4525);
AsyncDelay samplingInterval;

/**
 * @brief Read and interpret data from the airspeed (differential pressure) sensor
 * 
 * @return int Status:
 * 	-1 Critical error,
 * 	 0 Try again,
 *   1 Good data			
 */
int readMS4525( float * diff_press_Pa, float * temp_C );

/**
 * @section interrupt functions
 */
void alpha_rising();
void beta_rising();
void alpha_falling();
void beta_falling();

/* Global variables */
float diff_press_Pa, temp_C;
float P_offset = 0.0f;
volatile int alpha_PWM = 0;
volatile int beta_PWM = 0;
volatile int alpha_prev_time = 0;
volatile int beta_prev_time = 0;

/* sign function */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * @brief Arduino setup function. Runs once on startup.
 */
void setup() {
	//
	// Begin the serial monitor for error messages and debugging
	//
	Serial.begin(115200);
	delay(500);
	Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>");
	//
	// Ensure we see the device
	//
	uint8_t startResult = MS4525.llStart(I2C_ADDRESS_MS4525);
	if (startResult < 1) {
		Serial.println("Error: Failed to signal a read from device.");
	}
	//
	// Calibrate the airspeed
	//
	int MS4525_status = readMS4525(&diff_press_Pa, &temp_C);
	P_offset = diff_press_Pa;
	//
	// Set the PWM input pins and make sure we are reading data
	//
	attachInterrupt(ALPHA_PIN, alpha_rising, RISING); // when pin goes high, call the rising function
	attachInterrupt(BETA_PIN, beta_rising, RISING);  // when pin goes high, call the rising function
	
}

/**
 * @brief Main Arduino loop
 */
void loop() {
	//
	// Read the differential pressure sensor
	//
	int MS4525_status = readMS4525(&diff_press_Pa, &temp_C);
	//
	// Error handling with MS4525_status
	//
	if (MS4525_status < 1) {
		delay(100);
	}
	//
	// The alpha and beta PWM readings are interrupt-driven
	// and will be updated when there is data
	//
	// print out the data
	//
	#if DEBUG
		char data_print[128];
		sprintf(data_print,
				"P_Pa = %+0.4f, T_C = %+0.4f, alpha(PWM) = %d, beta(PWM) = %d",
				diff_press_Pa, temp_C, alpha_PWM, beta_PWM);
		Serial.println(data_print);
	#endif
	//
	// pause for timing and stability
	//
    delay(10);
}

/**
 * @brief Read and interpret data from the airspeed (differential pressure) sensor
 */
int readMS4525( float * diff_press_Pa, float * temp_C ) {
	
	/**
	 * @section Read the raw data from the I2C MS4525DO sensor
	 * 
	 * @link https://www.te.com/usa-en/product-CAT-BLPS0002.html
	 */
	//
	// set the rx buffer and size
	//
	uint8_t RxBuffer[4] = {0, 0, 0, 0};
	MS4525.setRxBuffer(RxBuffer, 4);
	//
	// write the start bit to access the first register
	//
	MS4525.beginTransmission(I2C_ADDRESS_MS4525);
	MS4525.write((uint8_t)ADDR_READ_MR);
	MS4525.endTransmission();
	delay(10);
	//
	// fill the 4-byte buffer with data
	//
	int numBytes = MS4525.requestFrom(I2C_ADDRESS_MS4525, 4);
	for (int i = 0; i < numBytes; ++i) {
		RxBuffer[i] = MS4525.read();
		#if DEBUG
			// Serial.println(RxBuffer[i]);
		#endif
	}
	if (numBytes != 4) {
		Serial.print("Read wrong number of bytes: ");
		Serial.println((int)numBytes);
		delay(100);
		return 0;
	}
	MS4525.endTransmission(); // is this necessary?
	
	/**
	 * @section Interpret the raw data
	 * 
	 * @link https://www.amsys-sensor.com/downloads/notes/I2C-Interface-to-Digital-Pressure-Sensors-AMSYS-an802e.pdf
	 * @link https://forum.arduino.cc/t/ms-4525do/298848/3
	 * @link https://github.com/PX4/PX4-Autopilot/blob/master/src/drivers/differential_pressure/ms4525/ms4525_airspeed.cpp
	 */
	//
	// Check the status bits (2 MSB of Output Data Packet)
	//
	uint8_t status = (RxBuffer[0] & 0xC0) >> 6;
	switch (status) {
	case 0:
		// Normal Operation. Good Data Packet
		break;
	case 1:
		// Reserved
		return 0;
	case 2:
		// Stale Data. Data has been fetched since last measurement cycle.
		return 0;
	case 3:
		// Fault Detected
		return 0;
	}
	//
	// Do some bit math to get data from the buffer
	//
	int16_t dp_raw = 0, dT_raw = 0;
	dp_raw = (RxBuffer[0] << 8) + RxBuffer[1];
	dp_raw = 0x3FFF & dp_raw; // mask the used bits
	dT_raw = (RxBuffer[2] << 8) + RxBuffer[3];
	dT_raw = (0xFFE0 & dT_raw) >> 5;
	//
	// A max dT is almost certainly an invalid reading
	//
	if (dT_raw == 2047) {
		return 0;
	}
	//
	// Temperature in Celcius
	//
	*temp_C = ((200.0f * dT_raw) / 2047) - 50;
	//
	// Calculate differential pressure. As its centered around 8000
	// and can go positive or negative
	//
	const float P_min = -1.0f;
	const float P_max = 1.0f;
	const float PSI_to_Pa = 6894.757f;
	/*
	  This equation is an inversion of the equation in the
	  pressure transfer function figure on page 4 of the datasheet.
	  We negate the result so that positive differential pressures
	  are generated when the bottom port is used as the static
	  port on the pitot and top port is used as the dynamic port
	 */
	float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
	*diff_press_Pa = (diff_press_PSI * PSI_to_Pa) - P_offset;
	//
	// We got a good packet of data!
	//
	return 1;
}

/**
 * @section Interrupt functions fopr alpha and beta vanes
 * @note Most arduinos have external interrupts on pins 2 and 3 (numbers 0 and 1)
 * @link https://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
 */
void alpha_rising() {
	attachInterrupt(ALPHA_PIN, alpha_falling, FALLING);
	alpha_prev_time = micros();
}
void beta_rising() {
	attachInterrupt(BETA_PIN, beta_falling, FALLING);
	beta_prev_time = micros();
}
void alpha_falling() {
	attachInterrupt(ALPHA_PIN, alpha_rising, RISING);
	alpha_PWM = micros()-alpha_prev_time;
}
void beta_falling() {
	attachInterrupt(BETA_PIN, beta_rising, RISING);
	beta_PWM = micros()-beta_prev_time;
}