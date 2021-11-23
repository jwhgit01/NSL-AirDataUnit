/**
 * @file main.cpp
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 * @brief VT Nonlinear Systems Lab air data unit mircrocontroller code
 * @link TODO: Github
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Arduino.h>
#include <i2c_t3.h>

/* Debugging Modes */
#define DEBUG 		false
#define DEBUG_TIME	true
#define DEBUG_BYTES false

/* Calibration Mode */
#define CALIBRATION	false

/* I2C Addresses */
#define I2C_ADDRESS_ADU 0x4F
#define I2C_ADDRESS_MS4525 0x28
#define READ_CMD 0x00

/* Number of bytes sent and received */
#define NUM_BYTES_OUT 6
#define NUM_BYTES_MS4525 4

/* Teensy LC Pins */
#define ALPHA_PIN 3
#define BETA_PIN  4
#define MS4525_PINS I2C_PINS_22_23
#define ADU_PINS I2C_PINS_18_19

/**
 * @section Declarations
 */
//
// Calibration for ADU-001 on 11/22/21
//	deg = m*PWM + b
//
float b_alpha = -244.20;
float b_beta  = -143.19;
float m_alpha = 0.3704;
float m_beta  = 0.3535;
//
// Conversion factors and max values
//
const int dP_scale = 53; // dp_int = floor(dp_PA*dP_scale)
const int ab_scale = 180; // alpha_int = alpha_deg*ab_scale
//
// Interrupt functions and variables
//
void alpha_rising();
void beta_rising();
void alpha_falling();
void beta_falling();
volatile int alpha_PWM = 0;
volatile int beta_PWM = 0;
volatile int alpha_prev_time = 0;
volatile int beta_prev_time = 0;
//
// I2C slave
//
volatile uint8_t ready;
uint8_t TxBuffer[NUM_BYTES_OUT] = {0};
//
// I2C master
//
uint8_t RxBuffer[NUM_BYTES_MS4525] = {0};
//
// Sensor measuements
//
float diff_press_Pa = 0.0f;
float temp_C = 0.0f;
float P_offset = 0.0f;
uint16_t dP_int, alpha_int, beta_int;
int ret = 0;
bool collect_phase = false;
//
// Timing vars
//
volatile long t0 = 0;
volatile long t1 = 0;

/**
 * @section Airspeed sensor function declarations
 */
int measure(void);
int collect(void);
int readMS4525(void);

/**
 * @brief I2C slave requests
 */
void requestEvent( void );

/**
 * @brief Arduino setup function. Runs once on startup.
 */
void setup() {
	//
	// LED for debugging
	//
	pinMode(LED_BUILTIN,OUTPUT); // LED
	//
	// Setup for ADU in slave mode, address 0x4F, pins 18/19, external pullups, 400kHz
	//
	ready = 0;
	memset(TxBuffer, 0, NUM_BYTES_OUT);
    Wire.begin(I2C_SLAVE, I2C_ADDRESS_ADU, ADU_PINS, I2C_PULLUP_EXT, 400000);
    //
    // Set register request event
	//
    Wire.onRequest(requestEvent);
	//
	// Setup MS4525 in master mode, address 0x44, pins 22/23, external pullups, 400kHz
	//
	memset(RxBuffer, 0, NUM_BYTES_MS4525);
    Wire1.begin(I2C_MASTER, I2C_ADDRESS_MS4525, MS4525_PINS, I2C_PULLUP_EXT, 400000);
	delay(100);
	//
	// Begin the serial monitor for error messages and debugging
	//
	Serial.begin(115200);
	delay(500);
	Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>");
	//
	// Calibrate the pitot tube
	//
	float calib_data = 0; 
	int n = 100;
	for (int i=0; i<n; i++) {
		int status = readMS4525();
		if (status < 1) {
			n--;
			delay(1);
			continue;
		}
		calib_data = calib_data + diff_press_Pa;
	}
	P_offset = calib_data/n;
	Serial.printf("diff Pressure calibrated from %d samples.\n",n);
	Serial.printf("dP_offset = %0.4f\n", P_offset);
	//
	// Set the PWM interrupt pins
	//
	pinMode(ALPHA_PIN, INPUT); // this may not be necessary
	pinMode(BETA_PIN, INPUT);  // this may not be necessary
  	digitalWrite(ALPHA_PIN, LOW); // this may not be necessary
	digitalWrite(BETA_PIN, LOW); // this may not be necessary
	attachInterrupt(ALPHA_PIN, alpha_rising, RISING); // when pin goes high, call the rising function
	attachInterrupt(BETA_PIN, beta_rising, RISING);  // when pin goes high, call the rising function
	//
	// If calibration mode, get samples of vane data and spit it out
	//
	#if CALIBRATION
		delay(1000);
		n = 100;
		long alphaPWM_sum = 0;
		long betaPWM_sum = 0;
		for (int i=0; i<n; i++) {
			delay(1);
			alphaPWM_sum = alphaPWM_sum + alpha_PWM;
			betaPWM_sum = betaPWM_sum + beta_PWM;
		}
		int alphaPWM_avg = alphaPWM_sum/n;
		int betaPWM_avg = betaPWM_sum/n;
		Serial.println("###############");
		Serial.print("alpha = ");
		Serial.println(alphaPWM_avg);
		Serial.print("beta = ");
		Serial.println(betaPWM_avg);
		Serial.println("###############");
		delay(10000);
	#endif
}

/**
 * @brief Main Arduino loop
 */
void loop() {
	//
	// LED off
	//
	digitalWrite(LED_BUILTIN,LOW);
	//
	// The alpha and beta PWM readings are interrupt-driven
	// and will be updated when there is data
	//
	// Read data from the differential pressure sensor
	//
	int status = readMS4525();
	if (status < 1) {
		#if DEBUG
			Serial.print("!");
		#endif
		return;
	}
	#if DEBUG
		Serial.print("dP = "); // something buggy with prinf and floats
		Serial.print(diff_press_Pa);
		Serial.print(", T_C = ");
		Serial.print(temp_C);
		Serial.printf("alpha_PWM = %d, beta_PWM = %d \n", alpha_PWM, beta_PWM);
	#endif
	//
	// Convert vane angles to degrees
	//
	float alpha_deg = m_alpha*alpha_PWM + b_alpha;
	float beta_deg = m_beta*beta_PWM + b_beta;
	//
	// Convert data to 16-bit integers
	//
	dP_int = static_cast<uint16_t>(diff_press_Pa*dP_scale);
	alpha_int = static_cast<uint16_t>(alpha_deg*ab_scale);
	beta_int = static_cast<uint16_t>(beta_deg*ab_scale);
	//
	// Put data into a big-endian array of bytes
	//
	TxBuffer[0] = (dP_int >> 8) & 0xFF;
	TxBuffer[1] = dP_int & 0xFF;
	TxBuffer[2] = (alpha_int >> 8) & 0xFF;
	TxBuffer[3] = alpha_int & 0xFF;
	TxBuffer[4] = (beta_int >> 8)  & 0xFF;
	TxBuffer[5] = beta_int & 0xFF;
	//
	// If we've gotten here, there is an assembled packet and we're ready
	// to send data as long as it has been requested.
	//
	ready = 1;
	//
	// debug timing
	//
	#if DEBUG_TIME
		t1 = micros();
		Serial.print("dt = ");
		Serial.println(t1-t0);
		t0 = t1;
	#endif
	//
	// pause for timing and stability
	// We're aiming for just faster than 100Hz
	//
    delayMicroseconds(9000);
	return;
}

/**
 * @brief Measurement phase of the airspeed sensor
 * 
 * @return number of available bytes
 */
int measure(void) {
	//
	// write the start bit to access the first register
	//
	Wire1.beginTransmission(I2C_ADDRESS_MS4525);
	Wire1.write(READ_CMD); // read command
	Wire1.endTransmission(I2C_NOSTOP); // non-blocking
	int numBytesAvail = Wire1.requestFrom(I2C_ADDRESS_MS4525, NUM_BYTES_MS4525);
	//
	// Check if error occured
	//
	if (Wire1.getError()) {
		Serial.println("Failed to request bytes from MS4525");
		return -1;
	}
	return numBytesAvail;
}

/**
 * @brief Collection phase of the airspeed sensor
 * 
 * @return number of bytes read
 * 
 * @link https://www.te.com/usa-en/product-CAT-BLPS0002.html
 * @link https://www.amsys-sensor.com/downloads/notes/I2C-Interface-to-Digital-Pressure-Sensors-AMSYS-an802e.pdf
 * @link https://forum.arduino.cc/t/ms-4525do/298848/3
 * @link https://github.com/PX4/PX4-Autopilot/blob/master/src/drivers/differential_pressure/ms4525/ms4525_airspeed.cpp
 */
int collect(void) {
	//
	// Read the requested bytes
	//
	int numBytesRead = Wire1.read(RxBuffer, NUM_BYTES_MS4525);
	//
	// Check for errors
	//
	if (numBytesRead != NUM_BYTES_MS4525) {
		return numBytesRead;
	}
	if (Wire1.getError()) {
		Serial.println("Failed to read bytes from MS4525");
		return -1;
	}
	//
	// Check the status bits (2 MSB of Output Data Packet)
	//
	uint8_t status = (RxBuffer[0] & 0xC0) >> 6;
	#if DEBUG_BYTES
		Serial.printf("Status = %d\n", status);
	#endif
	switch (status) {
	case 0: // Normal Operation. Good Data Packet
		break;
	case 1: // Reserved
		return 0;
	case 2: // Stale Data. Data has been fetched since last measurement cycle.
		return 0;
	case 3: // Fault Detected
		return -1;
	}
	#if DEBUG_BYTES
		//
		// print the raw buffer
		//
		Serial.print("MS4525 buffer: | ");
		for (int i=0; i<NUM_BYTES_MS4525; i++) {
			Serial.print(RxBuffer[i]);
			Serial.print(" | ");
		}
		Serial.print("\n");
	#endif
	//
	// Do some bit math to get data from the buffer
	//
	int16_t dp_raw = 0, dT_raw = 0;
	dp_raw = (RxBuffer[0] << 8) | RxBuffer[1];
	dp_raw = 0x3FFF & dp_raw; // mask the used bits
	dT_raw = (RxBuffer[2] << 8) | RxBuffer[3];
	dT_raw = (0xFFE0 & dT_raw) >> 5;
	#if DEBUG_BYTES
		Serial.printf("dp_raw = %d, dT_raw = %d \n", dp_raw, dT_raw);
	#endif
	//
	// A max dT is almost certainly an invalid reading
	//
	if (dT_raw == 2047) {
		return 0;
	}
	//
	// Temperature in Celcius
	//
	temp_C = ((200.0f * dT_raw) / 2047) - 50;
	//
	// Calculate differential pressure. It's centered about 8000
	// and can be positive or negative
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
	diff_press_Pa = (diff_press_PSI * PSI_to_Pa) - P_offset;
	//
	// We got a good packet of data!
	//
	return numBytesRead;
}

/**
 * @brief Read from the airspeed sensor
 * 
 * @return status:
 * 			0 - Try again
 * 			1 - Good data
 */
int readMS4525(void) {
	//
	// set the buffer to zero
	//
	memset(RxBuffer, 0, NUM_BYTES_MS4525);
	//
	// collection phase
	//
	if (collect_phase) {
		//
		// perform collection
		//
		ret = collect();
		delayMicroseconds(100);
		if (ret < NUM_BYTES_MS4525) {
			//
			// restart the measurement state machine
			//
			collect_phase = false;
			return 0;
		}
		//
		// next phase is measurement
		//
		collect_phase = false;
	}
	//
	// measurement phase
	//
	ret = measure();
	delayMicroseconds(100);
	if (ret < NUM_BYTES_MS4525) {
		Serial.print("Measure error");
		return 0;
	}
	// 
	// next phase is collection
	//
	collect_phase = true;
	return 1;
}

/**
 * @brief handle Tx Event (outgoing I2C data)
 */
void requestEvent(void) {
	//
	// if the command was to send data, send it out!
	//
	if (ready) {
		Wire.write(TxBuffer,NUM_BYTES_OUT);
		#if DEBUG
			Serial.print("bytes = | ");
			for (int i=0; i<5; i++) { 
				Serial.printf("%x", TxBuffer[i]);
				Serial.print(" | ");
			}
			Serial.print("\n");
		#endif
		//
		// LED on
		//
		digitalWrite(LED_BUILTIN,HIGH);
		//
		// Get Wire Error - returns "Wire" error code from a failed Tx/Rx command
		// 0=success, 1=data too long, 2=recv addr NACK, 3=recv data NACK, 4=other error (timeout, arb lost)
		//
		uint8_t TxError = Wire.getError();
		if (TxError > 0){
			Serial.print(TxError);
		}
		ready = 0;
		#if DEBUG_TIME
			Serial.println(".");
		#endif
	}
}

/**
 * @section Interrupt functions for alpha and beta vanes
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