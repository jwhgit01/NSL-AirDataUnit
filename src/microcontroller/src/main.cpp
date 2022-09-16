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
#define READ_CMD 0x00

/* Number of bytes sent */
#define NUM_BYTES_OUT 4

/* Teensy LC Pins */
#define ALPHA_PIN 3
#define BETA_PIN  4
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
// Timing vars
//
volatile long t0 = 0;
volatile long t1 = 0;


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
	memset(TxBuffer, 0, NUM_BYTES_OUT);
    Wire.begin(I2C_SLAVE, I2C_ADDRESS_ADU, ADU_PINS, I2C_PULLUP_EXT, 400000);
    //
    // Set register request event
	//
    Wire.onRequest(requestEvent);
	//
	// Begin the serial monitor for error messages and debugging
	//
	Serial.begin(115200);
	delay(500);
	Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>");
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
	// Convert vane angles to degrees
	//
	float alpha_deg = m_alpha*alpha_PWM + b_alpha + 180;
	float beta_deg = m_beta*beta_PWM + b_beta + 180;
	#if DEBUG
		Serial.print("alpha_deg = ");
		Serial.print(alpha_deg);
		Serial.print(", beta_deg = ");
		Serial.println(beta_deg);
	#endif
	//
	// Convert data to 16-bit integers
	//
	uint16_t alpha_int = static_cast<uint16_t>(alpha_deg*ab_scale);
	uint16_t beta_int = static_cast<uint16_t>(beta_deg*ab_scale);
	//
	// Put data into a big-endian array of bytes
	//
	TxBuffer[0] = (alpha_int >> 8) & 0xFF;
	TxBuffer[1] = alpha_int & 0xFF;
	TxBuffer[2] = (beta_int >> 8)  & 0xFF;
	TxBuffer[3] = beta_int & 0xFF;
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
	//
    delayMicroseconds(4900);
	return;
}

/**
 * @brief handle Tx Event (outgoing I2C data)
 */
void requestEvent(void) {

		Wire.write(TxBuffer,NUM_BYTES_OUT);
		#if DEBUG
			Serial.print("bytes = | ");
			for (int i=0; i<(NUM_BYTES_OUT-1); i++) { 
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