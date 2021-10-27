#include <Arduino.h>
#include <SoftWire.h>
#include <AsyncDelay.h>

/* I2C Addresses */
#define I2C_ADDRESS_ADU 0x70
#define I2C_ADDRESS_MS4525 0x28

/* Register address */
#define ADDR_READ_MR 0x00 // write to this address to start conversion 

/* Arduino Pins */
#define SDA_MS4525 20
#define SCL_MS4525 21

/* SoftWire and AsyncDelay Objects */
SoftWire MS4525(SDA_MS4525, SCL_MS4525);
AsyncDelay samplingInterval;

/* Function declarations */
int readMS4525(unsigned int *p_P_dat, unsigned int *p_T_dat);

void setup() {

	Serial.begin(9600);
	delay(500);
	Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>");

	// Ensure we see the device
	uint8_t startResult = MS4525.llStart(I2C_ADDRESS_MS4525); // Signal a read
	if (startResult < 1) {
		Serial.println("Error: Failed to signal a read from device.");
	}

	
	
	
}

void loop() {

	byte MS4525_status;
	unsigned int P_dat, T_dat;
	float PR, TR, V, VV;

	MS4525_status = readMS4525(&P_dat, &T_dat);

	switch (MS4525_status) {
		case 0: Serial.println("Ok");
		break;
	case 1: Serial.println("Busy");
		break;
	case 2: Serial.println("Stale Data");
		break;
	default: Serial.println("Error");
		break;
	}
	
	PR = (double)((P_dat-819.15)/(14744.7)) ;
	PR = (PR - 0.49060678) ;
	PR = abs(PR);
	V = ((PR*13789.5144)/1.225);
	VV = (sqrt((V)));

	TR = (double)((T_dat*0.09770395701));
	TR = TR-50;

	Serial.print("raw Pressure:");
	Serial.println(P_dat);
	Serial.print("pressure psi:");
	Serial.println(PR,10);
	Serial.print("raw Temp:");
	Serial.println(T_dat);
	Serial.print("temp:");
	Serial.println(TR);
	Serial.print("speed m/s :");
	Serial.println(VV,5);
  
    delay(1000);
}

/**
 * @brief TODO
 * 
 * @return int 
 */
int readMS4525(unsigned int *p_P_dat, unsigned int *p_T_dat) {
	
	// intialize variables
	uint8_t PH, PL, TH, TL, status;
	unsigned int P_dat, T_dat;
	uint8_t rxBuffer[4] = {0, 0, 0, 0};
	
	// set the rx buffer and size
	MS4525.setRxBuffer(rxBuffer, 4);
	
	// write the start bit to access the first register
	MS4525.beginTransmission(I2C_ADDRESS_MS4525);
	MS4525.write((uint8_t)ADDR_READ_MR);
	MS4525.endTransmission();
	delay(100);

	// fill the 4-byte buffer with data
	int numBytes = MS4525.requestFrom(I2C_ADDRESS_MS4525, 4);
	for (int i = 0; i < numBytes; ++i) {
		rxBuffer[i] = MS4525.read();
		//Serial.println(rxBuffer[i]);
	}
	if (numBytes != 4) {
		Serial.print("Read wrong number of bytes: ");
		Serial.println((int)numBytes);
		return -1;
	}
	MS4525.endTransmission(); // just to be sure?

	// interpret the data
	// see https://www.amsys-sensor.com/downloads/notes/I2C-Interface-to-Digital-Pressure-Sensors-AMSYS-an802e.pdf
	// and https://forum.arduino.cc/t/ms-4525do/298848/3
	PH = rxBuffer[0];
	PL = rxBuffer[1];
	TH = rxBuffer[2];
	TL = rxBuffer[3];

	// status bits (2 MSB of Output Data Packet)
	status = (PH >> 6) & 0x03;

    PH = PH & 0x3f;
    P_dat = (((unsigned int)PH) << 8) | PL;
	*p_P_dat = P_dat;


	TL = (TL >> 5);
	T_dat = (((unsigned int)TH) << 3) | TL;
	*p_T_dat = T_dat;
	
	return status;
	
}