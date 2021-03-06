# SENZ012-VL53L0X-ToF-Laser-Range-Finder

###### Translation

> For `English`, please click [`here.`](https://github.com/njustcjj/SENZ012-VL53L0X-ToF-Laser-Range-Finder/blob/master/README.md)

> For `Chinese`, please click [`here.`](https://github.com/njustcjj/SENZ012-VL53L0X-ToF-Laser-Range-Finder/blob/master/README_CN.md)

![](https://github.com/njustcjj/SENZ012-VL53L0X-ToF-Laser-Range-Finder/blob/master/pic/SENZ012.jpg "SENZ012")


### Introduction

> The SENZ012 VL53L0X range finder is a high-precision distance finder that based on new Time-of-Flight (ToF) principle. VL53L0X provides accurate distance measurement whatever the target reflectance unlike traditional technology. It can measure absolute distances up to 2m.

The VL53L0X integrates a leading-edge SPAD array (Single Photon Avalanche Diodes) and embeds ST’s second generation FlightSenseTM patented technology. Its accuracy is ±3%, response time is less than 30ms, power consumption is only 20mW in normal operation mode, stand-by power consumption is 5uA.

The VL53L0X's 940nm VCSEL emitter (vertical cavity surface emitting laser) is totally invisible to the human eye, coupled with internal physical infrared filters, it enables longer ranging distance, higher immunity to ambient light and better robustness to cover-glass optical cross-talk.
>
> Usage : auto-focus in the photography system, gesture detection, and etc.



### Specification

- Power supply: +3~5V DC
- Infrared emitter: 940nm
- Range: 30-2000mm
- FOV: 25°
- Ranging Accuracy: ±3%
- Sampling Time: ≤ 30ms
- Operating Temperature: -20 ~ +70 °C
- Size: 11x 14 mm


### Tutorial

#### Wire Definition

|Sensor pin|Ardunio Pin|Function Description|
|-|:-:|-|
|VIN|3V~5V|Power|
|GND|GND||
|SCL|Analog pin|I2C bus interface clock|
|SDA|Analog pin|I2C bus interface data|


![](https://github.com/njustcjj/SENZ012-VL53L0X-ToF-Laser-Range-Finder/blob/master/pic/SENZ012_pin.jpg "Pin Definition") 

#### Connecting Diagram

![](https://github.com/njustcjj/SENZ012-VL53L0X-ToF-Laser-Range-Finder/blob/master/pic/SENZ012_connect.png "Connecting Diagram") 

#### Sample Code

	#include <Wire.h>

	#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
	#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
	#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
	#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
	#define VL53L0X_REG_SYSRANGE_START                  0x00
	#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
	#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
	#define address 0x29

	byte gbuf[16];

	void setup() {
	  // put your setup code here, to run once:
	  Wire.begin();        // join i2c bus (address optional for master)
	  Serial.begin(9600);  // start serial for output
	  Serial.println("VLX53LOX test started.");
	}

	void loop() {
	  Serial.println("----- START TEST ----");
	  test();
	  Serial.println("----- END TEST ----");
	  Serial.println("");
	  delay(1000);
	}

	void test() {
	  byte val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_REVISION_ID);
	  Serial.print("Revision ID: "); Serial.println(val1);

	  val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
	  Serial.print("Device ID: "); Serial.println(val1);

	  val1 = read_byte_data_at(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);
	  Serial.print("PRE_RANGE_CONFIG_VCSEL_PERIOD="); Serial.println(val1); 
	  Serial.print(" decode: "); Serial.println(VL53L0X_decode_vcsel_period(val1));

	  val1 = read_byte_data_at(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);
	  Serial.print("FINAL_RANGE_CONFIG_VCSEL_PERIOD="); Serial.println(val1);
	  Serial.print(" decode: "); Serial.println(VL53L0X_decode_vcsel_period(val1));

	  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

	  byte val = 0;
	  int cnt = 0;
	  while (cnt < 100) { // 1 second waiting time max
	    delay(10);
	    val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
	    if (val & 0x01) break;
	    cnt++;
	  }
	  if (val & 0x01) Serial.println("ready"); else Serial.println("not ready");

	  read_block_data_at(0x14, 12);
	  uint16_t acnt = makeuint16(gbuf[7], gbuf[6]);
	  uint16_t scnt = makeuint16(gbuf[9], gbuf[8]);
	  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
	  byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);

	  Serial.print("ambient count: "); Serial.println(acnt);
	  Serial.print("signal count: ");  Serial.println(scnt);
	  Serial.print("distance ");       Serial.println(dist);
	  Serial.print("status: ");        Serial.println(DeviceRangeStatusInternal);
	}

	uint16_t bswap(byte b[]) {
	  // Big Endian unsigned short to little endian unsigned short
	  uint16_t val = ((b[0] << 8) & b[1]);
	  return val;
	}

	uint16_t makeuint16(int lsb, int msb) {
	    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
	}

	void write_byte_data(byte data) {
	  Wire.beginTransmission(address);
	  Wire.write(data);
	  Wire.endTransmission();
	}

	void write_byte_data_at(byte reg, byte data) {
	  // write data word at address and register
	  Wire.beginTransmission(address);
	  Wire.write(reg);
	  Wire.write(data);
	  Wire.endTransmission();
	}

	void write_word_data_at(byte reg, uint16_t data) {
	  // write data word at address and register
	  byte b0 = (data &0xFF);
	  byte b1 = ((data >> 8) && 0xFF);
    
	  Wire.beginTransmission(address);
	  Wire.write(reg);
	  Wire.write(b0);
	  Wire.write(b1);
	  Wire.endTransmission();
	}

	byte read_byte_data() {
	  Wire.requestFrom(address, 1);
	  while (Wire.available() < 1) delay(1);
	  byte b = Wire.read();
	  return b;
	}

	byte read_byte_data_at(byte reg) {
	  //write_byte_data((byte)0x00);
	  write_byte_data(reg);
	  Wire.requestFrom(address, 1);
	  while (Wire.available() < 1) delay(1);
	  byte b = Wire.read();
	  return b;
	}

	uint16_t read_word_data_at(byte reg) {
	  write_byte_data(reg);
	  Wire.requestFrom(address, 2);
	  while (Wire.available() < 2) delay(1);
	  gbuf[0] = Wire.read();
	  gbuf[1] = Wire.read();
	  return bswap(gbuf); 
	}

	void read_block_data_at(byte reg, int sz) {
	  int i = 0;
	  write_byte_data(reg);
	  Wire.requestFrom(address, sz);
	  for (i=0; i<sz; i++) {
	    while (Wire.available() < 1) delay(1);
	    gbuf[i] = Wire.read();
	  }
	}


	uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
	  // Converts the encoded VCSEL period register value into the real
	  // period in PLL clocks
	  uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
	  return vcsel_period_pclks;
	}



### Purchasing [*SENZ012 VL53L0X ToF Laser Range Finder*](https://www.ebay.com/).
