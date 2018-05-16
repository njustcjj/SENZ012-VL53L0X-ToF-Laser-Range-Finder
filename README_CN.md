# SENZ012 VL53L0X ToF 激光测距传感器

###### 翻译

> `英文` 请参考 [`这里`](https://github.com/njustcjj/SENZ011-Environment-Light-Sensor/blob/master/README.md)

> `中文` 请参考 [`这里`](https://github.com/njustcjj/SENZ011-Environment-Light-Sensor/blob/master/README_CN.md)

![](https://github.com/njustcjj/SENZ011-Environment-Light-Sensor/blob/master/pic/SENZ010.jpg "SENZ010")
 

### 产品介绍

> SENZ012 VL53L0X激光测距传感器是一款基于意法半导体（STMicroelectronics）新出的基于飞行时间测距 (ToF) 原理设计的高精度测距传感器。与传统的技术不同，VL53L0X无论目标反射率如何，都能提供精确的距离测量，最高测量距离2米。

VL53L0X集成了尖端的SPAD (Single Photon Avalanche Diodes) 阵列，并嵌入ST的第二代FlightSenseTM专利技术。精度达±3%，响应时间小于30ms，正常工作模式下功耗仅20mW，待机功耗为5uA。
VL53L0X的940nm VCSEL发射器（垂直腔面发射激光器）对人眼来说是完全不可见的，加上内部物理红外滤波器，它可以实现更远的距离，更强的抗环境光的能力，以及更好的覆盖玻璃光学截面。

> 
> 用途：拍照系统自动对焦、接近检测、手势感应、干手机等

### 产品参数

* 供电电压：+3~5V
- 激光波长：940 nm
- 量程范围：30 - 2000 mm
- 测距角度：25°
- 测距精度：±3%
- 测距时间：≤30 ms
- 工作温度：-20 ~ +70 ℃
- 产品尺寸：11 x 14 mm


### 使用教程

#### 引脚定义

|Sensor pin|Ardunio Pin|Function Description|
|-|:-:|-|
|VIN|3V~5V|Power|
|GND|GND||
|SCL|Analog pin|I2C bus interface clock|
|SDA|Analog pin|I2C bus interface data|


![](https://github.com/njustcjj/SENZ011-Environment-Light-Sensor/blob/master/pic/SENZ011_pin.jpg "引脚定义") 


#### 连线图

![](https://github.com/njustcjj/SENZ011-Environment-Light-Sensor/blob/master/pic/SENZ011_connect.png "连线图") 


### 示例代码

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


### 购买[*SENZ012 VL53L0X ToF 激光测距传感*](https://www.ebay.com/).