#define F_CPU 16000000UL
#define MPU_ADDR 0x68
#define MAG_ADDR 0x1E
#define RAD_TO_DEG 180.0 / M_PI

#include <stdint.h>
extern "C" {
	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <util/delay.h>
	#include <stdlib.h>
	#include <stdio.h>
	#include <math.h>
};

volatile uint32_t milliseconds = 0;
ISR(TIMER0_COMPA_vect) {
	milliseconds++;
}

uint32_t millis() {
	cli();
	uint32_t ms = milliseconds;
	sei();
	return ms;
}

void timer0_init() {
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);
	OCR0A = 249;
	TIMSK0 |= (1 << OCIE0A);
	sei();
}

void ltrim(char *str) {
	if (!str) return;
	
	int8_t i = 0;
	while (str[i] == ' ' || str[i] == '\t') i++;
	
	if (i > 0) {
		int j = 0;
		while (str[i]) str[j++] = str[i++];
		str[j] = '\0';
	}
}

class Serial {
	public:
	void init(uint16_t baud) {
		uint16_t ubrr = F_CPU / 16 / baud - 1;
		UBRR0H = (uint8_t)(ubrr >> 8);
		UBRR0L = (uint8_t)(ubrr);
		UCSR0B = (1 << RXEN0) | (1 << TXEN0);
		UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	}
	
	void sendByte(uint8_t data) {
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = data;
	}
	
	void print(const char* str) {
		while (*str) sendByte(*str++);
	}
	
	void println(const char* str) {
		print(str);
		sendByte('\n');
	}
};

namespace I2C {
	void init() {
		TWBR = 0x20;
		TWSR = 0x00;
	}

	void start() {
		TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
		while (!(TWCR & (1 << TWINT)));
	}

	void stop() {
		TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
		while (TWCR & (1 << TWSTO));
	}

	void write(uint8_t data) {
		TWDR = data;
		TWCR = (1 << TWEN) | (1 << TWINT);
		while (!(TWCR & (1 << TWINT)));
	}

	uint8_t readACK() {
		TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
		while (!(TWCR & (1 << TWINT)));
		return TWDR;
	}

	uint8_t readNACK() {
		TWCR = (1 << TWEN) | (1 << TWINT);
		while (!(TWCR & (1 << TWINT)));
		return TWDR;
	}

	void writeRegister(uint8_t dev, uint8_t reg, uint8_t data) {
		start();
		write(dev << 1);
		write(reg);
		write(data);
		stop();
	}

	uint8_t readRegister(uint8_t dev, uint8_t reg) {
		start();
		write(dev << 1);
		write(reg);
		start();
		write((dev << 1) | 1);
		uint8_t data = readNACK();
		stop();
		return data;
	}

	void readRegisters(uint8_t dev, uint8_t reg, uint8_t* dest, uint8_t n) {
		start();
		write(dev << 1);
		write(reg);
		start();
		write((dev << 1) | 1);
		for (uint8_t i = 0; i < n - 1; i++) dest[i] = readACK();
		dest[n - 1] = readNACK();
		stop();
	}
}

class OrientationManager {
	public:
	float pitch = 0, roll = 0, yaw = 0;
	void init() {
		// init MPU6050
		I2C::writeRegister(MPU_ADDR, 0x6b, 0x00);
		I2C::writeRegister(MPU_ADDR, 0x1b, 0x00);
		I2C::writeRegister(MPU_ADDR, 0x1c, 0x00);

		uint8_t acc_set = (I2C::readRegister(MPU_ADDR, 0x1c) >> 3) & 0b11;
		if (acc_set != 0x00) error("Unexpected accelerometer setting");
		acc_scale = 16384.0f / (1 >> acc_set);

		uint8_t gyr_set = (I2C::readRegister(MPU_ADDR, 0x1b) >> 3) & 0b11;
		if (gyr_set != 0x00) error("Unexpected gyroscope setting");
		gyr_scale = 131.0f / (1 >> gyr_set);
		
		calibrate_mpu();
		
		// init HMC5883L
		I2C::writeRegister(MAG_ADDR, 0x00, 0x70);
		I2C::writeRegister(MAG_ADDR, 0x01, 0xa0);
		I2C::writeRegister(MAG_ADDR, 0x02, 0x00);
		calibrate_mag();
	}
	
	void update(Serial serial) {
		PORTB |= (1 << PB5);
		uint32_t current_time = millis();
		float dt = (current_time - prev_time) / 1000.0;
		prev_time = current_time;
		
		read_mpu();
		read_mag();
		
		// accelerometer
		float ax = ax_raw / acc_scale;
		float ay = ay_raw / acc_scale;
		float az = az_raw / acc_scale;
		
		// gyro
		float gx = ((float)gx_raw - gx_off) / gyr_scale;
		float gy = ((float)gy_raw - gy_off) / gyr_scale;
		float gz = ((float)gz_raw - gz_off) / gyr_scale;
		
		// mag
		float len = sqrt((float)mx_raw * mx_raw + (float)my_raw * my_raw + (float)mz_raw * mz_raw);
		float mx = mx_raw / len;
		float my = my_raw / len;
		float mz = mz_raw / len;
		
		pitch = 0.95 * (pitch + (-gx) * dt) + 0.05 * (atan2(-ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG);
		roll = 0.95 * (roll + gy * dt) + 0.05 * (atan2(-ax, az) * RAD_TO_DEG);
		yaw = 0;
		log(serial, "pitch:%s roll:%s yaw:%s", pitch, roll, yaw);
		
		PORTB &= ~(1 << PB5);
	}
	
	void log(Serial &serial, const char *str, float f1, float f2, float f3) {
		char strs[3][12];
		dtostrf(f1, 12, 4, strs[0]); ltrim(strs[0]);
		dtostrf(f2, 12, 4, strs[1]); ltrim(strs[1]);
		dtostrf(f3, 12, 4, strs[2]); ltrim(strs[2]);
		
		char buf[64];
		sprintf(buf, str, strs[0], strs[1], strs[2]);
		serial.println(buf);
	}
	private:
	float acc_scale = 16384.0, gyr_scale = 131.0, mag_scale = 1.0;
	int16_t gx_off = 0, gy_off = 0, gz_off = 0;
	int16_t ax_raw, ay_raw, az_raw;
	int16_t gx_raw, gy_raw, gz_raw;
	
	int16_t mx_min = -0x8000, my_min = -0x8000, mz_min = -0x8000;
	int16_t my_max = 0x7FFF, mx_max = 0x7FFF, mz_max = 0x7FFF;
	int16_t mx_raw, my_raw, mz_raw;
	
	uint32_t prev_time;
	
	void error(const char* str) {
		Serial serial;
		serial.init(9600);
		serial.println(str);
		PORTB |= (1 << PB5);
	}
	
	void calibrate_mpu() {
		// calculate gyro offsets
		uint32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
		for (uint8_t i = 0; i < 100; i++) {
			read_mpu();
			gx_sum += gx_raw;
			gy_sum += gy_raw;
			gz_sum += gz_raw;
			_delay_ms(10);
		}
		
		gx_off = gx_sum / 100;
		gy_off = gy_sum / 100;
		gz_off = gz_sum / 100;
	}
	
	void calibrate_mag() {
		// handled by continuous calibration
		read_mag();
	}
	
	void read_mpu() {
		uint8_t data[14];
		I2C::readRegisters(MPU_ADDR, 0x3b, data, 14);
		ax_raw = (data[0] << 8) | data[1];
		ay_raw = (data[2] << 8) | data[3];
		az_raw = (data[4] << 8) | data[5];
		gx_raw = (data[8] << 8) | data[9];
		gy_raw = (data[10] << 8) | data[11];
		gz_raw = (data[12] << 8) | data[13];
	}
	
	void read_mag() {
		uint8_t data[6];
		I2C::readRegisters(MAG_ADDR, 0x03, data, 6);
		mx_raw = (data[0] << 8) | data[1];
		my_raw = (data[2] << 8) | data[3];
		mz_raw = (data[4] << 8) | data[5];
		
		// continuous calibration
		if (mx_raw < mx_min)		mx_min = mx_raw;
		else if (mx_raw > mx_max)	mx_max = mx_raw;
		if (my_raw < my_min)		my_min = my_raw;
		else if (my_raw > my_max)	my_max = my_raw;
		if (mz_raw < mz_min)		mz_min = mz_raw;
		else if (mz_raw > mz_max)	mz_max = mz_raw;
	}
};

Serial serial;
OrientationManager ori;

void setup() {
	// set onboard led as output
	DDRB |= (1 << PB5);
	
	// use timer1 (d9, d10) OCR1x
	DDRB |= (1 << PB1) | (1 << PB2);
	TCCR1A |= (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
	TCCR1B |= (1 << CS11);
	
	serial.init(9600);
	I2C::init();
	timer0_init();
	ori.init();
}

void loop() {
	// PORTB ^= (1 << PB5);
	ori.update(serial);
}

int main(void) {
	setup();
	while (1) loop();
}
