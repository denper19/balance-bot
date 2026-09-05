#include "bot.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

namespace {

bool i2c_write_byte(int file, uint8_t reg, uint8_t value) {
	uint8_t buf[2] = {reg, value};
	return write(file, buf, 2) == 2;
}

bool i2c_read_bytes(int file, uint8_t reg, uint8_t* buffer, int length) {
	if (write(file, &reg, 1) != 1) return false;
	return read(file, buffer, length) == length;
}

} // namespace

Imu::Imu() {
	fd = open("/dev/i2c-1", O_RDWR);
	if (fd < 0) {
		std::cout << "Failed to open the I2C bus. Ensure I2C is enabled." << std::endl;
		return;
	}

	if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
		std::cout << "Failed to acquire bus access/talk to MPU6050." << std::endl;
		return;
	}

	i2c_write_byte(fd, PWR_MGMT_1, 0x00); // wake the MPU6050 (starts in sleep mode)
}

void Imu::SetCal(const std::array<float,3> a_cal_in, const std::array<float,3> g_cal_in) {
	for (int i = 0; i < 3; i++) {
		a_cal[i] = a_cal_in[i];
		g_cal[i] = g_cal_in[i];
	}
}

void Imu::GetAcc(float& ax, float& ay, float& az) {
	uint8_t data[14];
	i2c_read_bytes(fd, ACCEL_XOUT_H, data, 14);

	int16_t raw_ax = (data[0] << 8) | data[1];
	int16_t raw_ay = (data[2] << 8) | data[3];
	int16_t raw_az = (data[4] << 8) | data[5];

	ax = raw_ax / ACCEL_SCALE - a_cal[0];
	ay = raw_ay / ACCEL_SCALE - a_cal[1];
	az = raw_az / ACCEL_SCALE - a_cal[2];
}

void Imu::GetGyr(float& gx, float& gy, float& gz) {
	uint8_t data[14];
	i2c_read_bytes(fd, ACCEL_XOUT_H, data, 14);

	int16_t raw_gx = (data[8]  << 8) | data[9];
	int16_t raw_gy = (data[10] << 8) | data[11];
	int16_t raw_gz = (data[12] << 8) | data[13];

	gx = raw_gx / GYRO_SCALE - g_cal[0];
	gy = raw_gy / GYRO_SCALE - g_cal[1];
	gz = raw_gz / GYRO_SCALE - g_cal[2];
}

BotControl* BotControl::instance = nullptr;

BotControl::BotControl() {
	instance = this;
	left_encoder_tick = 0;
	right_encoder_tick = 0;
	imu.SetCal({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});

	if (wiringPiSetupPhys() < 0) {
		std::cout << "Failed to init wiring pi" << std::endl;
	}

	pinMode(LEFT_ENA, OUTPUT);
	pinMode(RIGHT_ENA, OUTPUT);
	softPwmCreate(LEFT_ENB, 0, 255);
	softPwmCreate(RIGHT_ENB, 0, 255);

	pinMode(LEFT_ENC_ENA, INPUT);
	pinMode(LEFT_ENC_ENB, INPUT);
	pinMode(RIGHT_ENC_ENA, INPUT);
	pinMode(RIGHT_ENC_ENB, INPUT);

	// open-drain encoder outputs float when not triggered - pull them up so noise doesn't fire spurious edges
	pullUpDnControl(LEFT_ENC_ENA, PUD_UP);
	pullUpDnControl(LEFT_ENC_ENB, PUD_UP);
	pullUpDnControl(RIGHT_ENC_ENA, PUD_UP);
	pullUpDnControl(RIGHT_ENC_ENB, PUD_UP);

	// setup interrupts change
	wiringPiISR(LEFT_ENC_ENA, INT_EDGE_BOTH, &BotControl::UpdateLeftEncoder);
	wiringPiISR(RIGHT_ENC_ENA, INT_EDGE_BOTH, &BotControl::UpdateRightEncoder);
}

void BotControl::UpdateEncoders(const int motor)
{
	
	int pin_b;
	int* tickPtr = nullptr;

	if (motor == LEFT_MOTOR) {
		pin_b = LEFT_ENC_ENB;
		tickPtr = &left_encoder_tick;
	}
	else if (motor == RIGHT_MOTOR){
		pin_b = RIGHT_ENC_ENB;
		tickPtr = &right_encoder_tick;
	}

	int state_b = digitalRead(pin_b);

	if (state_b > 0){
		(*tickPtr)++;
	}
	else{ // reverse
		(*tickPtr)--;
	}
}	

void BotControl::ClearEncoders() {
	left_encoder_tick = 0;
	right_encoder_tick = 0;
}

void BotControl::GetEncoders(int& left, int& right) {
	left = left_encoder_tick;
	right = right_encoder_tick;
}

void BotControl::SetEncoders(const int& left, const int& right) {
	left_encoder_tick = left;
	right_encoder_tick = right;
}

void BotControl::SetSpeed(const int& left, const int& right) {
	digitalWrite(LEFT_ENA, left >= 0 ? HIGH : LOW);
	softPwmWrite(LEFT_ENB, std::min(std::abs(left), 255));

	digitalWrite(RIGHT_ENA, right >= 0 ? HIGH : LOW);
	softPwmWrite(RIGHT_ENB, std::min(std::abs(right), 255));
}
