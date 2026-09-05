#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

// Helper to write a single byte to an I2C register
bool i2c_write_byte(int file, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return write(file, buf, 2) == 2;
}

// Helper to read contiguous data registers
bool i2c_read_bytes(int file, uint8_t reg, uint8_t* buffer, int length) {
    if (write(file, &reg, 1) != 1) return false;
    return read(file, buffer, length) == length;
}

int main() {
    // 1. Open the I2C bus device
    int file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open the I2C bus. Ensure I2C is enabled." << std::endl;
        return 1;
    }

    // 2. Set the target I2C slave address
    if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "Failed to acquire bus access/talk to slave." << std::endl;
        close(file);
        return 1;
    }

    // 3. Wake up the MPU6050 (it starts in sleep mode by default)
    if (!i2c_write_byte(file, PWR_MGMT_1, 0b00000000)) {
        std::cerr << "Failed to wake up MPU6050." << std::endl;
        close(file);
        return 1;
    }

    std::cout << "MPU6050 Initialized Successfully.\n" << std::endl;

    // Scale factors based on default ranges (Accel: +/- 2g, Gyro: +/- 250 deg/s)
    const float ACCEL_SCALE = 16384.0;
    const float GYRO_SCALE = 131.0;

    bool disable_output=true;

    const int N = 2000;

    // final acc
    float axs = 0.0;
    float ays = 0.0;
    float azs = 0.0;
    // final gyro
    float gxs = 0.0;
    float gys = 0.0;
    float gzs = 0.0;

    for(int i = 0; i < N; i++) {
        uint8_t data[14]; // 14 registers hold all sensor readings

        // Read 14 bytes starting from ACCEL_XOUT_H
        if (i2c_read_bytes(file, ACCEL_XOUT_H, data, 14)) {
            // Combine high and low bytes (bit manipulation)
            int16_t raw_ax = (data[0] << 8) | data[1];
            int16_t raw_ay = (data[2] << 8) | data[3];
            int16_t raw_az = (data[4] << 8) | data[5];
            int16_t raw_temp = (data[6] << 8) | data[7];
            int16_t raw_gx = (data[8] << 8) | data[9];
            int16_t raw_gy = (data[10] << 8) | data[11];
            int16_t raw_gz = (data[12] << 8) | data[13];

            // Convert raw values to human-readable measurements
            float ax = raw_ax / ACCEL_SCALE;
            float ay = raw_ay / ACCEL_SCALE;
            float az = raw_az / ACCEL_SCALE;
            float gx = raw_gx / GYRO_SCALE;
            float gy = raw_gy / GYRO_SCALE;
            float gz = raw_gz / GYRO_SCALE;
            float temp = (raw_temp / 340.0) + 36.53; // Formula from datasheet

	    axs += ax; ays += ay; azs += az;
	    gxs += gx; gys += gy; gzs += gz;

	    if(!disable_output)
	    {
		    // Print output
		    std::cout << "\r"
			      << "N=" << i
			      << " Accel (g): X=" << ax << " Y=" << ay << " Z=" << az << " | "
			      << "Gyro (°/s): X=" << gx << " Y=" << gy << " Z=" << gz << " | "
			      << "Temp: " << temp << "°C" << std::flush;
	    }
        } 
	else 
	{
            std::cerr << "\nError reading data from MPU6050." << std::endl;
        }

        usleep(100000); // Sample every 100ms
    }


    axs = axs / N; ays = ays / N; azs = azs / N;
    gxs = gxs / N; gys = gys / N; gzs = gzs / N;
    std::cout << "Final measurements: "
	      << "Accel (g): X=" << axs << " Y=" << ays << " Z=" << azs << " | "
	      << "Gyro (°/s): X=" << gxs << " Y=" << gys << " Z=" << gzs << std::flush;

    close(file);
    return 0;
}

