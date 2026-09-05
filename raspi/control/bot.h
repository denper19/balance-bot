#pragma once
#include <iostream>
#include <array>
#include <cstdlib>
#include <algorithm>
#include <wiringPi.h>
#include <softPwm.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define LEFT_ENA 40
#define LEFT_ENB 38
#define LEFT_ENC_ENA 24
#define LEFT_ENC_ENB 19

#define RIGHT_ENA 37
#define RIGHT_ENB 35
#define RIGHT_ENC_ENA 36
#define RIGHT_ENC_ENB 12

class Imu {
	private:
		int fd;
		float a_cal[3], g_cal[3];
	public:
		Imu();
		void SetCal(const std::array<float,3>a_cal, const std::array<float,3>g_cal);
		void GetAcc(float& ax, float& ay, float& az);
		void GetGyr(float& gx, float& gy, float& gz);
};

class BotControl {
	private:
		int left_encoder_tick, right_encoder_tick;
		static BotControl* instance;
	public:
		Imu imu;
		BotControl();
		void ClearEncoders();
		void GetEncoders(int& left, int& right);
		void SetEncoders(const int& left, const int& right);
		void UpdateEncoders(const int motor);
		static void UpdateLeftEncoder() {instance->UpdateEncoders(LEFT_MOTOR);}
		static void UpdateRightEncoder() {instance->UpdateEncoders(RIGHT_MOTOR);};
		void SetSpeed(const int& left, const int& right);
};
