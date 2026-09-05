#include "bot.h"

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
