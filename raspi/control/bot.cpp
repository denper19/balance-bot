#include "bot.h"

BotControl::BotControl() {

	if (wiringPiSetupGpio() < 0) {
		std::cout << "Failed to init wiring pi" << std::endl;
	}
	
	pinMode(LEFT_ENA, OUTPUT);
	pinMode(LEFT_ENB, OUTPUT);
	pinMode(RIGHT_ENA, OUTPUT);
	pinMode(RIGHT_ENB, OUTPUT);

	pinMode(LEFT_ENC_ENA, INPUT);
	pinMode(LEFT_ENC_ENB, INPUT);
	pinMode(RIGHT_ENC_ENA, INPUT);
	pinMode(RIGHT_ENC_ENB, INPUT);

	// setup interrupts change
	wiringPiISR(LEFT_ENC_ENA, INT_EDGE_BOTH, &BotControl::UpdateLeftEncoder);
	wiringPiISR(RIGHT_ENC_ENA, INT_EDGE_BOTH, &BotControl::UpdateRightEncoder);
}

void BotControl::UpdateEncoders(int motor)
{
	
	int pin_a, pin_b;
	int* tickPtr = nullptr;

	if (motor == LEFT_MOTOR) {
		pin_a = LEFT_ENC_ENA;
		pin_b = LEFT_ENC_ENB;
		tickPtr = &left_encoder_tick;	
	}
	else if (motor == RIGHT_MOTOR){
		pin_a = RIGHT_ENC_ENA;
		pin_b = RIGHT_ENC_ENB;
		tickPtr = &right_encoder_tick;	
	}

	int state_a = digitalRead(pin_a);	
	int state_b = digitalRead(pin_b);

	if (state_a > state_b){
		(*tickPtr)++;
	}
	else{
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

void BotControl::SetEncoders(int& left, int& right) {
	left_encoder_tick = left;
	right_encoder_tick = right;
}
