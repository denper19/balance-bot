#include "bot.h"

int main() {
	BotControl bot;
	int left_motor, right_motor;
	while (true) {
		bot.GetEncoders(left_motor, right_motor);
		std::cout << "Left motor count: "  << left_motor 
		          << "Right motor count: " << right_motor << std::endl;
	}
	return 0;
}
