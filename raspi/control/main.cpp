#include "actor.h"
#include "bot.h"
#include <math.h>

const float COMP_ALPHA = 0.99f;         // Alpha for complementary filter (must match training)
const float TIMESTEP = 0.005f;          // Time (sec) between intervals
const float MOTOR_SCALE = 1023.0f;      // Scale motors from [-1, 1] to [-1023, 1023]
const float ENC_TICKS_PER_REV = 12.0f; // Encoder ticks per wheel revolution
const float TIP_THRESHOLD = 0.79f;      // radians (~45 deg), stop motors if exceeded
const int16_t MOTOR_DIR_LEFT = -1;      // Left motor direction
const int16_t MOTOR_DIR_RIGHT = -1;     // Right motor direction
const int32_t ENC_DIR_LEFT = 1;         // Left encoder direction
const int32_t ENC_DIR_RIGHT = 1;        // Right encoder direction
const unsigned long RESET_TIME_MS = 1000; // How long to wait before running again
// Derived constants
const float ENC_TICKS_TO_RADS = (2.0f * M_PI) / ENC_TICKS_PER_REV;
// Globals
BotControl bot;
float pitch = 0.0f;
int32_t prev_enc_left = 0;
int32_t prev_enc_right = 0;
bool tipped = false;

int main() {

	bot.ClearEncoders();
	bot.SetSpeed(0,0);

	while (true) {
		int enc_left;
		int enc_right;
		static int print_counter = 0;

		// Get timestamp for pacing to timestep interval
		unsigned long step_start = micros();

		// Read Imu data
		float gx, gy, gz;
		float ax, ay, az;
		bot.imu.GetAcc(ax, ay, az);
		bot.imu.GetAcc(gx, gy, gz);

		float pitch_rate = -gx * (M_PI / 180.0f);
		float accel_pitch = -atan2f(ay, az);
		pitch = COMP_ALPHA * (pitch + (pitch_rate * TIMESTEP)) + ((1.0f - COMP_ALPHA) * accel_pitch);
		
		bot.GetEncoder(&enc_left, &enc_right);
  		enc_left = ENC_DIR_LEFT * enc_left;
  		enc_right = ENC_DIR_RIGHT * enc_right;
	
		int delta_enc_left  = enc_left  - prev_enc_left;
		int delta_enc_right = enc_right - prev_enc_right;
  		prev_enc_left  = enc_left;
		prev_enc_right = enc_right;
		float wheel_vel_left  = (float)delta_enc_left  * ENC_TICKS_TO_RADS / TIMESTEP;
		float wheel_vel_right = (float)delta_enc_right * ENC_TICKS_TO_RADS / TIMESTEP;
		
		if (fabsf(pitch) > TIP_THRESHOLD) {
    			tipped = true;
  		}

		if (!tipped) {
			float obs[ACTOR_OBS_SIZE] = {
				pitch,
				pitch_rate,
				wheel_vel_left,
				wheel_vel_right,
			};
			float action[ACTOR_ACTION_SIZE];
			actor_forward(obs, action);
			// Clamp actions to [-1, 1]
			action[0] = constrain(action[0], -1.0f, 1.0f);
			action[1] = constrain(action[1], -1.0f, 1.0f);
   			// Calculate actual motor values from normalized values (boost if needed)
    			int16_t motor_left = MOTOR_DIR_LEFT * (int16_t)(action[0] * MOTOR_SCALE);
    			int16_t motor_right = MOTOR_DIR_RIGHT * (int16_t)(action[1] * MOTOR_SCALE);
			
			bot.SetSpeed(motor_left, motor_right);
		}
		else {
			bot.SetSpeed(0, 0);
			if (fabs(pitch) <= 0.3) {
				tipped = false;
				bot.ClearEncoder();
				prev_enc_left = 0;
				prev_enc_right = 0;
				delay(RESET_TIME_MS);
			}
		}

		while (micros() - step_start < (unsigned long)(TIMESTEP * 1e6f));

		#if DEBUG
		if (++print_counter >= 20) {
		    print_counter = 0;
		    std::cout << "pitch=" << pitch << " pitch_rate=" << pitch_rate << " vL=" << wheel_vel_left << " vR=" << wheel_vel_right << " tipped=" << (int)tipped << std::endl;
		#endif

	}
	return 0;
}
