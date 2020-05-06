#ifndef __DCSBIOS_POTS_H
#define __DCSBIOS_POTS_H

#include <math.h>
#include "Arduino.h"
#include "PollingInput.h"

namespace DcsBios {

	template <unsigned int pollInterval = 5, unsigned int hysteresis = 128, unsigned int ewma_divisor = 5>
	class PotentiometerEWMA : PollingInput {
		private:
			void pollInput() {
				unsigned char now = (unsigned char)millis();
				if ((unsigned char)(now - lastPollTime_) < pollInterval) return;
				lastPollTime_ = now;
				
				unsigned int state = map(analogRead(pin_), 0, 1023, 0, 65535);
				accumulator += ((float)state - accumulator) / (float)ewma_divisor;
				state = (unsigned int)accumulator;
				
				if (((lastState_ > state && (lastState_ - state > hysteresis)))
				|| ((state > lastState_) && (state - lastState_ > hysteresis))
				|| ((state > (65535 - hysteresis) && state > lastState_))
				|| ((state < hysteresis && state < lastState_))
				) {
					char buf[6];
					utoa(state, buf, 10);
					if (tryToSendDcsBiosMessage(msg_, buf))
						lastState_ = state;
				}
			}
			const char* msg_;
			char pin_;
			unsigned int lastState_;
			float accumulator;
			unsigned char lastPollTime_;
			
		public:
			PotentiometerEWMA(const char* msg, char pin) {
				msg_ = msg;
				pin_ = pin;
				pinMode(pin_, INPUT);
				lastState_ = (float)map(analogRead(pin_), 0, 1023, 0, 65535);
				lastPollTime_ = (unsigned char)millis();
			}
	};

	typedef PotentiometerEWMA<> Potentiometer;

	
	typedef struct PotentiometerPositionStruct {unsigned int position; byte min; byte max;} PotentiometerPosition;

	template <unsigned int pollInterval =  5, unsigned int hysteresis = 128, unsigned int ewma_divisor = 5>
	class PotentiometerMultipleSwitch : PollingInput {
		private:
			void pollInput() {
				unsigned char now = (unsigned char)millis();
				if ((unsigned char)(now - lastPollTime_) < pollInterval) return;
				lastPollTime_ = now;
				
				unsigned int state = map(analogRead(pin_), 0, 1023, 0, 65535);
				accumulator += ((float)state - accumulator) / (float)ewma_divisor;
				state = (unsigned int)accumulator;
				
				
				tryToSendDcsBiosMessage("DEBUG_POSITION_" + values_[0].position, "0");

				if (((lastState_ > state && (lastState_ - state > hysteresis)))
				|| ((state > lastState_) && (state - lastState_ > hysteresis))
				|| ((state > (65535 - hysteresis) && state > lastState_))
				|| ((state < hysteresis && state < lastState_))
				) {
					
					unsigned int i;
					for (i = 0; i < numberOfPositions_; i++) {
						
						float min = map(values_[i].min, 0, range_, 0, 65535);
						float max = map(values_[i].max, 0, range_, 0, 65535);

						if (state >= min && state <= max) {
							lastState_ = state;
							tryToSendDcsBiosMessage(msg_, values_[i].position);							
							break;
						}
					}

					
				}
			}

			const char* msg_;
			char pin_;
			PotentiometerPosition* values_;
			size_t numberOfPositions_;
			unsigned int range_;
			unsigned int lastState_;
			float accumulator;
			unsigned char lastPollTime_;
			
		public:
			PotentiometerMultipleSwitch(const char* msg, char pin, PotentiometerPosition* values, unsigned int range) {
				msg_ = msg;
				pin_ = pin;
				values_ = values;
				range_ = range;
				numberOfPositions_ = sizeof values_ / sizeof values_[0];
				pinMode(pin_, INPUT);
				lastState_ = (float)map(analogRead(pin_), 0, 1023, 0, 65535);
				lastPollTime_ = (unsigned char)millis();
			}

		typedef PotentiometerMultipleSwitch<> PotentiometerSwitch;
	};	
	
		
	
}

#endif
