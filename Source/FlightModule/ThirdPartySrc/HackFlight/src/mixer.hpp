/*
   Mixer class

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "filters.hpp"

namespace hf {

    class Mixer {

        friend class Hackflight;
        friend class MspParser;
        friend class RealBoard;

        private:

            // Custom mixer data per motor
			//每个motor的 mixer数据:
            typedef struct motorMixer_t {
                int8_t throttle; // T
                int8_t roll; 	 // A
                int8_t pitch;	 // E
                int8_t yaw;	     // R
            } motorMixer_t;

            // Arbitrary
            static const uint8_t MAXMOTORS = 20;

            float _motorsPrev[MAXMOTORS] = {0};


			//更新每个motor的值:
            void writeMotor(uint8_t index, float value)
            {
                // Avoid sending the motor the same value over and over
				//避免反复发送相同的值:  到 motor
					//更前面的值不同,再返送
                if (_motorsPrev[index] != value) {
					//motor的索引,  对应的值
                    board->writeMotor(index,value);
                }

                _motorsPrev[index] = value;
            }

        protected:

            Board * board;

            motorMixer_t motorDirections[MAXMOTORS];

            Mixer(uint8_t _nmotors)
            {
                nmotors = _nmotors;

                // set disarmed, previous motor values
				//初始值: 给0
                for (uint8_t i = 0; i < nmotors; i++) {
                    motorsDisarmed[i] = 0;
                    _motorsPrev[i] = 0;
                }

            }

            // These are also use by MSP
            float  motorsDisarmed[MAXMOTORS];
			//motors的数量:
            uint8_t nmotors;

			//根据想要达到的角度demands * mixer ==> 得到, 想要的转速  ==> 设置mixer
            void runArmed(demands_t demands)
            {
                // Map throttle demand from [-1,+1] to [0,1]
				//油门范围映射:  [-1,+1] to [0,1]
                demands.throttle = (demands.throttle + 1) / 2;

                float motors[MAXMOTORS];

				//设置motors[i]: 
					//根据想要达到的角度demands * mixer ==> 得到, 想要的转速
                for (uint8_t i = 0; i < nmotors; i++) {
					//根据roll-pitch等: 计算出每个旋翼的值:
						//motorDirections[i] 的值==> 在子类中有定义的:  见 MixerQuadXAP
						//* 混合控制器mixer = 结果
                    motors[i] = 
                        (demands.throttle * motorDirections[i].throttle + 
                         demands.roll     * motorDirections[i].roll +     
                         demands.pitch    * motorDirections[i].pitch +   
                         demands.yaw      * motorDirections[i].yaw);      
                }

				//找到4个motor的最大值:  maxMotor
                float maxMotor = motors[0];
                for (uint8_t i = 1; i < nmotors; i++)
                    if (motors[i] > maxMotor)
                        maxMotor = motors[i];
				
				//motors[i] 设置值:
                for (uint8_t i = 0; i < nmotors; i++) {

                    // This is a way to still have good gyro corrections if at least one motor reaches its max
					//这仍然是一种有良好的陀螺修正, 如果至少一个电机达到最大值
                    if (maxMotor > 1) {
                        motors[i] -= maxMotor - 1;
                    }

                    // Keep motor values in interval [0,1]
					//clamp到:  [0,1]
                    motors[i] = Filter::constrainMinMax(motors[i], 0, 1);
                }

				//更新每个motor的值:  ===> 用 motors[i]
                for (uint8_t i = 0; i < nmotors; i++) {
                    writeMotor(i, motors[i]);
                }
            }

            // This is how we can spin the motors from the GCS
			//从地面工作站(GCS):  来旋转motor
            void runDisarmed(void)
            {
				//更新每个motor的值:  ===> 用 motorsDisarmed[i]
                for (uint8_t i = 0; i < nmotors; i++) {
                    writeMotor(i, motorsDisarmed[i]);
                }
            }

			//设置所有motor转速为0:
            void cutMotors(void)
            {
                for (uint8_t i = 0; i < nmotors; i++) {
                    board->writeMotor(i, 0);
                }
            }

    }; // class Mixer

} // namespace
