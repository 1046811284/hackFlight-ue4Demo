/*
   Positon-hold PID controller using optical flow (body-frame velocity)

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#pragma once

#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {

	//无人机光流模块使用技巧
			//光流模块在无 GPS 环境下，课实时检测飞机水平移动距离，实现对四轴无人机长时间的稳定悬停
    class FlowHoldPid : public PidController {

        friend class Hackflight;

        private: 

            // Helper class
            class _FlowVelocityPid : public VelocityPid {

                private:

                    // Arbitrary constants
                    static constexpr float PILOT_VELXY_MAX  = 2.5f; // http://ardupilot.org/copter/docs/altholdmode.html

                public:
					
                    void init(float Kp, float Ki)
                    {
                        VelocityPid::init(Kp, Ki, 0);
                    }

					//velocity: 当前速度
					//demand: demands.roll/demands.pitch ==> 如果roll是否是死区 (roll不在死区==>那么悬停会参数左右位移)
						//如果pitch不在死区:  前后位移
                    void update(float & demand, float velocity)
                    {
						//根据油门是否在死区(是否按下):  决定油门值
						//demand死区:  目标速度0:  在固定位置飞行
						//demand非死区:  demand *  2*PILOT_VELXY_MAX  ==> 产生,移动位置;(前后左右) ==> 然后死区后,又固定位置飞行
                        demand = VelocityPid::compute(demand, 0, 2*PILOT_VELXY_MAX, velocity);

                    }

            }; // _FlowVelocityPid

            _FlowVelocityPid _rollPid;
            _FlowVelocityPid _pitchPid;

        protected:

			//demands_t:  roll/pitch/yaw/y油门  ==> 当前状态
			//state_t:  目标状态
            void modifyDemands(state_t & state, demands_t & demands)
            {
				//使用update: 实现稳定悬停
                _rollPid.update(demands.roll,  state.bodyVel[1]);
                _rollPid.update(demands.pitch, state.bodyVel[0]);
            }

            virtual bool shouldFlashLed(void) override 
            {
                return true;
            }

        public:

            FlowHoldPid(const float Kp, float Ki)
            {
                _rollPid.init(Kp, Ki);
                _pitchPid.init(Kp, Ki);
            }

    };  // class FlowHoldPid

} // namespace hf
