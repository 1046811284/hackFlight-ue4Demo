/*
   PID controller for Level mode

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

	//2个轴角度的pid: roll和pitch
    class LevelPid : public PidController {

        private:

            // Helper class
			//定义单个轴的pid:   角度pid
            class _AnglePid : public Pid {

                private:

					//最大角度: roll /pitch的
                    static constexpr float MAX_ANGLE_DEGREES = 45;

                    // Maximum roll pitch demand is +/-0.5, so to convert demand to 
                    // angle for error computation, we multiply by the folling amount:
					//roll/pitch的乘数:    
						//1° = π / 180 ≈ 0.01745 rad
						//1rad = 180 / π = 57.30°
						//Filter::deg2rad:  转换为度数
                    float _demandMultiplier = 2 * Filter::deg2rad(MAX_ANGLE_DEGREES);

                public:

                    void init(const float Kp) 
                    {
                        Pid::init(Kp, 0, 0);
                    }

                    float compute(float demand, float angle)
                    {
                        return Pid::compute(demand*_demandMultiplier, angle);
                    }

            }; // class _AnglePid


			//2个轴的pid: roll和pitch
            _AnglePid _rollPid;
            _AnglePid _pitchPid;

        public:

            LevelPid(float rollLevelP, float pitchLevelP)
            {
                _rollPid.init(rollLevelP);
                _pitchPid.init(pitchLevelP);
            }

            LevelPid(float rollPitchLevelP)
                : LevelPid(rollPitchLevelP, rollPitchLevelP)
            {
            }

			//计算2个轴的:  pid
            void modifyDemands(state_t & state, demands_t & demands)
            {
                demands.roll  = _rollPid.compute(demands.roll, state.rotation[0]); 
                demands.pitch = _pitchPid.compute(demands.pitch, state.rotation[1]);
            }

    };  // class LevelPid

} // namespace
