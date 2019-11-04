/*
   Angular-velocity-based PID controller for roll, pitch, yaw

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

#include "receiver.hpp"
#include "filters.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {

    // Helper class for all three axes
	//���ٶ�:pid����
    class _AngularVelocityPid : public Pid {

        private: 

            // Arbitrary constants : ����ֵ
            static constexpr float BIG_DEGREES_PER_SECOND = 40.0f; 
            static constexpr float WINDUP_MAX = 6.0f;

            // Converted to radians from degrees in constructor for efficiency
			//ת�� degree  to radians:  �����ٶ�
            float _bigAngularVelocity = 0;

        public:

            void init(const float Kp, const float Ki, const float Kd) 
            {
                Pid::init(Kp, Ki, Kd, WINDUP_MAX);

                // Convert degree parameters to radians for use later
				//ת�� degree  to radians
                _bigAngularVelocity = Filter::deg2rad(BIG_DEGREES_PER_SECOND);
            }

			//����pid:
            float compute(float demand, float angularVelocity)
            {
                // Reset integral on quick angular velocity change
				//���������ٶ�:  ����pid
                if (fabs(angularVelocity) > _bigAngularVelocity) {
                    reset();
                }

                return Pid::compute(demand, angularVelocity);
            }

    };  // class _AngularVelocityPid

	//����PID:  ������yaw roll, pitch 3��pid
    class RatePid : public PidController {

        private: 

            // Aribtrary constants
            static constexpr float BIG_YAW_DEMAND = 0.1f;

            // Rate mode uses a rate controller for roll, pitch
			//yaw roll, pitch��==>���ٶ�pid:
            _AngularVelocityPid _rollPid;
            _AngularVelocityPid _pitchPid;
            _AngularVelocityPid _yawPid;

        public:

            RatePid(const float Kp, const float Ki, const float Kd, const float Kp_yaw, const float Ki_yaw) 
            {
                _rollPid.init(Kp, Ki, Kd);
                _pitchPid.init(Kp, Ki, Kd);
                _yawPid.init(Kp_yaw, Ki_yaw, 0);
            }

			//ͬʱ����3��pid:
            void modifyDemands(state_t & state, demands_t & demands)
            {
                demands.roll  = _rollPid.compute(demands.roll,  state.angularVel[0]);
                demands.pitch = _pitchPid.compute(demands.pitch, state.angularVel[1]);
                demands.yaw   = _yawPid.compute(demands.yaw, state.angularVel[2]);

			//yaw: �����⴦��:
                // Prevent "yaw jump" during correction
				//��ֹ   "yaw jump": ���������������:   �ڽ���ʱ
                demands.yaw = Filter::constrainAbs(demands.yaw, 0.1 + fabs(demands.yaw));

                // Reset yaw integral on large yaw command
				//yaw ���� ���ֵ: ����pid
                if (fabs(demands.yaw) > BIG_YAW_DEMAND) {
                    _yawPid.reset();
                }
            }

			//�����������:  ������3��pid
            virtual void updateReceiver(demands_t & demands, bool throttleIsDown) override
            {
                // Check throttle-down for integral reset
				//�����������:  ������3��pid
                _rollPid.updateReceiver(demands, throttleIsDown);
                _pitchPid.updateReceiver(demands, throttleIsDown);
                _yawPid.updateReceiver(demands, throttleIsDown);
            }

    };  // class RatePid

} // namespace hf
