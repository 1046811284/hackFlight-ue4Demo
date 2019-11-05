/*
   Abstract class for PID controllers, plus helper classes

   Copyright (c) 2018 Simon D. Levy

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
#include "filters.hpp"

namespace hf {

	//�������pid����: =====>��ͬʱ����3����� (XYZ��3����)
    class PidController {

        friend class Hackflight;

        protected:

        static constexpr float STICK_DEADBAND = 0.10;

		//����: ͬʱ����3�������pid
        virtual void modifyDemands(state_t & state, demands_t & demands) = 0;

        virtual bool shouldFlashLed(void) { return false; }
		
		//�������Χ:  reset:
        virtual void updateReceiver(demands_t & demands, bool throttleIsDown) { (void)demands; (void)throttleIsDown; }

		//��������, ״̬
        uint8_t auxState = 0;

    };  // class PidController

    // PID controller for a single degree of freedom
	//pid��:   ===> ������� (XYZ�� һ����)
    class Pid {

        private: 

            // PID constants: pidϵ��
            float _Kp = 0;
            float _Ki = 0;
            float _Kd = 0;

            // Accumulated values
			//��ʼֵ:
            float _lastError   = 0;
            float _errorI      = 0;
            float _deltaError1 = 0;
            float _deltaError2 = 0;

            // For deltaT-based controllers
			//����deltatime����:
            float _previousTime = 0;
     
            // Prevents integral windup
			//��ֹ���ֽ���
            float _windupMax = 0;

        public:

			//��ʼ��3��ϵ��:
            void init(const float Kp, const float Ki, const float Kd, const float windupMax=0.4) 
            {
                // Set constants
                _Kp = Kp;
                _Ki = Ki;
                _Kd = Kd;
                _windupMax = windupMax;

                // Initialize error integral, previous value
				//��ʼ��������,ǰһ��ֵ, ==>000
                reset();
            }

			//����pid:
				//target: Ŀ��ֵ    actual: ʵ��ֵ
			//ʵ��pid��ʽ:  �鿴��ͼע��
            float compute(float target, float actual)
            {
                // Compute error as scaled target minus actual
				//���:
                float error = target - actual;

                // Compute P term: 
				//����P
                float pterm = error * _Kp;

                // Compute I term
				//����I:
                float iterm = 0;
                if (_Ki > 0) { // optimization
                    _errorI = Filter::constrainAbs(_errorI + error, _windupMax); // avoid integral windup
                    iterm =  _errorI * _Ki;
                }

                // Compute D term
				//����D:
                float dterm = 0;
                if (_Kd > 0) { // optimization
                    float deltaError = error - _lastError;
                    dterm = (_deltaError1 + _deltaError2 + deltaError) * _Kd; 
                    _deltaError2 = _deltaError1;
                    _deltaError1 = deltaError;
                    _lastError = error;
                }

				//���չ�ʽ:  PID֮= P + I +D;
                return pterm + iterm + dterm;
            }

			//throttleIsDown:   ��������???
            void updateReceiver(demands_t & demands, bool throttleIsDown)
            {
                (void)demands; 

                // When landed, reset integral component of PID
				//��: ��½ʱ==>��������PID����
                if (throttleIsDown) {
                    reset();
                }
            }

            void reset(void)
            {
                _errorI = 0;
                _lastError = 0;
                _previousTime = 0;
            }

    };  // class Pid


	//�ٶ�PID:  �鿴Ӧ�ô�==> ����һ���߶ȷ���ʱ ==> ��compute��������ֵ
    // Velocity-based PID controller
    class VelocityPid : public Pid {

        private:

            static constexpr float STICK_DEADBAND = 0.10;

            bool _inBandPrev = false;
            bool _didReset = false;

        public:

            void init(float Kp, float Ki, float Kd)
            {
                Pid::init(Kp, Ki, Kd);

                _inBandPrev = false;
                _didReset = false;
            }


            //************************************
            // @param    demand     ����ֵ
            // @param    inBandTargetVelocity   inBand�ٶ�
            // @param    outOfBandTargetScale    outOfBand������Scale
            // @param    actualVelocity   ʵ���ٶ�
            //************************************
            float compute(float demand, float inBandTargetVelocity, float outOfBandTargetScale, float actualVelocity)
            {
                _didReset = false;

			//====================>����targetVelocity:
				// Is throttle stick in deadband?
				   //����: ���ž���ֵ < 0.1 : ������û�а�
				bool inBand = fabs(demand) < STICK_DEADBAND;

				// Reset controller when moving into deadband
				   //���ӷ�����, ������ʱ ==> ��Ҫ��������pid:
				if (inBand && !_inBandPrev) {
					reset();
					_didReset = true;
				}
				_inBandPrev = inBand;


				// Target velocity is a setpoint inside deadband, scaled constant outside
				   //Ŀ���ٶ�: 
					   //����ʱ:  ΪinBandTargetVelocity  ==>�̶��߶ȷ���
					   //������ʱ:  outOfBandTargetScale * demand ==>  ���� * outOfBandTargetScale
				float targetVelocity = inBand ? inBandTargetVelocity : outOfBandTargetScale * demand;

                // Run velocity PID controller to get correction
				//����PID���㹫ʽ:  �õ���ȷ��ֵreturn
                return Pid::compute(targetVelocity, actualVelocity);
            }

            bool didReset(void)
            {
                return _didReset;
            }

    }; // class VelocityPid

} // namespace hf
