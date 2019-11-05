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

	//包含多个pid控制: =====>如同时包含3个轴的 (XYZ的3个轴)
    class PidController {

        friend class Hackflight;

        protected:

        static constexpr float STICK_DEADBAND = 0.10;

		//需求: 同时计算3个方向的pid
        virtual void modifyDemands(state_t & state, demands_t & demands) = 0;

        virtual bool shouldFlashLed(void) { return false; }
		
		//超出最大范围:  reset:
        virtual void updateReceiver(demands_t & demands, bool throttleIsDown) { (void)demands; (void)throttleIsDown; }

		//辅助开关, 状态
        uint8_t auxState = 0;

    };  // class PidController

    // PID controller for a single degree of freedom
	//pid类:   ===> 单个轴的 (XYZ的 一个轴)
    class Pid {

        private: 

            // PID constants: pid系数
            float _Kp = 0;
            float _Ki = 0;
            float _Kd = 0;

            // Accumulated values
			//初始值:
            float _lastError   = 0;
            float _errorI      = 0;
            float _deltaError1 = 0;
            float _deltaError2 = 0;

            // For deltaT-based controllers
			//用于deltatime计算:
            float _previousTime = 0;
     
            // Prevents integral windup
			//防止积分结束
            float _windupMax = 0;

        public:

			//初始化3个系数:
            void init(const float Kp, const float Ki, const float Kd, const float windupMax=0.4) 
            {
                // Set constants
                _Kp = Kp;
                _Ki = Ki;
                _Kd = Kd;
                _windupMax = windupMax;

                // Initialize error integral, previous value
				//初始化误差积分,前一个值, ==>000
                reset();
            }

			//计算pid:
				//target: 目标值    actual: 实际值
			//实现pid公式:  查看蓝图注释
            float compute(float target, float actual)
            {
                // Compute error as scaled target minus actual
				//误差:
                float error = target - actual;

                // Compute P term: 
				//计算P
                float pterm = error * _Kp;

                // Compute I term
				//计算I:
                float iterm = 0;
                if (_Ki > 0) { // optimization
                    _errorI = Filter::constrainAbs(_errorI + error, _windupMax); // avoid integral windup
                    iterm =  _errorI * _Ki;
                }

                // Compute D term
				//计算D:
                float dterm = 0;
                if (_Kd > 0) { // optimization
                    float deltaError = error - _lastError;
                    dterm = (_deltaError1 + _deltaError2 + deltaError) * _Kd; 
                    _deltaError2 = _deltaError1;
                    _deltaError1 = deltaError;
                    _lastError = error;
                }

				//最终公式:  PID之= P + I +D;
                return pterm + iterm + dterm;
            }

			//throttleIsDown:   油门向下???
            void updateReceiver(demands_t & demands, bool throttleIsDown)
            {
                (void)demands; 

                // When landed, reset integral component of PID
				//当: 着陆时==>重新设置PID计算
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


	//速度PID:  查看应用处==> 保持一定高度飞行时 ==> 用compute返回油门值
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
            // @param    demand     输入值
            // @param    inBandTargetVelocity   inBand速度
            // @param    outOfBandTargetScale    outOfBand的缩放Scale
            // @param    actualVelocity   实际速度
            //************************************
            float compute(float demand, float inBandTargetVelocity, float outOfBandTargetScale, float actualVelocity)
            {
                _didReset = false;

			//====================>计算targetVelocity:
				// Is throttle stick in deadband?
				   //死区: 油门绝对值 < 0.1 : 即油门没有按
				bool inBand = fabs(demand) < STICK_DEADBAND;

				// Reset controller when moving into deadband
				   //当从非死区, 到死区时 ==> 需要重新设置pid:
				if (inBand && !_inBandPrev) {
					reset();
					_didReset = true;
				}
				_inBandPrev = inBand;


				// Target velocity is a setpoint inside deadband, scaled constant outside
				   //目标速度: 
					   //死区时:  为inBandTargetVelocity  ==>固定高度飞行
					   //非死区时:  outOfBandTargetScale * demand ==>  油门 * outOfBandTargetScale
				float targetVelocity = inBand ? inBandTargetVelocity : outOfBandTargetScale * demand;

                // Run velocity PID controller to get correction
				//运行PID计算公式:  得到正确的值return
                return Pid::compute(targetVelocity, actualVelocity);
            }

            bool didReset(void)
            {
                return _didReset;
            }

    }; // class VelocityPid

} // namespace hf
