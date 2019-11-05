/*
   Abstract RC receiver class

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

#include <stdint.h>
#include <math.h>

#include "datatypes.hpp"

namespace hf {

    class Receiver {

        friend class Hackflight;
        friend class RealBoard;
        friend class MspParser;

        private: 

            const float THROTTLE_MARGIN = 0.1f;
            const float CYCLIC_EXPO     = 0.65f;//循环 exponential(指数)
            const float CYCLIC_RATE     = 0.90f;//循环比率
            const float THROTTLE_EXPO   = 0.20f;//节流 exponential(指数)
            const float AUX_THRESHOLD   = 0.4f;//阈值

	//表达式: 对输入做一些指数/等公式处理==> 然后得到值
            float adjustCommand(float command, uint8_t channel)
            {
                command /= 2;

                if (rawvals[_channelMap[channel]] < 0) {
                    command = -command;
                }

                return command;
            }

            float applyCyclicFunction(float command)
            {
                return rcFun(command, CYCLIC_EXPO, CYCLIC_RATE);
            }

			//取输入的绝对值:
            float makePositiveCommand(uint8_t channel)
            {
                return fabs(rawvals[_channelMap[channel]]);
            }


            static float rcFun(float x, float e, float r)
            {
                return (1 + e*(x*x - 1)) * x * r;
            }

            // [-1,+1] -> [0,1] -> [-1,+1]
            float throttleFun(float x)//[-1,1]
            {
                float mid = 0.5;
                float tmp = (x + 1) / 2 - mid;//[-0.5,0.5]
                float y = tmp>0 ? 1-mid : (tmp<0 ? mid : 1);
                return (mid + tmp*(1-THROTTLE_EXPO + THROTTLE_EXPO * (tmp*tmp) / (y*y))) * 2 - 1;
            }

        protected: 

            // maximum number of channels that any receiver will send (of which we'll use six)
            static const uint8_t MAXCHAN = 8;

            uint8_t _aux1State = 0;
            uint8_t _aux2State = 0;

            float _demandScale = 0;

            // channel indices
            enum {
                CHANNEL_THROTTLE, 
                CHANNEL_ROLL,    
                CHANNEL_PITCH,  
                CHANNEL_YAW,   
                CHANNEL_AUX1,
                CHANNEL_AUX2
            };

            uint8_t _channelMap[6] = {0};

            // These must be overridden for each receiver
			//必须在子类中重写
            virtual bool gotNewFrame(void) = 0;
            virtual void readRawvals(void) = 0;

            // This can be overridden optionally
            virtual void begin(void) { }

            // Software trim
            float _trimRoll = 0;
            float _trimPitch = 0;
            float _trimYaw = 0;

            // Default to non-headless mode
			//默认是:  无头模式
            float headless = false;

            // Raw receiver values in [-1,+1]
			//-1到1:的输入值  8个输入元素
            float rawvals[MAXCHAN] = {0};  

            demands_t demands;

			//获取某一个元素的输入:
            float getRawval(uint8_t chan)
            {
                return rawvals[_channelMap[chan]];
            }

            // Override this if your receiver provides RSSI or other weak-signal detection
			//重写:  用于: 接收信号强度指示
            virtual bool lostSignal(void) { return false; }

            /**
              * channelMap: throttle, roll, pitch, yaw, aux, arm
              */
			//channelMap参数: 的顺序是 throttle, roll, pitch, yaw, aux, arm
            Receiver(const uint8_t channelMap[6], float demandScale=1.0) 
            { 
				//初始化_channelMap[]值:   
					//throttle, roll, pitch, yaw, aux, arm 和 6个输入对应(1到6)
                for (uint8_t k=0; k<6; ++k) {
                    _channelMap[k] = channelMap[k];
                }

                _trimRoll  = 0;
                _trimPitch = 0;
                _trimYaw   = 0;

                _demandScale = demandScale;
            }

			//1)检测接收的数据是否有效:  查看引用
			//2)设置demands 和 _aux1State/_aux2State的值:
            bool getDemands(float yawAngle)
            {
                // Wait till there's a new frame
					//等待, 直到更高进度的帧
                if (!gotNewFrame()) return false;

                // Read raw channel values
                readRawvals();

                // Convert raw [-1,+1] to absolute value
				//把[-1,+1] ,转换为绝对值
                demands.roll  = makePositiveCommand(CHANNEL_ROLL);
                demands.pitch = makePositiveCommand(CHANNEL_PITCH);
                demands.yaw   = makePositiveCommand(CHANNEL_YAW);

                // Apply expo nonlinearity to roll, pitch
				//非线性表达式??: 用于roll和pitch
                demands.roll  = applyCyclicFunction(demands.roll);
                demands.pitch = applyCyclicFunction(demands.pitch);

                // Put sign back on command, yielding [-0.5,+0.5]
				//把值, 调整到 [-0.5,+0.5]
                demands.roll  = adjustCommand(demands.roll, CHANNEL_ROLL);
                demands.pitch = adjustCommand(demands.pitch, CHANNEL_PITCH);
                demands.yaw   = adjustCommand(demands.yaw, CHANNEL_YAW);

                // Add in software trim
				//加入软件: 裁剪?
                demands.roll  += _trimRoll;
                demands.pitch += _trimPitch;
                demands.yaw   += _trimYaw;

                // Support headless mode
				//支持无头模式(无人机-无头模式=>百度)  ==> 即无视yaw旋转,向目标点飞行
                if (headless) {
                    float c = cos(yawAngle);
                    float s = sin(yawAngle);
                    float p = demands.pitch;
                    float r = demands.roll;
                    
					//算出:，测量飞行器相对于地球磁场的角度， ==> 实现无头模式
                    demands.roll  = c*r - s*p;
                }

                // Yaw demand needs to be reversed
				//YAW取相反值
                demands.yaw = -demands.yaw;

                // Pass throttle demand through exponential function
				//油门通过: 指数表达式获取  ===> 缓冲效果??
                demands.throttle = throttleFun(rawvals[_channelMap[CHANNEL_THROTTLE]]);

                // Store auxiliary switch state
				//存放: 2个备用开关, 状态
					//channelMap: throttle, roll, pitch, yaw, aux, arm  ==> aux是aux, arm的输入
                _aux1State = getRawval(CHANNEL_AUX1) >= 0.0 ? (getRawval(CHANNEL_AUX1) > AUX_THRESHOLD ? 2 : 1) : 0;//输入>0.4否: 是取2 ==>0.4到>0.0,取1 ==> <0取0
                _aux2State = getRawval(CHANNEL_AUX2) >= AUX_THRESHOLD ? 1 : 0;//>0.4否==> 是取1/ 不是取0
				

                // Got a new frame
                return true;

            }  // getDemands

		//throttle是否按下:
            bool throttleIsDown(void)
            {
                return getRawval(CHANNEL_THROTTLE) < -1 + THROTTLE_MARGIN;
            }

		//获取2个开关状态:
            virtual uint8_t getAux1State(void)
            {
                return _aux1State;
            }

            virtual uint8_t getAux2State(void)
            {
                return _aux2State;
            }

        public:
		//设置trim:
            void setTrimRoll(float trim)
            {
                _trimRoll = trim;
            }

            void setTrimPitch(float trim)
            {
                _trimPitch = trim;
            }

            void setTrimYaw(float trim)
            {
                _trimYaw = trim;
            }

    }; // class Receiver

} // namespace
