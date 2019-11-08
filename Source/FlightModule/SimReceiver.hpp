/*
   Hackflight Receiver subclass for MulticopterSim Allows us to treat an input
   device (joystick, game controller, R/C transmitter) as a "virtual receiver"
   for the firmware.

   Copyright(C) 2019 Simon D.Levy

   MIT License

   游戏手柄/或摇杆控制:  获取输入
   */

#pragma once

#include <receiver.hpp>

#include "../MainModule/joystick/Joystick.h"

class SimReceiver : public hf::Receiver {

    friend class FHackflightManager;

    private:

		static constexpr uint8_t DEFAULT_CHANNEL_MAP[6] = { 0, 1, 2, 3, 4, 5 };
		static constexpr float DEMAND_SCALE = 1.0f;

		//摇杆:
		Joystick * _joystick;

		// Helps mock up periodic availability of new data frame (output data rate; ODR)
		double _deltaT;
		double _previousTime;

    protected:

		uint8_t getAux1State(void) 
		{
			return Receiver::getAux1State();
		}

		uint8_t getAux2State(void)
		{
			// Always armed!  一直武装?
			return 1;
		}

    public:

		SimReceiver(uint16_t updateFrequency=50)
			: Receiver(DEFAULT_CHANNEL_MAP, DEMAND_SCALE)
		{
			//摇杆:
			_joystick = new Joystick();
			//deltatime: 初始化为 1/50
			_deltaT = 1./updateFrequency;
			_previousTime = 0;
		}

		void begin(void)
		{
		}

		bool gotNewFrame(void)
		{
			// Get a high-fidelity current time value from the OS
			//获取的更高精度的时间:  
			double currentTime = FPlatformTime::Seconds();//从游戏开始,到现在的时间==> FPlatformTime::Seconds()

			if (currentTime-_previousTime > _deltaT) {
				_previousTime = currentTime;
				return true;
			}

			return false;
		}

		void readRawvals(void)
		{
		}

		Joystick::error_t update(void)
		{
			// Joystick::poll() returns zero (okay) or a postive value (error)
			//把手柄状态:  设置到rawvals[]数组中
				//返回0,  或 error
			 _joystick->poll(rawvals);
			 //强制返回0:  后续查看返回error原因!!!
			 return Joystick::error_t::ERROR_NOERROR;
		}

		float GetRawvalsInput(int i)
		{
			return  rawvals[i];
		}

}; // class SimReceiver
