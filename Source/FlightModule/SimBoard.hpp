/*
   Hackflight Board class implementation for MulticopterSim

   Copyright(C) 2019 Simon D.Levy

   MIT License
*/

#pragma once

#include <board.hpp>

#include "../MainModule/OSD.hpp"

class SimBoard : public hf::Board {

    private:
   
        static const uint8_t MAXMOTORS = 10;
		static const uint8_t Q_DIVISOR = 5; // this many gyro readings per quaternion reading

        float _currentTime = 0; // must be float for Hackflight

        double _quat[4] = {0};
        double _gyro[3] = {0};
        double _motors[MAXMOTORS] = {0};

		uint8_t _qcount = 0;

    protected:


		//获取四元素测量的值:
        bool getQuaternion(float & qw, float & qx, float & qy, float & qz)
        {
			_qcount = ((_qcount+1) % Q_DIVISOR);

			if (_qcount) return false;

			//改变实参:
            qw =   _quat[0];
            qx = - _quat[1];// invert X
            qy = - _quat[2];// invert Y
            qz =   _quat[3];

            return true;
        }

		//获取陀螺仪测量的值:
        bool getGyrometer(float & gx, float & gy, float & gz)
        {
			//改变实参:
            gx = _gyro[0];
            gy = _gyro[1];
            gz = _gyro[2];

            return true;
        }

		//对motors[]值, 进行设置 ==> 这样,下面的getMotors才能获取到
        void writeMotor(uint8_t index, float value)
        {
            _motors[index] = value;
        }

        float getTime(void)
        {
            return _currentTime;
        }

    public:

        SimBoard() 
		{
			_qcount = 0;
		}

        virtual ~SimBoard() 
		{ 
		}

		//对_quat(姿态) 和 _gyro(角速度)初始化
		//同时返回:  motors
        void getMotors(const double time, const double quat[4], const double gyro[3], double * motors, uint8_t motorCount)
        {
            _currentTime = time;

            // Copy in quaternion
			//从参数: 拷贝四元数:
            for (uint8_t j=0; j<4; ++j) {
                _quat[j] = quat[j];
            }

            // Copy in gyro
			//从参数拷贝:  拷贝Gyrometer:   陀螺测试仪
            for (uint8_t j=0; j<3; ++j) {
                _gyro[j] = gyro[j];
            }

            // Copy out motors
			//吧旋翼==> 写入到参数motors
            for (uint8_t j=0; j<motorCount; ++j) {
                motors[j] = _motors[j];
            }
        }

}; // class Simboard
