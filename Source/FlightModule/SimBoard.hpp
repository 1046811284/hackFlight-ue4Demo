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


		//��ȡ��Ԫ�ز�����ֵ:
        bool getQuaternion(float & qw, float & qx, float & qy, float & qz)
        {
			_qcount = ((_qcount+1) % Q_DIVISOR);

			if (_qcount) return false;

			//�ı�ʵ��:
            qw =   _quat[0];
            qx = - _quat[1];// invert X
            qy = - _quat[2];// invert Y
            qz =   _quat[3];

            return true;
        }

		//��ȡ�����ǲ�����ֵ:
        bool getGyrometer(float & gx, float & gy, float & gz)
        {
			//�ı�ʵ��:
            gx = _gyro[0];
            gy = _gyro[1];
            gz = _gyro[2];

            return true;
        }

		//��motors[]ֵ, �������� ==> ����,�����getMotors���ܻ�ȡ��
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

		//��_quat(��̬) �� _gyro(���ٶ�)��ʼ��
		//ͬʱ����:  motors
        void getMotors(const double time, const double quat[4], const double gyro[3], double * motors, uint8_t motorCount)
        {
            _currentTime = time;

            // Copy in quaternion
			//�Ӳ���: ������Ԫ��:
            for (uint8_t j=0; j<4; ++j) {
                _quat[j] = quat[j];
            }

            // Copy in gyro
			//�Ӳ�������:  ����Gyrometer:   ���ݲ�����
            for (uint8_t j=0; j<3; ++j) {
                _gyro[j] = gyro[j];
            }

            // Copy out motors
			//������==> д�뵽����motors
            for (uint8_t j=0; j<motorCount; ++j) {
                motors[j] = _motors[j];
            }
        }

}; // class Simboard
