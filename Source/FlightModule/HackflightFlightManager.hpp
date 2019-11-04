/*
   MulticopterSim FlightManager class implementation using a stub

   Just spins propellers

   Copyright(C) 2019 Simon D.Levy

   MIT License
*/

#include "../MainModule/FlightManager.hpp"

#include <hackflight.hpp>

// PID controllers
#include <pidcontrollers/level.hpp>
#include <pidcontrollers/rate.hpp>
#include <pidcontrollers/althold.hpp>
#include <pidcontrollers/flowhold.hpp>

// Mixer
#include <mixers/quadxap.hpp>

#include "SimReceiver.hpp"
#include "SimBoard.hpp"
#include "SimSensors.hpp"

class FHackflightFlightManager : public FFlightManager {

    private:

  //=========================PID tuning

		// Rate:  比率pid==>3个<角速度>pid
		hf::RatePid ratePid = hf::RatePid(
			.01,	// Kp_roll_pitch
			.01,	// Ki_roll_pitch
			.01,	// Kd_roll_pitch
			.025,	// Kp_yaw 
			.01); 	// Ki_yaw

        // Level:  2个轴<角度>的pid: roll和pitch
        hf::LevelPid levelPid = hf::LevelPid(0.8);

        // Alt-hold:  保持一定高度飞行的pid
        hf::AltitudeHoldPid althold = hf::AltitudeHoldPid(
                10.00f, // altHoldPosP
                1.00f,  // altHoldVelP
                0.01f,  // altHoldVelI
                0.10f); // altHoldVelD


        // Pos-hold (via simulated optical flow) ==>位置控制,(通过模拟光学流)
		//无人机光流模块使用技巧
			//光流模块在无 GPS 环境下，课实时检测飞机水平移动距离，实现对四轴无人机长时间的稳定悬停
        hf::FlowHoldPid flowhold = hf::FlowHoldPid(0.05, 0.05);



   //==========================控制组件
        // Main firmware
        hf::Hackflight _hackflight;


        // Flight-controller board (board类的作用:)
			//1)获取四元数
			//2)获取 陀螺仪的速率
			//3)发送命令到 motor
			//4)获取当前时间
        SimBoard _board;

        // "Receiver" (joystick/gamepad)
		//手柄-摇杆输入: Receive ==> 调用update,获取手柄输入
        SimReceiver _receiver;

        // Mixer:   pid给出的输出, 经过mixer==> 给到4个Motor    查一下源码???
        hf::MixerQuadXAP _mixer;

        // "Sensors" (get values from dynamics)   ===> 查看源码??
        SimSensors * _sensors = NULL;

    public:

        // Constructor
        FHackflightFlightManager(MultirotorDynamics * dynamics) 
            : FFlightManager(dynamics) 
        {
            // Start Hackflight firmware, indicating already armed
			//初始化:  &_board, &_receiver, &_mixer  ==>初始化这3个
            _hackflight.init(&_board, &_receiver, &_mixer, true);

            // Add simulated sensor suite
            _sensors = new SimSensors(_dynamics);
            _hackflight.addSensor(_sensors);


		//pid控制器:
			// Add altitude-hold and position-hold PID controllers in switch position 1
			_hackflight.addPidController(&althold, 1);
			_hackflight.addPidController(&flowhold, 1);

			// Add rate and level PID controllers for all aux switch positions
			_hackflight.addPidController(&levelPid);
			_hackflight.addPidController(&ratePid);
		}

        virtual ~FHackflightFlightManager(void)
        {
        }


        virtual void getMotors(const double time, const MultirotorDynamics::state_t & state, double * motorvals) override
        {
			//update输入:
				//初始化:  _receiver.rawvals[] 成员
            Joystick::error_t joystickError = _receiver.update();

			//输入错误码:
            switch (joystickError) {

                case Joystick::ERROR_MISSING:
                    debug("*** NO JOYSTICK DETECTED ***");
                    break;

                case Joystick::ERROR_PRODUCT:
                    debug("*** JOYSTICK NOT RECOGNIZED ***");
                    break;
				//没有错误: 更新状态:
                default:
					//
                    _hackflight.update();
                    // Input deltaT, quat, gyro; output motor values
                    _board.getMotors(time, state.quaternion, state.angularVel, motorvals, 4);
            }
         }

}; // HackflightFlightManager
