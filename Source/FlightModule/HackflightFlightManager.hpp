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

        // Alt-hold:  
		//定高飞行pid:   保持一定高度飞行的pid
        hf::AltitudeHoldPid althold = hf::AltitudeHoldPid(
                10.00f, // altHoldPosP
                1.00f,  // altHoldVelP
                0.01f,  // altHoldVelI
                0.10f); // altHoldVelD


        // Pos-hold (via simulated optical flow) 
		//定位置飞行模式: pid
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

        // Mixer:   pid给出的输出, 经过mixer的机型矩阵配置
			//在_hackflight中有公式用到 , Mixer::RunArmed()中
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


		//pid控制器: 初始化 , ==> 状态1或0  ==> 见hackflight.hpp的runPidControllers()使用
			// Add altitude-hold and position-hold PID controllers in switch position 1
			//对pidController:  添加到数组, 并设置auxState 为1 ===> switch position为 1时更新 (aux按下时才更新)
			//高度控制 和  位置控制 的 pid控制器:
			_hackflight.addPidController(&althold, 1);
			_hackflight.addPidController(&flowhold, 1);

			// Add rate and level PID controllers for all aux switch positions
			//速度和角度  的  pid控制器
			//添加到数组, 但是, auxState 为0  ===> switch positions 任何时候更新 (不用管aux按下否)
			_hackflight.addPidController(&levelPid);
			_hackflight.addPidController(&ratePid);
		}

        virtual ~FHackflightFlightManager(void)
        {
        }

		//得到motor的转速, 并更新_hackflight的(角速度+姿态)
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
					//更新:  demands的值:
                    _hackflight.update();
                    // Input deltaT, quat, gyro;         output motor values
					//输入:  对_quat(姿态) 和 _gyro(角速度)初始化  ===> 来着与函数参数 ==> 来自于 dynamics->update()
					//输出:  同时返回:  motors
                    _board.getMotors(time, state.quaternion, state.angularVel, motorvals, 4);
            }
         }

}; // HackflightFlightManager
