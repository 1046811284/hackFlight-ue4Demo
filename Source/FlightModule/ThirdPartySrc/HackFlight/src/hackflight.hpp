/*
   Hackflight core algorithm

   Copyright (c) 2018 Simon D. Levy, Alec Singer

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

#include "sensor.hpp"
#include "board.hpp"
#include "mspparser.hpp"
#include "mixer.hpp"
#include "receiver.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "sensors/surfacemount/gyrometer.hpp"
#include "sensors/surfacemount/quaternion.hpp"
#include "sensors/mspsensor.hpp"

namespace hf {

    class Hackflight : public MspParser {

        private: 

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25.0f;

            // Passed to Hackflight::init() for a particular build
			//通过 Hackflight::init() 初始化这3个
            Board      * _board = NULL;
            Receiver   * _receiver = NULL;
            Mixer      * _mixer = NULL;

            // Supports periodic ad-hoc debugging
            Debugger _debugger;

            // PID controllers
            PidController * _pid_controllers[256] = {NULL};
            uint8_t _pid_controller_count = 0;

            // Mandatory sensors on the board
			//板子上的:  2个传感器
				//不是真正的传感器, 我们模拟一个(引擎可以直接获取)
            Gyrometer _gyrometer;//角速度
			//飞机四元数(姿态):
            Quaternion _quaternion; // not really a sensor, but we treat it like one!

            // Additional sensors 
			//其他传感器
            Sensor * _sensors[256] = {NULL};
            uint8_t _sensor_count = 0;

            // Vehicle state
            state_t _state;

            // Demands sent to mixer
            demands_t _demands;

            // Safety
            bool _safeToArm = false;
            bool _failsafe = false;

            // Support for headless mode
            float _yawInitial = 0;

			//判断是否是:  安全角度范围内
            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.rotation[axis]) < Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

			//对_quaternion进行设置:	
				//1) _quaternion.ready() 对_quaternion值进行更新
				//2)把_quaternion==>定义到  _state.rotation成员中:
            void checkQuaternion(void)
            {
                // Some quaternion filters may need to know the current time
				//当前时间:
                float time = _board->getTime();

                // If quaternion data ready
				//四元数是否已经初始化:
					//1) _quaternion.ready() 对_quaternion值进行更新
                if (_quaternion.ready(time)) {

                    // Adjust quaternion values based on IMU orientation
					//对_quaternion 变量更新:    基于  惯性测量单元测量  (SimBoard.hpp未实现)
                    _board->adjustQuaternion(_quaternion._w, _quaternion._x, _quaternion._y, _quaternion._z);

                    // Update state with new quaternion to yield Euler angles
					//2)把_quaternion==>定义到  _state.rotation成员中:
                    _quaternion.modifyState(_state, time);

                    // Adjust Euler angles to compensate for sloppy IMU mounting
					//调整 欧拉角,来  来弥补IMU安装调整   (SimBoard.hpp未实现)
                    _board->adjustRollAndPitch(_state.rotation[0], _state.rotation[1]);

                    // Synch serial comms to quaternion check
					//同步一系列命令到:  四元素检测  (SimBoard.hpp未实现)
                    doSerialComms();
                }
            }

			//对Gyrometer(角-->速度)进行设置: (姿态进行更新)
            void checkGyrometer(void)
            {
                // Some gyrometers may need to know the current time
                float time = _board->getTime();

                // If gyrometer data ready
				//_gyrometer.ready() :  对_gyrometer值进行更新
                if (_gyrometer.ready(time)) {

                    // Adjust gyrometer values based on IMU orientation 
					//从Board获取  gyrometer, ==>(SimBoard.hpp未实现)
                    _board->adjustGyrometer(_gyrometer._x, _gyrometer._y, _gyrometer._z);

                    // Update state with gyro rates
					//_gyrometer 写入到 :  _state.angularVel[]数组中
                    _gyrometer.modifyState(_state, time);

                    // For PID control, start with demands from receiver, scaling roll/pitch/yaw by constant
					//对输入进行缩放:
                    _demands.throttle = _receiver->demands.throttle;
                    _demands.roll     = _receiver->demands.roll  * _receiver->_demandScale;
                    _demands.pitch    = _receiver->demands.pitch * _receiver->_demandScale;
                    _demands.yaw      = _receiver->demands.yaw   * _receiver->_demandScale;


					// Sync PID controllers to gyro update
					//对所有pidController进行更新==> 从而修改 _demands的<部分>值
						//用pid, 对姿态进行更新:
                    runPidControllers();


                    // Use updated demands to run motors
					//通过设置motors的转速 ==> 达到更新demands角度的目的:
                    if (_state.armed && !_failsafe && !_receiver->throttleIsDown()) {
						//根据想要达到的角度demands * mixer ==> 得到, 想要的转速  ==> 设置mixer
                        _mixer->runArmed(_demands);
                    }
                }
            }

			//对所有pidController进行更新==> 从而修改 _demands的值
            void runPidControllers(void)
            {
                // Each PID controllers is associated with at least one auxiliary switch state
				//每一个PID控制器与至少一个辅助开关的状态有关
                uint8_t auxState = _receiver->getAux1State();//开关是否按下

                // Some PID controllers should cause LED to flash when they're active
				//一些pid 可以引起 LED闪烁
                bool shouldFlash = false;


				//遍历所有pid_Controller:
                for (uint8_t k=0; k<_pid_controller_count; ++k) {

                    PidController * pidController = _pid_controllers[k];

					//pid是否更新条件:
						//根据pid的auxState  和 aux状态,对pid进行更新
                    if (pidController->auxState <= auxState) {
						//那么: 用pid来更新   ==> demands中成员的值: (4个pid,4个定义!!)
							//_state: 当前状态
							//_demands: 返回修改自己
                        pidController->modifyDemands(_state, _demands); 

						//是否闪烁LED:   有些pidController子类, 返回的是true  ==> 如AltitudeHoldPid
                        if (pidController->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain PID controllers
				//刷新LED:  闪烁
                _board->flashLed(shouldFlash);
            }

            void checkReceiver(void)
            {

                // Sync failsafe to receiver
				//失效保护:
					//因为无线电波在传输过程中可能受到干扰或是数据丢失等等问题，当接收机无法接收到发射器的数据时，通常会进入保护状态，
						//也就是仍旧向无人机发送控制信号，此时的信号就是接收机收到遥控器发射器最后一次的有效数据。==>这样因为信号丢失而发送的保护数数据通常叫做failsafe数据。
				//如果丢失了遥控器型号,  同时起落架是打开状态:
                if (_receiver->lostSignal() && _state.armed) {
					//设置转速为0
                    _mixer->cutMotors();
					//收起起落架
                    _state.armed = false;
					//打开失效保护==> 设置为true:
                    _failsafe = true;
					//显示arm状态:
                    _board->showArmedStatus(false);
                    return;
                }


                // Check whether receiver data is available
				//检测接收的数据是否有效:  
					//并设置  _receiver->demands的值
                if (!_receiver->getDemands(_state.rotation[AXIS_YAW] - _yawInitial)) return;

                // Update PID controllers with receiver demands
				//用上面设置的: _receiver->demands , 进行PID更新
                for (uint8_t k=0; k<_pid_controller_count; ++k) {
					//更新所有PID controller:   使用参数_receiver->demands
                    _pid_controllers[k]->updateReceiver(_receiver->demands, _receiver->throttleIsDown());
                }

                // Disarm:  
					//只有Aux2按钮按下时: 才可以打开起落架!!
				//如起落架已经打开了, 并且 Aux2按钮没有按下  ==> 收起起落架
                if (_state.armed && !_receiver->getAux2State()) {
                    _state.armed = false;  //收起起落架
                } 

                // Avoid arming if aux2 switch down on startup
				//避免展开起落架:    如果当启动时, aux2开关向下按:
				//设置_safeToArm:  用于下面判定是否可以可以进行arm操作
                if (!_safeToArm) {
                    _safeToArm = !_receiver->getAux2State();
                }

                // Arm (after lots of safety checks!)
				//在大量的安全检查(一堆判断条件)之后:   进行Arm操作==> 打开起落架
                if (_safeToArm && !_state.armed && _receiver->throttleIsDown() && _receiver->getAux2State() && 
                        !_failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {
					//设置为true:   (开始打开起落架)
                    _state.armed = true;
					//为无头模式: 设置yaw  ==>
                    _yawInitial = _state.rotation[AXIS_YAW]; // grab yaw for headless mode
                }

                // Cut motors on throttle-down
				// _state.armed 为true  (起落架打开:)
					//且: throttle是否完全向下按:  (向下飞)  ===>   motror设置为0 (停止旋转)
                if (_state.armed && _receiver->throttleIsDown()) {
                    _mixer->cutMotors();
                }

                // Set LED based on arming status
				//LED, 显示arm状态
                _board->showArmedStatus(_state.armed);

            } // checkReceiver




			// (SimBoard.hpp未实现)
            void doSerialComms(void)
            {
				//(SimBoard.hpp未实现)
                while (_board->serialAvailableBytes() > 0) {

                    if (MspParser::parse(_board->serialReadByte())) {
                        _board->reboot(); // parser returns true when reboot requested
                    }
                }

				//(SimBoard.hpp未实现)
                while (MspParser::availableBytes() > 0) {
                    _board->serialWriteByte(MspParser::readByte());
                }

                // Support motor testing from GCS (门控开关)
                if (!_state.armed) {
                    _mixer->runDisarmed();
                }
            }

            void checkOptionalSensors(void)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    Sensor * sensor = _sensors[k];
                    float time = _board->getTime();
                    if (sensor->ready(time)) {
                        sensor->modifyState(_state, time);
                    }
                }
            }

			//加到传感器数组:
            void add_sensor(Sensor * sensor)
            {
                _sensors[_sensor_count++] = sensor;
            }

			//设置传感器
            void add_sensor(SurfaceMountSensor * sensor, Board * board) 
            {
				//加到传感器数组:
                add_sensor(sensor);
				//board :
                sensor->board = board;
            }

        protected:

            virtual void handle_STATE_Request(float & altitude, float & variometer, float & positionX, float & positionY, 
                    float & heading, float & velocityForward, float & velocityRightward) 
            {
                // XXX Use only heading for now
                altitude = 0;
                variometer = 0;
                positionX = 0;
                positionY = 0;
                heading = -_state.rotation[AXIS_YAW]; // NB: Angle negated for remote visualization
                velocityForward = 0;
                velocityRightward = 0;
            }
 
            virtual void handle_SET_ARMED(uint8_t  flag)
            {
                if (flag) {  // got		 command: arm only if throttle is down
                    if (_receiver->throttleIsDown()) {
                        _state.armed = true;
                    }
                }
                else {          // got disarming command: always disarm
                    _state.armed = false;
                }
            }

            virtual void handle_RC_NORMAL_Request(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6) override
            {
                c1 = _receiver->getRawval(0);
                c2 = _receiver->getRawval(1);
                c3 = _receiver->getRawval(2);
                c4 = _receiver->getRawval(3);
                c5 = _receiver->getRawval(4);
                c6 = _receiver->getRawval(5);
            }

            virtual void handle_ATTITUDE_RADIANS_Request(float & roll, float & pitch, float & yaw) override
            {
                roll  = _state.rotation[AXIS_ROLL];
                pitch = _state.rotation[AXIS_PITCH];
                yaw   = _state.rotation[AXIS_YAW];
            }

            virtual void handle_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4) override
            {
                _mixer->motorsDisarmed[0] = m1;
                _mixer->motorsDisarmed[1] = m2;
                _mixer->motorsDisarmed[2] = m3;
                _mixer->motorsDisarmed[3] = m4;
            }

        public:

			//初始化 board,  * receiver,  * mixer:
            void init(Board * board, Receiver * receiver, Mixer * mixer, bool armed=false)
            {  
                // Store the essentials
				//初始化值:
                _board    = board;
                _receiver = receiver;
                _mixer    = mixer;

                // Ad-hoc debugging support
                _debugger.init(board);

                // Support for mandatory sensors
				//吧board添加, 传感器成员
                add_sensor(&_quaternion, board);
                add_sensor(&_gyrometer, board);

                // Support adding new sensors and PID controllers
                _sensor_count = 0;

                // Initialize state
                memset(&_state, 0, sizeof(state_t));

                // Support safety override by simulator
                _state.armed = armed;

                // Initialize MPS parser for serial comms
                MspParser::init();

                // Initialize the receiver
                _receiver->begin();

                // Tell the mixer which board to use
                _mixer->board = board; 

                // Setup failsafe
                _failsafe = false;

            } // init

            void addSensor(Sensor * sensor) 
            {
                add_sensor(sensor);
            }

			//对pidController:  添加到数组, 并设置auxState
            void addPidController(PidController * pidController, uint8_t auxState=0) 
            {
				//设置auxState: 
                pidController->auxState = auxState;
				//添加到数组:
                _pid_controllers[_pid_controller_count++] = pidController;
            }

            void update(void)
            {
                // Grab control signal if available
				//检测是否有控制信号 - 以及起落架arm的控制
                checkReceiver();

                // Check mandatory sensors
                checkGyrometer();//陀螺测试仪==>角速度
                checkQuaternion();//四元数 ==> 姿态roll-pitch-yaw

                // Check optional sensors
				//其他附加传感器:
				  //运行其: sensor->modifyState(_state, time);
                checkOptionalSensors();//选项传感器?
            } 

    }; // class Hackflight

} // namespace
