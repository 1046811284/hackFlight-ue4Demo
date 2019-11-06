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
			//ͨ�� Hackflight::init() ��ʼ����3��
            Board      * _board = NULL;
            Receiver   * _receiver = NULL;
            Mixer      * _mixer = NULL;

            // Supports periodic ad-hoc debugging
            Debugger _debugger;

            // PID controllers
            PidController * _pid_controllers[256] = {NULL};
            uint8_t _pid_controller_count = 0;

            // Mandatory sensors on the board
			//�����ϵ�:  2��������
				//���������Ĵ�����, ����ģ��һ��(�������ֱ�ӻ�ȡ)
            Gyrometer _gyrometer;//���ٶ�
			//�ɻ���Ԫ��(��̬):
            Quaternion _quaternion; // not really a sensor, but we treat it like one!

            // Additional sensors 
			//����������
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

			//�ж��Ƿ���:  ��ȫ�Ƕȷ�Χ��
            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.rotation[axis]) < Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

			//��_quaternion��������:	
				//1) _quaternion.ready() ��_quaternionֵ���и���
				//2)��_quaternion==>���嵽  _state.rotation��Ա��:
            void checkQuaternion(void)
            {
                // Some quaternion filters may need to know the current time
				//��ǰʱ��:
                float time = _board->getTime();

                // If quaternion data ready
				//��Ԫ���Ƿ��Ѿ���ʼ��:
					//1) _quaternion.ready() ��_quaternionֵ���и���
                if (_quaternion.ready(time)) {

                    // Adjust quaternion values based on IMU orientation
					//��_quaternion ��������:    ����  ���Բ�����Ԫ����  (SimBoard.hppδʵ��)
                    _board->adjustQuaternion(_quaternion._w, _quaternion._x, _quaternion._y, _quaternion._z);

                    // Update state with new quaternion to yield Euler angles
					//2)��_quaternion==>���嵽  _state.rotation��Ա��:
                    _quaternion.modifyState(_state, time);

                    // Adjust Euler angles to compensate for sloppy IMU mounting
					//���� ŷ����,��  ���ֲ�IMU��װ����   (SimBoard.hppδʵ��)
                    _board->adjustRollAndPitch(_state.rotation[0], _state.rotation[1]);

                    // Synch serial comms to quaternion check
					//ͬ��һϵ�����:  ��Ԫ�ؼ��  (SimBoard.hppδʵ��)
                    doSerialComms();
                }
            }

			//��Gyrometer(��-->�ٶ�)��������: (��̬���и���)
            void checkGyrometer(void)
            {
                // Some gyrometers may need to know the current time
                float time = _board->getTime();

                // If gyrometer data ready
				//_gyrometer.ready() :  ��_gyrometerֵ���и���
                if (_gyrometer.ready(time)) {

                    // Adjust gyrometer values based on IMU orientation 
					//��Board��ȡ  gyrometer, ==>(SimBoard.hppδʵ��)
                    _board->adjustGyrometer(_gyrometer._x, _gyrometer._y, _gyrometer._z);

                    // Update state with gyro rates
					//_gyrometer д�뵽 :  _state.angularVel[]������
                    _gyrometer.modifyState(_state, time);

                    // For PID control, start with demands from receiver, scaling roll/pitch/yaw by constant
					//�������������:
                    _demands.throttle = _receiver->demands.throttle;
                    _demands.roll     = _receiver->demands.roll  * _receiver->_demandScale;
                    _demands.pitch    = _receiver->demands.pitch * _receiver->_demandScale;
                    _demands.yaw      = _receiver->demands.yaw   * _receiver->_demandScale;


					// Sync PID controllers to gyro update
					//������pidController���и���==> �Ӷ��޸� _demands��<����>ֵ
						//��pid, ����̬���и���:
                    runPidControllers();


                    // Use updated demands to run motors
					//ͨ������motors��ת�� ==> �ﵽ����demands�Ƕȵ�Ŀ��:
                    if (_state.armed && !_failsafe && !_receiver->throttleIsDown()) {
						//������Ҫ�ﵽ�ĽǶ�demands * mixer ==> �õ�, ��Ҫ��ת��  ==> ����mixer
                        _mixer->runArmed(_demands);
                    }
                }
            }

			//������pidController���и���==> �Ӷ��޸� _demands��ֵ
            void runPidControllers(void)
            {
                // Each PID controllers is associated with at least one auxiliary switch state
				//ÿһ��PID������������һ���������ص�״̬�й�
                uint8_t auxState = _receiver->getAux1State();//�����Ƿ���

                // Some PID controllers should cause LED to flash when they're active
				//һЩpid �������� LED��˸
                bool shouldFlash = false;


				//��������pid_Controller:
                for (uint8_t k=0; k<_pid_controller_count; ++k) {

                    PidController * pidController = _pid_controllers[k];

					//pid�Ƿ��������:
						//����pid��auxState  �� aux״̬,��pid���и���
                    if (pidController->auxState <= auxState) {
						//��ô: ��pid������   ==> demands�г�Ա��ֵ: (4��pid,4������!!)
							//_state: ��ǰ״̬
							//_demands: �����޸��Լ�
                        pidController->modifyDemands(_state, _demands); 

						//�Ƿ���˸LED:   ��ЩpidController����, ���ص���true  ==> ��AltitudeHoldPid
                        if (pidController->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain PID controllers
				//ˢ��LED:  ��˸
                _board->flashLed(shouldFlash);
            }

            void checkReceiver(void)
            {

                // Sync failsafe to receiver
				//ʧЧ����:
					//��Ϊ���ߵ粨�ڴ�������п����ܵ����Ż������ݶ�ʧ�ȵ����⣬�����ջ��޷����յ�������������ʱ��ͨ������뱣��״̬��
						//Ҳ�����Ծ������˻����Ϳ����źţ���ʱ���źž��ǽ��ջ��յ�ң�������������һ�ε���Ч���ݡ�==>������Ϊ�źŶ�ʧ�����͵ı���������ͨ������failsafe���ݡ�
				//�����ʧ��ң�����ͺ�,  ͬʱ������Ǵ�״̬:
                if (_receiver->lostSignal() && _state.armed) {
					//����ת��Ϊ0
                    _mixer->cutMotors();
					//���������
                    _state.armed = false;
					//��ʧЧ����==> ����Ϊtrue:
                    _failsafe = true;
					//��ʾarm״̬:
                    _board->showArmedStatus(false);
                    return;
                }


                // Check whether receiver data is available
				//�����յ������Ƿ���Ч:  
					//������  _receiver->demands��ֵ
                if (!_receiver->getDemands(_state.rotation[AXIS_YAW] - _yawInitial)) return;

                // Update PID controllers with receiver demands
				//���������õ�: _receiver->demands , ����PID����
                for (uint8_t k=0; k<_pid_controller_count; ++k) {
					//��������PID controller:   ʹ�ò���_receiver->demands
                    _pid_controllers[k]->updateReceiver(_receiver->demands, _receiver->throttleIsDown());
                }

                // Disarm:  
					//ֻ��Aux2��ť����ʱ: �ſ��Դ������!!
				//��������Ѿ�����, ���� Aux2��ťû�а���  ==> ���������
                if (_state.armed && !_receiver->getAux2State()) {
                    _state.armed = false;  //���������
                } 

                // Avoid arming if aux2 switch down on startup
				//����չ�������:    ���������ʱ, aux2�������°�:
				//����_safeToArm:  ���������ж��Ƿ���Կ��Խ���arm����
                if (!_safeToArm) {
                    _safeToArm = !_receiver->getAux2State();
                }

                // Arm (after lots of safety checks!)
				//�ڴ����İ�ȫ���(һ���ж�����)֮��:   ����Arm����==> �������
                if (_safeToArm && !_state.armed && _receiver->throttleIsDown() && _receiver->getAux2State() && 
                        !_failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {
					//����Ϊtrue:   (��ʼ�������)
                    _state.armed = true;
					//Ϊ��ͷģʽ: ����yaw  ==>
                    _yawInitial = _state.rotation[AXIS_YAW]; // grab yaw for headless mode
                }

                // Cut motors on throttle-down
				// _state.armed Ϊtrue  (����ܴ�:)
					//��: throttle�Ƿ���ȫ���°�:  (���·�)  ===>   motror����Ϊ0 (ֹͣ��ת)
                if (_state.armed && _receiver->throttleIsDown()) {
                    _mixer->cutMotors();
                }

                // Set LED based on arming status
				//LED, ��ʾarm״̬
                _board->showArmedStatus(_state.armed);

            } // checkReceiver




			// (SimBoard.hppδʵ��)
            void doSerialComms(void)
            {
				//(SimBoard.hppδʵ��)
                while (_board->serialAvailableBytes() > 0) {

                    if (MspParser::parse(_board->serialReadByte())) {
                        _board->reboot(); // parser returns true when reboot requested
                    }
                }

				//(SimBoard.hppδʵ��)
                while (MspParser::availableBytes() > 0) {
                    _board->serialWriteByte(MspParser::readByte());
                }

                // Support motor testing from GCS (�ſؿ���)
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

			//�ӵ�����������:
            void add_sensor(Sensor * sensor)
            {
                _sensors[_sensor_count++] = sensor;
            }

			//���ô�����
            void add_sensor(SurfaceMountSensor * sensor, Board * board) 
            {
				//�ӵ�����������:
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

			//��ʼ�� board,  * receiver,  * mixer:
            void init(Board * board, Receiver * receiver, Mixer * mixer, bool armed=false)
            {  
                // Store the essentials
				//��ʼ��ֵ:
                _board    = board;
                _receiver = receiver;
                _mixer    = mixer;

                // Ad-hoc debugging support
                _debugger.init(board);

                // Support for mandatory sensors
				//��board���, ��������Ա
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

			//��pidController:  ��ӵ�����, ������auxState
            void addPidController(PidController * pidController, uint8_t auxState=0) 
            {
				//����auxState: 
                pidController->auxState = auxState;
				//��ӵ�����:
                _pid_controllers[_pid_controller_count++] = pidController;
            }

            void update(void)
            {
                // Grab control signal if available
				//����Ƿ��п����ź� - �Լ������arm�Ŀ���
                checkReceiver();

                // Check mandatory sensors
                checkGyrometer();//���ݲ�����==>���ٶ�
                checkQuaternion();//��Ԫ�� ==> ��̬roll-pitch-yaw

                // Check optional sensors
				//�������Ӵ�����:
				  //������: sensor->modifyState(_state, time);
                checkOptionalSensors();//ѡ�����?
            } 

    }; // class Hackflight

} // namespace
