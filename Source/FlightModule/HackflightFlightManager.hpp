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

		// Rate:  ����pid==>3��<���ٶ�>pid
		hf::RatePid ratePid = hf::RatePid(
			.01,	// Kp_roll_pitch
			.01,	// Ki_roll_pitch
			.01,	// Kd_roll_pitch
			.025,	// Kp_yaw 
			.01); 	// Ki_yaw

        // Level:  2����<�Ƕ�>��pid: roll��pitch
        hf::LevelPid levelPid = hf::LevelPid(0.8);

        // Alt-hold:  
		//���߷���pid:   ����һ���߶ȷ��е�pid
        hf::AltitudeHoldPid althold = hf::AltitudeHoldPid(
                10.00f, // altHoldPosP
                1.00f,  // altHoldVelP
                0.01f,  // altHoldVelI
                0.10f); // altHoldVelD


        // Pos-hold (via simulated optical flow) 
		//��λ�÷���ģʽ: pid
			//����ģ������ GPS �����£���ʵʱ���ɻ�ˮƽ�ƶ����룬ʵ�ֶ��������˻���ʱ����ȶ���ͣ
        hf::FlowHoldPid flowhold = hf::FlowHoldPid(0.05, 0.05);



   //==========================�������
        // Main firmware
        hf::Hackflight _hackflight;


        // Flight-controller board (board�������:)
			//1)��ȡ��Ԫ��
			//2)��ȡ �����ǵ�����
			//3)������� motor
			//4)��ȡ��ǰʱ��
        SimBoard _board;

        // "Receiver" (joystick/gamepad)
		//�ֱ�-ҡ������: Receive ==> ����update,��ȡ�ֱ�����
        SimReceiver _receiver;

        // Mixer:   pid���������, ����mixer�Ļ��;�������
			//��_hackflight���й�ʽ�õ� , Mixer::RunArmed()��
        hf::MixerQuadXAP _mixer;

        // "Sensors" (get values from dynamics)   ===> �鿴Դ��??
        SimSensors * _sensors = NULL;

    public:

        // Constructor
        FHackflightFlightManager(MultirotorDynamics * dynamics) 
            : FFlightManager(dynamics) 
        {
            // Start Hackflight firmware, indicating already armed
			//��ʼ��:  &_board, &_receiver, &_mixer  ==>��ʼ����3��
            _hackflight.init(&_board, &_receiver, &_mixer, true);

            // Add simulated sensor suite
            _sensors = new SimSensors(_dynamics);
            _hackflight.addSensor(_sensors);


		//pid������: ��ʼ�� , ==> ״̬1��0  ==> ��hackflight.hpp��runPidControllers()ʹ��
			// Add altitude-hold and position-hold PID controllers in switch position 1
			//��pidController:  ��ӵ�����, ������auxState Ϊ1 ===> switch positionΪ 1ʱ���� (aux����ʱ�Ÿ���)
			//�߶ȿ��� ��  λ�ÿ��� �� pid������:
			_hackflight.addPidController(&althold, 1);
			_hackflight.addPidController(&flowhold, 1);

			// Add rate and level PID controllers for all aux switch positions
			//�ٶȺͽǶ�  ��  pid������
			//��ӵ�����, ����, auxState Ϊ0  ===> switch positions �κ�ʱ����� (���ù�aux���·�)
			_hackflight.addPidController(&levelPid);
			_hackflight.addPidController(&ratePid);
		}

        virtual ~FHackflightFlightManager(void)
        {
        }

		//�õ�motor��ת��, ������_hackflight��(���ٶ�+��̬)
        virtual void getMotors(const double time, const MultirotorDynamics::state_t & state, double * motorvals) override
        {
			//update����:
				//��ʼ��:  _receiver.rawvals[] ��Ա
            Joystick::error_t joystickError = _receiver.update();

			//���������:
            switch (joystickError) {

                case Joystick::ERROR_MISSING:
                    debug("*** NO JOYSTICK DETECTED ***");
                    break;

                case Joystick::ERROR_PRODUCT:
                    debug("*** JOYSTICK NOT RECOGNIZED ***");
                    break;
				//û�д���: ����״̬:
                default:
					//����:  demands��ֵ:
                    _hackflight.update();
                    // Input deltaT, quat, gyro;         output motor values
					//����:  ��_quat(��̬) �� _gyro(���ٶ�)��ʼ��  ===> �����뺯������ ==> ������ dynamics->update()
					//���:  ͬʱ����:  motors
                    _board.getMotors(time, state.quaternion, state.angularVel, motorvals, 4);
            }
         }

}; // HackflightFlightManager
