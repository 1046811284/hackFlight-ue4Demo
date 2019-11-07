/*
 * Header-only code for platform-independent multirotor dynamics
 *
 * Should work for any simulator, vehicle, or operating system
 *
 * Based on:
 *
 *   @inproceedings{DBLP:conf/icra/BouabdallahMS04,
 *     author    = {Samir Bouabdallah and Pierpaolo Murrieri and Roland Siegwart},
 *     title     = {Design and Control of an Indoor Micro Quadrotor},
 *     booktitle = {Proceedings of the 2004 {IEEE} International Conference on Robotics and
 *                 Automation, {ICRA} 2004, April 26 - May 1, 2004, New Orleans, LA, {USA}},
 *     pages     = {4393--4398},
 *     year      = {2004},
 *     crossref  = {DBLP:conf/icra/2004},
 *     url       = {https://doi.org/10.1109/ROBOT.2004.1302409},
 *     doi       = {10.1109/ROBOT.2004.1302409},
 *     timestamp = {Sun, 04 Jun 2017 01:00:00 +0200},
 *     biburl    = {https://dblp.org/rec/bib/conf/icra/BouabdallahMS04},
 *     bibsource = {dblp computer science bibliography, https://dblp.org}
 *   }
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../Utils.hpp"


//����: ��������ѧ:
class MultirotorDynamics {

public:

	/**
	 * Position map for state vector
	 */
	//רҵ����:
		//Pitch ���� ��X��  ��Ӧ������(phi)�ǣ�
		//Yaw ƫ�� ��Y�� ��Ӧ������(theta)�ǣ�
		//Roll ���� ��Z�� ��Ӧ������(psi)��: 
	//_DOT:  ��ʾ���ٶ�
	enum {
		STATE_X,
		STATE_X_DOT,
		STATE_Y,
		STATE_Y_DOT,
		STATE_Z,
		STATE_Z_DOT,
		STATE_PHI,	//pitch
		STATE_PHI_DOT,  //pitch��ת�ļ��ٶ�!
		STATE_THETA,
		STATE_THETA_DOT,
		STATE_PSI,
		STATE_PSI_DOT
	};

	/**
	 * Class for parameters from the table below Equation 3
	 * ���˻��Ĳ���: ��������� ����ʽ3
	 * ÿ̨���˻�������ͬ:    �������ԾͲ�ͬ
	 */
	class Parameters {

		friend class MultirotorDynamics;

	public:

		double b;  // ��������
		double d;  // ����ϵ��
		double m;  // m (kg)  ����
		double l;  // l (meters) ���
		//�������Ix,y,z
		double Ix; // Ix    X���ת������
		double Iy; // Iy    Y���ת������ 
		double Iz; // Iz    Z��ת������
		double Jr; // ת�ӹ���
		uint16_t maxrpm;//���ת��:(RPM,ÿ����ת��)


		Parameters(double b, double d, double m, double l, double Ix, double Iy, double Iz, double Jr, uint16_t maxrpm)
		{
			this->b = b;
			this->d = d;
			this->m = m;
			this->l = l;
			this->Ix = Ix;
			this->Iy = Iy;
			this->Iz = Iz;
			this->Jr = Jr;

			this->maxrpm = maxrpm;
		}
	};

	/**
	 * Exported state representations
	 */

	 // Kinematics
	typedef struct {

		double location[3];
		double rotation[3];

	} pose_t;

	// Dynamics:  ����ѧ
	typedef struct {
		
		double angularVel[3];//���ٶ�
		double bodyAccel[3];//���ٶ�
		double inertialVel[3];//�����ٶ�
		double quaternion[4];//��Ԫ��
		//λ����ת
		pose_t pose;

	} state_t;

private:

	// Data structure for returning state
	//״̬���ؽṹ:
	state_t _state = {};

	// Flag for whether we're airborne and can update dynamics
	//�Ƿ���: �ս� (���Զ�̬����)
	bool _airborne = false;

	// Inertial-frame acceleration
	//֡�����(����):  ���ٶ�
	double _inertialAccel[3] = {};

	// y = Ax + b helper for frame-of-reference conversion methods
	//���ʵ��:  y[3] =  A[3][3] * x[3]; 
	//����װ��:  A[][]��ת������ ==>X[] *ת������==>ת������ϵ
	static void dot(double A[3][3], double x[3], double y[3])
	{
		for (uint8_t j = 0; j < 3; ++j) {
			y[j] = 0;
			for (uint8_t k = 0; k < 3; ++k) {
				y[j] += A[j][k] * x[k];
			}
		}
	}

	// bodyToInertial method optimized for body X=Y=0
	//����bodyZ ��   rotation[3]:  ������� inertial[3]
	//��bodyToInertial(��ͬ) ==> ����ֻת����bodyZ������!! ==> ת�����������3����
	static void bodyZToInertial(double bodyZ, const double rotation[3], double inertial[3])
	{
		double phi = rotation[0]; //ptich
		double theta = rotation[1];//yaw
		double psi = rotation[2];//roll

		double cph = cos(phi);
		double sph = sin(phi);
		double cth = cos(theta);
		double sth = sin(theta);
		double cps = cos(psi);
		double sps = sin(psi);

		// This is the rightmost column of the body-to-inertial rotation matrix
		//����������:  ��װ�þ���
		//��˼��:  ֻ��Z�����תyaw ==> ת���� inertial����ϵ
		double R[3] = { sph * sps + cph * cps * sth, cph * sps * sth - cps * sph, cph * cth };//ֻȡ�����һ��: R[3][0/1/2]
		//double R[3][3] = { {cps * cth,                cth * sps,  -sth},
		//{cps * sph * sth - cph * sps,  cph * cps + sph * sps * sth,  cth * sph},
		//{sph * sps + cph * cps * sth,  cph * sps * sth - cps * sph,  cph * cth} };

		//�������:  inertial[3]
		for (uint8_t i = 0; i < 3; ++i) {
			inertial[i] = bodyZ * R[i];
		}
	}

	// Height above ground, set by kinematics
	//�����ĸ߶�: 
	double _agl = 0;

protected:

//===============================> �����һ�鶨�����������:  ����Ҫ, ����������ĺ���
	// universal constants
	//g=9.8 (����)
	static constexpr double g = 9.80665; // might want to allow this to vary!

	// state vector (see Eqn. 11) and its first temporal derivative
	//����ʽ11: pdf���ϵĹ�ʽ:  
		////��¼��: xyzλ��/xyzλ�Ƽ��ٶ� / xyz�Ƕ�/xyz�Ǽ��ٶ� ====> ȥ����delta��Ӱ��!! 1sΪ��λ
	double _x[12] = {};
	//��¼��: xyzλ��/xyzλ�Ƽ��ٶ� / xyz�Ƕ�/xyz�Ǽ��ٶ�
	double _dxdt[12] = {};

	// Values computed in Equation 6
	double _U1 = 0;     // total thrust     ������  (Ӧ����: ����������)
	double _U2 = 0;     // roll thrust right  roll���� (����)
	double _U3 = 0;     // pitch thrust forward  pitch���� (��ǰ)
	double _U4 = 0;     // yaw thrust clockwise  yaw����
	double _Omega = 0;  // torque clockwise  ����(˳ʱ��-��ת)

	// parameter block
	Parameters* _p = NULL;
 //roll-pitch-yaw:  (��3������, ��������ʵ��)
		////u2/u3/u4: ������ʵ��==> ����roll/pitch/yaw����???
	// roll right
	virtual double u2(double* o) = 0;
	// pitch forward
	virtual double u3(double* o) = 0;
	// yaw cw
	virtual double u4(double* o) = 0;

	// radians per second for each motor, and their squared values
	//����ת��:  ÿ����ٶ�:   ==>����, 4������
	double* _omegas = NULL;
	//����ת��:  ÿ����ٶȵ�ƽ��  ==> ����
	double* _omegas2 = NULL;

	// quad, hexa, octo, etc.:  ��Ҷ����, 4/6/8�� ==> ���˻�һ��Ϊ4
	uint8_t _motorCount = 0;


	/**
	 *  Constructor
	 * ���캯��:   ��ʼ�� _p = params  �� ��������motorCount����
	 */
	MultirotorDynamics(Parameters* params, const uint8_t motorCount)
	{
		_p = params;
		_motorCount = motorCount;

		_omegas = new double[motorCount]();
		_omegas2 = new double[motorCount]();

		for (uint8_t i = 0; i < 12; ++i) {
			_x[i] = 0;
		}
	}

	//����ƽ�⻷:
	virtual void updateGimbalDynamics(double dt) {}

	/**
	 * Implements Equation 12 computing temporal first derivative of state.
	 * Should fill _dxdx[0..11] with appropriate values.: 
	 * @param accelNED acceleration in NED inertial frame
	 * @param netz accelNED[2] with gravitational constant added in
	 * @param phidot rotational acceleration in roll axis  
	 * @param thedot rotational acceleration in pitch axis
	 * @param psidot rotational acceleration in yaw axis
	 */

	//����:  ��_dxdt[0]��_dxdt[11]�������ʵ�ֵ
			//accelNED: ���Լ��ٶ�:
			//p->xxx: �ɻ��ĸ������ (����-�����)
	virtual void computeStateDerivative(double accelNED[3], double netz)
	{
		double phidot = _x[STATE_PHI_DOT];//pitch�ļ��ٶ�
		double thedot = _x[STATE_THETA_DOT];//yaw�ļ��ٶ�
		double psidot = _x[STATE_PSI_DOT];//roll�ļ��ٶ�

		_dxdt[0] = _x[STATE_X_DOT];                                                              // x'
		_dxdt[1] = accelNED[0];                                                                  // x''
		_dxdt[2] = _x[STATE_Y_DOT];                                                              // y'
		_dxdt[3] = accelNED[1];                                                                  // y''
		_dxdt[4] = _x[STATE_Z_DOT];                                                              // z'
		_dxdt[5] = netz;                                                                         // z''
		_dxdt[6] = phidot;                                                                       // phi'
		_dxdt[7] = psidot * thedot * (_p->Iy - _p->Iz) / _p->Ix - _p->Jr / _p->Ix * thedot * _Omega + _U2 / _p->Ix;    // phi''
		_dxdt[8] = thedot;                                                                       // theta'
		_dxdt[9] = -(psidot * phidot * (_p->Iz - _p->Ix) / _p->Iy + _p->Jr / _p->Iy * phidot * _Omega + _U3 / _p->Iy); // theta''
		_dxdt[10] = psidot;                                                                        // psi'
		_dxdt[11] = thedot * phidot * (_p->Ix - _p->Iy) / _p->Iz + _U4 / _p->Iz;                               // psi''
	}

	/**
	 * Computes motor speed base on motor value
	 * @param motorval motor value in [0,1]
	 * @return motor speed in rad/s
	 */
	//������Ҷ��ת��[0,1]==>: ���������ת��
	virtual double computeMotorSpeed(double motorval)
	{
		//ת��*pi/30
		return motorval * _p->maxrpm * 3.14159 / 30;
	}

public:

	/**
	 *  Destructor
	 */
	virtual ~MultirotorDynamics(void)
	{
		delete _omegas;
		delete _omegas2;
	}

	/**
	 * Initializes kinematic pose, with flag for whether we're airbone (helps with testing gravity).
	 *
	 * @param rotation initial rotation
	 * @param airborne allows us to start on the ground (default) or in the air (e.g., gravity test)
	 */
	//��ʼ��: ��ת ��  �Ƿ��ڿ��п�ʼ(Ĭ��false)
	void init(double rotation[3], bool airborne = false)
	{
		// Always start at location (0,0,0)
		//initλ��: ��000
		_x[STATE_X] = 0;
		_x[STATE_Y] = 0;
		_x[STATE_Z] = 0;

		//init��ת
		_x[STATE_PHI] = rotation[0];
		_x[STATE_THETA] = rotation[1];
		_x[STATE_PSI] = rotation[2];

		// Initialize velocities and airborne flag
		_airborne = airborne;
		_x[STATE_X_DOT] = 0;
		_x[STATE_Y_DOT] = 0;
		_x[STATE_Z_DOT] = 0;
		_x[STATE_PHI_DOT] = 0;
		_x[STATE_THETA_DOT] = 0;
		_x[STATE_PSI_DOT] = 0;

		// Initialize inertial frame acceleration in NED coordinates
		//NED����ϵ����Ķ��壺
		//	N��������ָ����򱱣�
		//	E��������ָ����򶫣�
		//	D�������ᴹֱ�ڵ�����沢ָ���¡�
		//��ʼ������: _inertialAccel  (��: ���Լ��ٶ�)
		bodyZToInertial(-g, rotation, _inertialAccel);

		// We usuall start on ground, but can start in air for testing
		_airborne = airborne;
	}

	/**
	 * Updates state.
	 *
	 * @param dt time in seconds since previous update
	 */
	//dt: ��ʾdeltaTime
		//��һ֡-��һ֡ʱ��:  ��deltaTime
		//double dt = currentTime - _previousTime;
	//update:  ���·��е�״̬:  ===> �������  :  _state (���е�״̬λ��/��ת/���ٶȵ�)
		//computeStateDerivative():���: _dxdt[0]��_dxdt[11]ֵ  ===>  �Ӷ���ʼ�� _state���г�Աֵ
	void update(double dt)
	{
		// Use the current Euler angles to rotate the orthogonal thrust vector into the inertial frame.
		// Negate to use NED.
		//ŷ����:  ȡroll-pitch-yaw  ==>���ɾ���
		double euler[3] = { _x[6], _x[8], _x[10] };
		
		//��ʼ��accelNED:
		double accelNED[3] = {};
		//��Z��������ļ��ٶ�==>ת������������ϵ��3������
			//����1: up����/����,   euler: ����(����ת����������ϵ), ===> ��ʼ����:  accelNED: (���)
			//F= ma ==>a=F/m =>:  ����-_U1 / _p->m ��Z�������ļ��ٶ�
		bodyZToInertial(-_U1 / _p->m, euler, accelNED);

		// We're airborne once net downward acceleration goes below zero
		//���  ���µļ��ٶ�: < 0  ==> ��ô���Ǿ��� airborne ״̬
		//Z���ٶ�: = g+ ���������¼��ٶ�  (��������ϵ)
		double netz = accelNED[2] + g;//��: ��������ϵ��Z��==> accelNED[2]

		//z�ļ��ٶ�: (��������ϵ)
		double velz = _x[STATE_Z_DOT]; 

		//debugline("Airborne: %d   AGL: %3.2f   velz: %+3.2f   netz: %+3.2f", _airborne, _agl, velz, netz);

		// If we're airborne, check for low AGL on descent
		//�ڿ�����ɵ�:   �����ظ߶�,���½���ʱ��
		if (_airborne) {

			//�����������, ������һ֡:
				//_agl <= 0:  ��-1, ����û��hit�� ,����-1 ==>
				//netz >= 0:  ��ʾ, û�м������Ϸ��л��ȶ����� :  ����_airborne = false;
			if (_agl <= 0 && netz >= 0) {

				_airborne = false;//ִֻ��һ��(֡)(���ٽ��յ�һ��,ִ������) ===>�ѷɻ���̬���000

				//λ�úͽǶ�/���ٶ�==> ��ʼ��Ϊ0
				_x[STATE_PHI_DOT] = 0;
				_x[STATE_THETA_DOT] = 0;
				_x[STATE_PSI_DOT] = 0;
				_x[STATE_X_DOT] = 0;
				_x[STATE_Y_DOT] = 0;
				_x[STATE_Z_DOT] = 0;

				_x[STATE_PHI] = 0;
				_x[STATE_THETA] = 0;
				//��ظ߶�
				_x[STATE_Z] += _agl;
			}
		}

		// If we're not airborne, we become airborne when downward acceleration has become negative ==> �����ٶ�����ʱ,  airborne = true
		//(�Ƿ���:  �������Ϸ���): 
			//����������: _airborne = true
			//�ȶ�����/����: _airborne= false 			
		else {
			_airborne = netz < 0;
		}

		// Once airborne: (�������Ϸ�), we can update dynamics (�͸�������)
		//_airborne:  ���� (���ϼ��ٷ�)
		//�����������ʱ, ����̬:
		if (_airborne) {

			// Compute the state derivatives using Equation 12
			//���ݹ�ʽ12: ����
			// ����netz�� accelNED ==> 
					//���: _dxdt[0]��_dxdt[11]ֵ ===>���ɻ�����̬==>(λ��/�ٶ�/��ת��;)
			computeStateDerivative(accelNED, netz);

			// Compute state as first temporal integral of first temporal derivative
			//*deltatime������:  ȥ��֡��Ӱ��==> ��λ���1s: 
				//(��Ϊÿ֡��+=, *deltaTime==> ��λ���1s)
			for (uint8_t i = 0; i < 12; ++i) {
				_x[i] += dt * _dxdt[i];
			}

			// Once airborne, inertial-frame acceleration is same as NED acceleration
			//airborne=true: ���ɻ���������ʱ:    ��ô��������ϵ ==  NED����ϵ (���ٶ�)
			_inertialAccel[0] = accelNED[0];
			_inertialAccel[1] = accelNED[1];
			_inertialAccel[2] = accelNED[2];
		}
		else {
			//"fly" to agl=0
			double vz = 5 * _agl;
			_x[STATE_Z] += vz * dt;
		}

		//����ƽ�⻷���������:  ==> ��ʱ����ʵ��
		updateGimbalDynamics(dt);

		// Get most values directly from state vector
		//��_xֵ, ��ʼ��_state��ֵ:
			//i��0/1/2==> �ֱ��Ӧ x/y/z==>3���᷽���ֵ!
		for (uint8_t i = 0; i < 3; ++i) {
			uint8_t ii = 2 * i;//�鿴�궨�弴��
			_state.angularVel[i] = _x[STATE_PHI_DOT + ii];
			_state.inertialVel[i] = _x[STATE_X_DOT + ii];
			_state.pose.rotation[i] = _x[STATE_PHI + ii];
			//λ��XΪ��:   X = 
				//STATE_Y = STATE_X +2  
				//STATE_Z = STATE_Y +2 = STATE_X + 4
			_state.pose.location[i] = _x[STATE_X + ii];
		}

		// Convert inertial acceleration and velocity to body frame
		//���ٶȵ�:  ����ϵת��
		inertialToBody(_inertialAccel, _state.pose.rotation, _state.bodyAccel);

		// Convert Euler angles to quaternion
		//ŷ����תΪ:  ��Ԫ��  ==> ��ʼ��Ϊ:   _state.quaternion
		eulerToQuaternion(_state.pose.rotation, _state.quaternion);

	} // update

	/**
	 * Returns state structure.
	 * @return state structure
	 */
	state_t getState(void)
	{
		return _state;
	}

	/**
	 * Returns "raw" state vector.
	 * @return state vector
	 */
	double* getStateVector(void)
	{
		return _x;
	}

	/**
	 * Uses motor values to implement Equation 6.
	 *
	 * @param motorvals in interval [0,1]
	 * @param dt time constant in seconds
	 */
	//ʹ�ú���: u2/u3/u4
	//�������������:    ��ϸ���� ==> motorvals��ÿ�������ת�� ==> ���������
	virtual void setMotors(double* motorvals, double dt)
	{
		// Convert the  motor values to radians per second
		//���������ת��:  ���==>ÿs, ���ٶ�
		for (unsigned int i = 0; i < _motorCount; ++i) {
			//motorvals[i]:  ��[0-1]������ֵ==>Ӱ��(����),��ǰת��
			_omegas[i] = computeMotorSpeed(motorvals[i]); //rad/s
		}

		// Compute overall torque from omegas before squaring
		//u4: ���ڼ���������ת:  ����(˳ʱ��-��ת)
		_Omega = u4(_omegas);

		// Overall thrust is sum of squared omegas
		//omegas��ƽ��:  ����, ��������
		_U1 = 0;
		for (unsigned int i = 0; i < _motorCount; ++i) {
			//ƽ��:  ÿ������ת�ٵ�ƽ����
			_omegas2[i] = _omegas[i] * _omegas[i];
			//������(��������): 
			_U1 += _p->b * _omegas2[i];
		}

		// Use the squared Omegas to implement the rest of Eqn. 6
		//����3������: roll����(����) +  pitch����(��ǰ  + yaw����
			//u2/3/4:  ������ʵ��
		_U2 = _p->l * _p->b * u2(_omegas2);
		_U3 = _p->l * _p->b * u3(_omegas2);
		_U4 = _p->d * u4(_omegas2); 
	}

	/**
	 *  Gets current pose
	 *
	 *  @return data structure containing pose
	 */
	//���������ؾߵ�λ��/��ת
	pose_t getPose(void)
	{
		pose_t pose = {};

		for (uint8_t i = 0; i < 3; ++i) {
			uint8_t ii = 2 * i;
			pose.rotation[i] = _x[STATE_PHI + ii];
			pose.location[i] = _x[STATE_X + ii];
		}

		return pose;
	}

	/**
	 * Sets height above ground level (AGL).
	 * This method can be called by the kinematic visualization.
	 */
	//���þ������ĸ߶�:  AGL
	void setAgl(double agl)
	{
		_agl = agl;
	}

	// Motor direction for animation
	virtual int8_t motorDirection(uint8_t i) { (void)i; return 0; }

	/**
	 *  Frame-of-reference conversion routines.  : �ο�ϵת��
	 *
	 *  See Section 5 of http://www.chrobotics.com/library/understanding-euler-angles
	 */
	//����ϵת��: ����������ϵ, ת����==>��������ϵ
	//����1: ת��ǰ�Ľ��  ����2: ת������   ����3: ת����Ľ��
	static void bodyToInertial(double body[3], const double rotation[3], double inertial[3])
	{
		double phi = rotation[0];
		double theta = rotation[1];
		double psi = rotation[2];

		double cph = cos(phi);
		double sph = sin(phi);
		double cth = cos(theta);
		double sth = sin(theta);
		double cps = cos(psi);
		double sps = sin(psi);

		double R[3][3] = { {cps * cth,  cps * sph * sth - cph * sps,  sph * sps + cph * cps * sth},
			{cth * sps,  cph * cps + sph * sps * sth,  cph * sps * sth - cps * sph},
			{-sth,     cth * sph,                cph * cth} };
		
		//body * R(����ת��) =  ת���� ��������ϵ
		dot(R, body, inertial);
	}

	//����ϵ��ת�� : ��������ϵ inertial[3]     rotation[3]==>ת������   ת��Ϊ: ����������ϵ  body[3]
	static void inertialToBody(double inertial[3], const double rotation[3], double body[3])
	{
		double phi = rotation[0];
		double theta = rotation[1];
		double psi = rotation[2];

		double cph = cos(phi);
		double sph = sin(phi);
		double cth = cos(theta);
		double sth = sin(theta);
		double cps = cos(psi);
		double sps = sin(psi);

		//����?? 3X3:   ��rotation==>�õ�R
		double R[3][3] = { {cps * cth,                cth * sps,                   -sth},
			{cps * sph * sth - cph * sps,  cph * cps + sph * sps * sth,  cth * sph},
			{sph * sps + cph * cps * sth,  cph * sps * sth - cps * sph,  cph * cth} };

		//��ʼ��boddy:
		dot(R, inertial, body);
	}

	/**
	 * Converts Euler angles to quaterion.
	 *
	 * @param eulerAngles input
	 * @param quaternion output
	 */
	//ת��ŷ����Ϊ��Ԫ��:
	static void eulerToQuaternion(const double eulerAngles[3], double quaternion[4])
	{
		//FRotator::Quaternion() ue4���ֳɵĺ���

		// Convenient renaming
		double phi = eulerAngles[0] / 2;
		double the = eulerAngles[1] / 2;
		double psi = eulerAngles[2] / 2;

		// Pre-computation
		double cph = cos(phi);
		double cth = cos(the);
		double cps = cos(psi);
		double sph = sin(phi);
		double sth = sin(the);
		double sps = sin(psi);

		// Conversion
		quaternion[0] = cph * cth * cps + sph * sth * sps;
		quaternion[1] = cph * sth * sps - sph * cth * cps;
		quaternion[2] = -cph * sth * cps - sph * cth * sps;
		quaternion[3] = cph * cth * sps - sph * sth * cps;
	}

	/**
	 * Gets motor count set by constructor.
	 * @return motor count
	 */
	uint8_t motorCount(void)
	{
		return _motorCount;
	}

}; // class MultirotorDynamics
