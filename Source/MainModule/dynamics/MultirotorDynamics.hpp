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


//管理: 飞行器力学:
class MultirotorDynamics {

public:

	/**
	 * Position map for state vector
	 */
	//专业名词:
		//Pitch 升降 绕X轴  对应常见Φ(phi)角：
		//Yaw 偏航 绕Y轴 对应常见θ(theta)角：
		//Roll 翻滚 绕Z轴 对应常见φ(psi)角: 
	//_DOT:  表示加速度
	enum {
		STATE_X,
		STATE_X_DOT,
		STATE_Y,
		STATE_Y_DOT,
		STATE_Z,
		STATE_Z_DOT,
		STATE_PHI,	//pitch
		STATE_PHI_DOT,  //pitch旋转的加速度!
		STATE_THETA,
		STATE_THETA_DOT,
		STATE_PSI,
		STATE_PSI_DOT
	};

	/**
	 * Class for parameters from the table below Equation 3
	 * 无人机的参数: 用于下面的 方程式3
	 * 每台无人机参数不同:    物理特性就不同
	 */
	class Parameters {

		friend class MultirotorDynamics;

	public:

		double b;  // 推力因子
		double d;  // 阻力系数
		double m;  // m (kg)  重量
		double l;  // l (meters) 轴距
		//身体惯性Ix,y,z
		double Ix; // Ix    X轴的转动惯量
		double Iy; // Iy    Y轴的转动惯量 
		double Iz; // Iz    Z轴转动惯量
		double Jr; // 转子惯量
		uint16_t maxrpm;//最大转速:(RPM,每分钟转速)


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

	// Dynamics:  动力学
	typedef struct {
		
		double angularVel[3];//角速度
		double bodyAccel[3];//加速度
		double inertialVel[3];//惯性速度
		double quaternion[4];//四元数
		//位置旋转
		pose_t pose;

	} state_t;

private:

	// Data structure for returning state
	//状态返回结构:
	state_t _state = {};

	// Flag for whether we're airborne and can update dynamics
	//是否是: 空降 (可以动态设置)
	bool _airborne = false;

	// Inertial-frame acceleration
	//帧间隔的(惯性):  加速度
	double _inertialAccel[3] = {};

	// y = Ax + b helper for frame-of-reference conversion methods
	//点乘实现:  y[3] =  A[3][3] * x[3]; 
	//矩阵装换:  A[][]是转换矩阵 ==>X[] *转换矩阵==>转换坐标系
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
	//根据bodyZ 和   rotation[3]:  计算惯性 inertial[3]
	//和bodyToInertial(相同) ==> 不过只转换了bodyZ的推力!! ==> 转到世界坐标的3个轴
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
		//这是最右列:  的装置矩阵
		//意思是:  只包Z轴的旋转yaw ==> 转换到 inertial坐标系
		double R[3] = { sph * sps + cph * cps * sth, cph * sps * sth - cps * sph, cph * cth };//只取了最后一列: R[3][0/1/2]
		//double R[3][3] = { {cps * cth,                cth * sps,  -sth},
		//{cps * sph * sth - cph * sps,  cph * cps + sph * sps * sth,  cth * sph},
		//{sph * sps + cph * cps * sth,  cph * sps * sth - cps * sph,  cph * cth} };

		//计算惯性:  inertial[3]
		for (uint8_t i = 0; i < 3; ++i) {
			inertial[i] = bodyZ * R[i];
		}
	}

	// Height above ground, set by kinematics
	//离地面的高度: 
	double _agl = 0;

protected:

//===============================> 理解这一块定义的所有数据:  很重要, 方便理解后面的函数
	// universal constants
	//g=9.8 (物理)
	static constexpr double g = 9.80665; // might want to allow this to vary!

	// state vector (see Eqn. 11) and its first temporal derivative
	//看公式11: pdf书上的公式:  
		////记录了: xyz位移/xyz位移加速度 / xyz角度/xyz角加速度 ====> 去除了delta的影响!! 1s为单位
	double _x[12] = {};
	//记录了: xyz位移/xyz位移加速度 / xyz角度/xyz角加速度
	double _dxdt[12] = {};

	// Values computed in Equation 6
	double _U1 = 0;     // total thrust     总推力  (应该是: 上升的推力)
	double _U2 = 0;     // roll thrust right  roll推力 (向右)
	double _U3 = 0;     // pitch thrust forward  pitch推力 (向前)
	double _U4 = 0;     // yaw thrust clockwise  yaw推力
	double _Omega = 0;  // torque clockwise  力矩(顺时针-旋转)

	// parameter block
	Parameters* _p = NULL;
 //roll-pitch-yaw:  (这3个函数, 在子类有实现)
		////u2/u3/u4: 子类中实现==> 返回roll/pitch/yaw的力???
	// roll right
	virtual double u2(double* o) = 0;
	// pitch forward
	virtual double u3(double* o) = 0;
	// yaw cw
	virtual double u4(double* o) = 0;

	// radians per second for each motor, and their squared values
	//旋翼转速:  每秒多少度:   ==>数组, 4个旋翼
	double* _omegas = NULL;
	//旋翼转速:  每秒多少度的平方  ==> 数组
	double* _omegas2 = NULL;

	// quad, hexa, octo, etc.:  扇叶数量, 4/6/8等 ==> 无人机一般为4
	uint8_t _motorCount = 0;


	/**
	 *  Constructor
	 * 构造函数:   初始化 _p = params  和 旋翼数量motorCount参数
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

	//更新平衡环:
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

	//描述:  把_dxdt[0]到_dxdt[11]填满合适的值
			//accelNED: 惯性加速度:
			//p->xxx: 飞机的各项参数 (质量-阻尼等)
	virtual void computeStateDerivative(double accelNED[3], double netz)
	{
		double phidot = _x[STATE_PHI_DOT];//pitch的加速度
		double thedot = _x[STATE_THETA_DOT];//yaw的加速度
		double psidot = _x[STATE_PSI_DOT];//roll的加速度

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
	//根据扇叶的转速[0,1]==>: 计算旋翼的转速
	virtual double computeMotorSpeed(double motorval)
	{
		//转速*pi/30
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
	//初始化: 旋转 和  是否在空中开始(默认false)
	void init(double rotation[3], bool airborne = false)
	{
		// Always start at location (0,0,0)
		//init位置: 在000
		_x[STATE_X] = 0;
		_x[STATE_Y] = 0;
		_x[STATE_Z] = 0;

		//init旋转
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
		//NED坐标系各轴的定义：
		//	N——北轴指向地球北；
		//	E——东轴指向地球东；
		//	D——地轴垂直于地球表面并指向下。
		//初始化参数: _inertialAccel  (即: 惯性加速度)
		bodyZToInertial(-g, rotation, _inertialAccel);

		// We usuall start on ground, but can start in air for testing
		_airborne = airborne;
	}

	/**
	 * Updates state.
	 *
	 * @param dt time in seconds since previous update
	 */
	//dt: 表示deltaTime
		//这一帧-上一帧时间:  即deltaTime
		//double dt = currentTime - _previousTime;
	//update:  更新飞行的状态:  ===> 计算更新  :  _state (飞行的状态位置/旋转/加速度等)
		//computeStateDerivative():求得: _dxdt[0]到_dxdt[11]值  ===>  从而初始化 _state所有成员值
	void update(double dt)
	{
		// Use the current Euler angles to rotate the orthogonal thrust vector into the inertial frame.
		// Negate to use NED.
		//欧拉角:  取roll-pitch-yaw  ==>构成矩阵
		double euler[3] = { _x[6], _x[8], _x[10] };
		
		//初始化accelNED:
		double accelNED[3] = {};
		//把Z轴的推力的加速度==>转换到惯性坐标系的3个轴上
			//参数1: up推力/质量,   euler: 矩阵(用于转到惯性坐标系), ===> 初始化到:  accelNED: (结果)
			//F= ma ==>a=F/m =>:  所以-_U1 / _p->m 是Z轴推力的加速度
		bodyZToInertial(-_U1 / _p->m, euler, accelNED);

		// We're airborne once net downward acceleration goes below zero
		//如果  向下的加速度: < 0  ==> 那么我们就是 airborne 状态
		//Z加速度: = g+ 推力的向下加速度  (世界坐标系)
		double netz = accelNED[2] + g;//即: 世界坐标系的Z轴==> accelNED[2]

		//z的加速度: (本地坐标系)
		double velz = _x[STATE_Z_DOT]; 

		//debugline("Airborne: %d   AGL: %3.2f   velz: %+3.2f   netz: %+3.2f", _airborne, _agl, velz, netz);

		// If we're airborne, check for low AGL on descent
		//在空中起飞的:   检测离地高度,在下降的时候
		if (_airborne) {

			//处理加速向下, 结束的一帧:
				//_agl <= 0:  即-1, 射线没有hit到 ,返回-1 ==>
				//netz >= 0:  表示, 没有加速向上飞行或稳定空中 :  设置_airborne = false;
			if (_agl <= 0 && netz >= 0) {

				_airborne = false;//只执行一次(帧)(加速接收的一刻,执行以下) ===>把飞机姿态变成000

				//位置和角度/加速度==> 初始化为0
				_x[STATE_PHI_DOT] = 0;
				_x[STATE_THETA_DOT] = 0;
				_x[STATE_PSI_DOT] = 0;
				_x[STATE_X_DOT] = 0;
				_x[STATE_Y_DOT] = 0;
				_x[STATE_Z_DOT] = 0;

				_x[STATE_PHI] = 0;
				_x[STATE_THETA] = 0;
				//离地高度
				_x[STATE_Z] += _agl;
			}
		}

		// If we're not airborne, we become airborne when downward acceleration has become negative ==> 当加速度向下时,  airborne = true
		//(是否是:  加速向上飞行): 
			//加速向上是: _airborne = true
			//稳定空中/向下: _airborne= false 			
		else {
			_airborne = netz < 0;
		}

		// Once airborne: (加速向上飞), we can update dynamics (就更新物理)
		//_airborne:  升空 (往上加速飞)
		//计算加速向上时, 的姿态:
		if (_airborne) {

			// Compute the state derivatives using Equation 12
			//根据公式12: 求导数
			// 根据netz和 accelNED ==> 
					//求得: _dxdt[0]到_dxdt[11]值 ===>即飞机的姿态==>(位置/速度/旋转等;)
			computeStateDerivative(accelNED, netz);

			// Compute state as first temporal integral of first temporal derivative
			//*deltatime的作用:  去除帧数影响==> 单位变成1s: 
				//(因为每帧都+=, *deltaTime==> 单位变成1s)
			for (uint8_t i = 0; i < 12; ++i) {
				_x[i] += dt * _dxdt[i];
			}

			// Once airborne, inertial-frame acceleration is same as NED acceleration
			//airborne=true: 当飞机加速向上时:    那么惯性坐标系 ==  NED坐标系 (加速度)
			_inertialAccel[0] = accelNED[0];
			_inertialAccel[1] = accelNED[1];
			_inertialAccel[2] = accelNED[2];
		}
		else {
			//"fly" to agl=0
			double vz = 5 * _agl;
			_x[STATE_Z] += vz * dt;
		}

		//更新平衡环相机的物理:  ==> 暂时不用实现
		updateGimbalDynamics(dt);

		// Get most values directly from state vector
		//从_x值, 初始化_state的值:
			//i的0/1/2==> 分别对应 x/y/z==>3个轴方向的值!
		for (uint8_t i = 0; i < 3; ++i) {
			uint8_t ii = 2 * i;//查看宏定义即可
			_state.angularVel[i] = _x[STATE_PHI_DOT + ii];
			_state.inertialVel[i] = _x[STATE_X_DOT + ii];
			_state.pose.rotation[i] = _x[STATE_PHI + ii];
			//位置X为例:   X = 
				//STATE_Y = STATE_X +2  
				//STATE_Z = STATE_Y +2 = STATE_X + 4
			_state.pose.location[i] = _x[STATE_X + ii];
		}

		// Convert inertial acceleration and velocity to body frame
		//加速度的:  坐标系转换
		inertialToBody(_inertialAccel, _state.pose.rotation, _state.bodyAccel);

		// Convert Euler angles to quaternion
		//欧拉角转为:  四元数  ==> 初始化为:   _state.quaternion
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
	//使用函数: u2/u3/u4
	//旋翼的推力计算:    仔细看下 ==> motorvals是每个旋翼的转速 ==> 计算出推力
	virtual void setMotors(double* motorvals, double dt)
	{
		// Convert the  motor values to radians per second
		//根据旋翼的转速:  算出==>每s, 多少度
		for (unsigned int i = 0; i < _motorCount; ++i) {
			//motorvals[i]:  是[0-1]的输入值==>影响(计算),当前转速
			_omegas[i] = computeMotorSpeed(motorvals[i]); //rad/s
		}

		// Compute overall torque from omegas before squaring
		//u4: 用于计算力矩旋转:  力矩(顺时针-旋转)
		_Omega = u4(_omegas);

		// Overall thrust is sum of squared omegas
		//omegas的平方:  计算, 上升推力
		_U1 = 0;
		for (unsigned int i = 0; i < _motorCount; ++i) {
			//平方:  每个旋翼转速的平方和
			_omegas2[i] = _omegas[i] * _omegas[i];
			//总推力(上升推力): 
			_U1 += _p->b * _omegas2[i];
		}

		// Use the squared Omegas to implement the rest of Eqn. 6
		//设置3个推力: roll推力(向右) +  pitch推力(向前  + yaw推力
			//u2/3/4:  见子类实现
		_U2 = _p->l * _p->b * u2(_omegas2);
		_U3 = _p->l * _p->b * u3(_omegas2);
		_U4 = _p->d * u4(_omegas2); 
	}

	/**
	 *  Gets current pose
	 *
	 *  @return data structure containing pose
	 */
	//用于设置载具的位置/旋转
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
	//设置距离地面的高度:  AGL
	void setAgl(double agl)
	{
		_agl = agl;
	}

	// Motor direction for animation
	virtual int8_t motorDirection(uint8_t i) { (void)i; return 0; }

	/**
	 *  Frame-of-reference conversion routines.  : 参考系转换
	 *
	 *  See Section 5 of http://www.chrobotics.com/library/understanding-euler-angles
	 */
	//坐标系转换: 飞行器坐标系, 转换到==>惯性坐标系
	//参数1: 转换前的结果  参数2: 转换矩阵   参数3: 转换后的结果
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
		
		//body * R(矩阵转换) =  转换到 惯性坐标系
		dot(R, body, inertial);
	}

	//坐标系的转换 : 惯性坐标系 inertial[3]     rotation[3]==>转换矩阵   转换为: 飞行器坐标系  body[3]
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

		//矩阵?? 3X3:   由rotation==>得到R
		double R[3][3] = { {cps * cth,                cth * sps,                   -sth},
			{cps * sph * sth - cph * sps,  cph * cps + sph * sps * sth,  cth * sph},
			{sph * sps + cph * cps * sth,  cph * sps * sth - cps * sph,  cph * cth} };

		//初始化boddy:
		dot(R, inertial, body);
	}

	/**
	 * Converts Euler angles to quaterion.
	 *
	 * @param eulerAngles input
	 * @param quaternion output
	 */
	//转换欧拉角为四元数:
	static void eulerToQuaternion(const double eulerAngles[3], double quaternion[4])
	{
		//FRotator::Quaternion() ue4有现成的函数

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
