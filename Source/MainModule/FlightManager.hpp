/*
 * Abstract, threaded flight-management class for MulticopterSim
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "dynamics/MultirotorDynamics.hpp"
#include "ThreadedManager.hpp"

class FFlightManager : public FThreadedManager {

    private:

        // Current motor values from PID controller
		//当前马达值:  pid   (马达: 即4个飞行轴马达)
        double * _motorvals = NULL; 
        
        // For computing deltaT:  用于计算DeltaTime
        double   _previousTime = 0;

        bool _running = false;

        /**
         * Flight-control method running repeatedly on its own thread.  
         * Override this method to implement your own flight controller.: 重写这个方法,实现飞行控制
         *
         * @param time current time in seconds (input):  输入, 当前时间
         * @param state vehicle state (input)  : 输入,当前状态
         * @param motorvals motor values returned by your controller (output): 输出马达的值
         *
         */
        virtual void getMotors(const double time, const MultirotorDynamics::state_t & state, double * motorvals)  = 0;
        
    protected:

        uint8_t _motorCount = 0;

        MultirotorDynamics * _dynamics = NULL;

        MultirotorDynamics::state_t _state = {};

        // Constructor, called main thread:  父类中,在构造函数 中创建了线程
		//MultirotorDynamics:  包含了飞行控制算法
        FFlightManager(MultirotorDynamics * dynamics) 
            : FThreadedManager()
        {
            // Allocate array for motor values
			//马达数量:
            _motorvals = new double[dynamics->motorCount()]();

            // Store dynamics for performTask()
            _dynamics = dynamics;

            // Constant: 马达数量
            _motorCount = dynamics->motorCount();

            // For periodic update
            _previousTime = 0;

            _running = true;
        }

        // Called repeatedly on worker thread to compute dynamics and run flight controller (PID)
		//用pid: 计算飞行控制
        void performTask(double currentTime)
        {
            if (!_running) return;

            // Compute time deltay in seconds
			//这一帧-上一帧时间:  即deltaTime
			double dt = currentTime - _previousTime;

            // Send current motor values and time delay to dynamics
			//发送当前发动机: 值,时间
            _dynamics->setMotors(_motorvals, dt);

            // Update dynamics
			//更新:  运动
            _dynamics->update(dt);

            // Get new vehicle state
            _state = _dynamics->getState();

            // PID controller: update the flight manager (e.g., HackflightManager) with
            // the dynamics state, getting back the motor values
            this->getMotors(currentTime, _state, _motorvals);

            // Track previous time for deltaT
            _previousTime = currentTime;
        }

        // Supports subclasses that might need direct access to dynamics state vector
        double * getVehicleStateVector(void)
        {
            return _dynamics->getStateVector();
        }

    public:

        static const uint8_t MAX_MOTORS = 16;

        ~FFlightManager(void)
        {
        }

        // Called by VehiclePawn::Tick() method to propeller animation/sound (motorvals)
        void getMotorValues(float * motorvals)
        {
            // Get motor values for propeller animation / motor sound
            for (uint8_t j=0; j<_motorCount; ++j) {
                motorvals[j] = _motorvals[j];
            }
        }

        void stop(void)
        {
            _running = false;
        }

}; // class FFlightManager
