/*
 * Threading support for MulticopterSim
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "Runnable.h"
#include "Utils.hpp"

class FThreadedManager : public FRunnable {

    private:

        FRunnableThread * _thread = NULL;

        bool _running = false;

        // Start-time offset so timing begins at zero
        double _startTime = 0;

        // For FPS reporting
        uint32_t _count;

    protected:

        // Implemented differently by each subclass
		//实现当前任务内容:
        virtual void performTask(double currentTime) = 0;

		//获取当前帧数:
        uint32_t getFps(void)
        {
            return (uint32_t)(_count/(FPlatformTime::Seconds()-_startTime));
        }

    public:

        FThreadedManager(void)
        {
            _thread = FRunnableThread::Create(this, TEXT("FThreadedManage"), 0, TPri_BelowNormal); 

            _startTime = FPlatformTime::Seconds();

            _count = 0;
        }


        ~FThreadedManager()
        {
            delete _thread;
        }

        uint32_t getCount(void)
        {
            return _count;
        }

        static void stopThread(FThreadedManager ** worker)
        {
            if (*worker) {
                (*worker)->Stop();
                delete *worker;
            }

            *worker = NULL;
        }

        // FRunnable interface.

        virtual bool Init() override
        {
            _running = false;

			return FRunnable::Init();
        }

		//运行:
        virtual uint32_t Run() override
        {
            // Initial wait before starting
            FPlatformProcess::Sleep(0.5);

            _running = true;

            while (_running) {

                // Get a high-fidelity current time value from the OS
                double currentTime = FPlatformTime::Seconds() - _startTime;

                // Pass current time to task implementation
                performTask(currentTime);

                // Increment count for FPS reporting
                _count++;
            }

			return 0;
        }

		//停止
        virtual void Stop() override
        {
            _running = false;

            // Final wait after stopping
            FPlatformProcess::Sleep(0.03);

			FRunnable::Stop();
        }

}; // class FThreadedManage
