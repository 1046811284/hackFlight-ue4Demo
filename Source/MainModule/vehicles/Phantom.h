/*
 Helper class for pawns using DJI Phantom frame
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "../MainModule/Vehicle.hpp"
#include "../MainModule/Camera.hpp"
#include "../MainModule/dynamics/QuadXAP.hpp"

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

// Structures to hold static mesh initializations
DECLARE_STATIC_MESH(FFrameStatics, "Phantom/Frame.Frame", FrameStatics)//找到资源 FrameStatics
DECLARE_STATIC_MESH(FPropStatics, "Phantom/Prop.Prop", PropStatics)//找到资源  PropStatics
 
//大疆精灵: Phantom 
class Phantom {

    private:

		//设置大疆精灵的物理参数:
        MultirotorDynamics::Parameters params = MultirotorDynamics::Parameters(

                // Estimated
                5.E-06, // b
                2.E-06, // d

                // https://www.dji.com/phantom-4/info
                1.380,  // m (kg)
                0.350,  // l (meters)

                // Estimated
                2,      // Ix
                2,      // Iy
                3,      // Iz
                38E-04, // Jr
                15000); // maxrpm

    public:
		//dynamics动力学计算:   传入 params
        QuadXAPDynamics dynamics = QuadXAPDynamics(&params);
		//飞行器: 传入 dynamics
        Vehicle vehicle = Vehicle(&dynamics);

    private:

        // Threaded worker for flight control
		//开一个线程: 用于飞行控制
        FFlightManager * _flightManager = NULL;

    public:
		//构建:  vehicle的各个Mesh
        void build(APawn * pawn)
        {
			//mesh1:
            vehicle.buildFull(pawn, FrameStatics.mesh.Get(), 1.5, 0.5);

            // Add propellers
			//添加组件:  4个旋翼  + 位置
            addProp(+1, +1);
            addProp(-1, -1);
            addProp(+1, -1);
            addProp(-1, +1);

            _flightManager = NULL;
        }

        void PostInitializeComponents()
        {
            vehicle.PostInitializeComponents();
        }

		//初始化:  FHackflightFlightManager !!!
        void BeginPlay(FFlightManager * flightManager)
        {
            _flightManager = flightManager;

            vehicle.BeginPlay(flightManager);
        }

        void EndPlay(void)
        {
            FThreadedManager::stopThread((FThreadedManager **)&_flightManager);
        }

        void Tick(float DeltaSeconds)
        {
            vehicle.Tick(DeltaSeconds);
        }

        void addCamera(Camera * camera)
        {
            vehicle.addCamera(camera);
        }

        void addProp(int8_t dx, int8_t dy)
        {
            vehicle.addProp(PropStatics.mesh.Get(), dx*0.12, dy*0.12);
        }

}; // class Phantom 
