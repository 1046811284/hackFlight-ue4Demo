/* Helper class for pawns using DJI TinyWhoop frame 
 *
 * Copyright (C) 2019 Simon D. Levy 
 *
 * MIT License 
 */ 

#pragma once 

#include "../MainModule/Vehicle.hpp" 
#include "../MainModule/dynamics/QuadXAP.hpp"

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

// Structures to hold static mesh initializations
DECLARE_STATIC_MESH(FFrameStatics, "TinyWhoop/Frame.Frame", FrameStatics)
DECLARE_STATIC_MESH(FPropCWStatics, "TinyWhoop/PropCW.PropCW", PropCWStatics)
DECLARE_STATIC_MESH(FPropCCWStatics, "TinyWhoop/PropCCW.PropCCW", PropCCWStatics)
DECLARE_STATIC_MESH(FMotor1Statics, "TinyWhoop/Motor1.Motor1", Motor1Statics)
DECLARE_STATIC_MESH(FMotor2Statics, "TinyWhoop/Motor2.Motor2", Motor2Statics)
DECLARE_STATIC_MESH(FMotor3Statics, "TinyWhoop/Motor3.Motor3", Motor3Statics)
DECLARE_STATIC_MESH(FMotor4Statics, "TinyWhoop/Motor4.Motor4", Motor4Statics)
DECLARE_STATIC_MESH(FBatteryStatics,"TinyWhoop/Battery.Battery", BatteryStatics)
DECLARE_STATIC_MESH(FCameraMountStatics,  "TinyWhoop/CameraMount.CameraMount", CameraMountStatics)
DECLARE_STATIC_MESH(FCameraStatics,  "TinyWhoop/Camera.Camera", CameraStatics)
DECLARE_STATIC_MESH(FWhoopFCStatics, "TinyWhoop/WhoopFC.WhoopFC", WhoopFCStatics)
DECLARE_STATIC_MESH(FScrew1Statics,  "TinyWhoop/Screw1.Screw1", Screw1Statics)
DECLARE_STATIC_MESH(FScrew2Statics,  "TinyWhoop/Screw2.Screw2", Screw2Statics)
DECLARE_STATIC_MESH(FScrew3Statics,  "TinyWhoop/Screw3.Screw3", Screw3Statics)
DECLARE_STATIC_MESH(FScrew4Statics,  "TinyWhoop/Screw4.Screw4", Screw4Statics)

class TinyWhoop {

    private:

		//使用的是:  大疆phantom-4型号的:参数
        MultirotorDynamics::Parameters params = MultirotorDynamics::Parameters(

                // Estimated
                5.E-06, // b
                2.E-06, // d

                // https://www.dji.com/phantom-4/info
                1.380,  // m (kg)  重量
                0.350,  // l (meters) 轴距

			//转动惯量，又称惯性距（俗称惯性力矩，易与力矩混淆），通常以Ix、Iy、Iz表示，单位为 kg * m ^ 2，
			//可说是一个物体对于旋转运动的惯性。
			//对于一个质点，I = mr ^ 2，其中 m 是其质量，r 是质点和转轴的垂直距离。
                // Estimated
				//应用垂直轴定理, Ix＋Iy＋Iz＝2*(m×R ^ 2)
                2,      // Ix    X轴的转动惯量
                2,      // Iy    Y轴的转动惯量 
                3,      // Iz    Z轴转动惯量
                38E-04, // Jr  
                15000); // maxrpm  转速

    public:

        QuadXAPDynamics dynamics = QuadXAPDynamics(&params);

        Vehicle vehicle = Vehicle(&dynamics);

    private:

        // Threaded worker for flight control
        FFlightManager * _flightManager = NULL;

        // Adds simulated motor barrel to frame
        void addMotor(UStaticMesh * motorMesh, uint8_t id)
        {
            char meshName[10];
            SPRINTF(meshName, "Motor%d", id);
            vehicle.addMesh(motorMesh, meshName);
        }

    public:


		//构建mesh:
        void build(APawn * pawn)
        {
            // Build the frame: 
			//构建:  相机-弹簧臂-Mesh  3部分
            vehicle.buildFull(pawn, FrameStatics.mesh.Get(), 1.5, 0.50);

            // Add propellers:  4个旋翼 + 位置
            float x13 = -.0470, x24 = +.0430, y14 = -.020, y23 = +.070;
            vehicle.addProp(PropCCWStatics.mesh.Get(), x13, y14);
            vehicle.addProp(PropCCWStatics.mesh.Get(), x24, y23);
            vehicle.addProp(PropCCWStatics.mesh.Get(), x13, y23);
            vehicle.addProp(PropCCWStatics.mesh.Get(), x24, y14);

            // Add motor barrels :  机桶
            addMotor(Motor1Statics.mesh.Get(), 1);
            addMotor(Motor2Statics.mesh.Get(), 2);
            addMotor(Motor3Statics.mesh.Get(), 3);
            addMotor(Motor4Statics.mesh.Get(), 4);

            // Add battery, camera, etc. //电池-相机等 mesh
            vehicle.addMesh(BatteryStatics.mesh.Get(), "BatteryMesh");
            vehicle.addMesh(CameraMountStatics.mesh.Get(), "CameraMountMesh");
            vehicle.addMesh(CameraStatics.mesh.Get(), "CameraMesh");
            vehicle.addMesh(WhoopFCStatics.mesh.Get(), "WhoopFCMesh");
            vehicle.addMesh(Screw1Statics.mesh.Get(), "Screw1Mesh");
            vehicle.addMesh(Screw2Statics.mesh.Get(), "Screw2Mesh");
            vehicle.addMesh(Screw3Statics.mesh.Get(), "Screw3Mesh");
            vehicle.addMesh(Screw4Statics.mesh.Get(), "Screw4Mesh");

            // Flight manager will be set in BeginPlay()
            _flightManager = NULL;
        }

        void PostInitializeComponents()
        {
            vehicle.PostInitializeComponents();
        }

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


}; // class TinyWhoop 
