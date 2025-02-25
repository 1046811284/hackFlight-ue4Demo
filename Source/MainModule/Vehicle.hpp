/*
 * General support for vehicles in MulticopterSim
 *
 * This class peforms the following functions:
 *
 * (1) Statically builds meshes, cameras, and other UE4 objects
 *
 * (2) Provides basic support for displaying vehicle kinematics
 *
 * Copyright (C) 2019 Simon D. Levy, Daniel Katzav
 *
 * MIT License
 */

#pragma once

#define WIN32_LEAN_AND_MEAN

#include "Utils.hpp"
#include "dynamics/MultirotorDynamics.hpp"
#include "FlightManager.hpp"
#include "Camera.hpp"
#include "Landscape.h"

#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

#include <stdio.h>

 // Windows/Linux compatibility 
#ifdef _WIN32
#define SPRINTF sprintf_s
#else
#include <wchar.h>
#define SPRINTF sprintf
#endif

// A macro for simplifying the declaration of static meshes
#define DECLARE_STATIC_MESH(structname, assetstr, objname)   \
    struct structname {                                             \
        ConstructorHelpers::FObjectFinderOptional<UStaticMesh> mesh;   \
        structname() : mesh(TEXT("/Game/Flying/Meshes/" assetstr)) { } \
    };                                                                     \
    static structname objname;


class Vehicle {

    private:

        ACameraActor* _groundCamera = NULL;

        // Useful approximation to infinity for tracing rays
		//无穷大的,追踪射线
        static constexpr float INF = 1e9;//1e9无穷大

        // Time during which velocity will be set to zero during final phase oflanding
		//最后阶段(着陆阶段), 把速度设置为0
        static constexpr float SETTLING_TIME = 1.0;

        // UE4 objects that must be built statically
		//ue4对象:  必须被静态构建  (飞机零部件)
        UStaticMesh* _frameMesh = NULL;
        UStaticMesh* _motorMesh = NULL;
        USoundCue* _soundCue = NULL;
        UAudioComponent* _audioComponent = NULL;
        USpringArmComponent* _gimbalSpringArm = NULL;
        USpringArmComponent * _playerCameraSpringArm = NULL;
        USpringArmComponent* _bodyHorizontalSpringArm = NULL;
        UCameraComponent* _playerCamera = NULL;

        // Support for switching from chase camera to FPV
        float _playerCameraFollowMeters = 0; //跟随相机的高度
        float _playerCameraElevationMeters = 0;//...相机的高度

        // PlayerController for getting keyboard events
        APlayerController * _playerController = NULL;

        // Cameras: 定义相机类,到pawn中
        Camera* _cameras[Camera::MAX_CAMERAS];
        uint8_t  _cameraCount;

        // Set in constructor
        MultirotorDynamics* _dynamics = NULL;

        // Threaded worker for running flight control
        class FFlightManager* _flightManager = NULL;

        // Have to seledct a map before flying
        bool _mapSelected = false;

        // Motor values for animation/sound
        float  _motorvals[FFlightManager::MAX_MOTORS] = {};

        // Circular buffer for moving average of motor values
		//循环缓冲区:  马达移动平局值
        TCircularBuffer<float>* _motorBuffer = NULL;
        uint32_t _bufferIndex = 0;

        // For computing AGL
        float _aglOffset = 0;

        // Countdown for zeroing-out velocity during final phase of landing
        float _settlingCountdown = 0;

        // Starting location, for kinematic offset
        FVector _startLocation = {};

        // Retrieves kinematics from dynamics computed in another thread, returning true if vehicle is airborne, false otherwise.
		//更新姿态:  通过_dynamics:  计算运动姿势
        void updateKinematics(void)
        {
            // Get vehicle pose from dynamics
			//获取姿态:  从_dynamics
            MultirotorDynamics::pose_t pose = _dynamics->getPose();
			//UKismetSystemLibrary::PrintString(nullptr, FString::Printf(TEXT("input %f, %f, %f"), pose.location[0], pose.location[1], pose.location[2]));

            // Set vehicle pose in animation
			//设置姿态:  位置 + 旋转
            _pawn->SetActorLocation(_startLocation +
                FVector(pose.location[0], pose.location[1], -pose.location[2]) * 100);  // NED => ENU
            _pawn->SetActorRotation(FMath::RadiansToDegrees(FRotator(pose.rotation[1], pose.rotation[2], pose.rotation[0])));
        }

		//螺旋桨动画 和 声音设置:
        void animatePropellers(void)
        {
            // Get motor values from dynamics
			//从_flightManager获取到==>螺旋桨的转速值:  ==> 写入_motorvals数组!
            _flightManager->getMotorValues(_motorvals);

            // Compute the sum of the motor values
			//求和:  所有螺旋桨 转速的和
            float motorsum = 0;
            for (uint8_t j = 0; j < _dynamics->motorCount(); ++j) {
                motorsum += _motorvals[j];
            }

            // Rotate props. For visual effect, we can ignore actual motor values, and just keep increasing the rotation.
			//更新每个旋翼的旋转角度:  (for 世界效果)  ==> 让旋翼看起来,在旋转
            if (motorsum > 0) {
				//参数1: 旋转方向 (顺时针/逆时针)   参数2: 旋翼数量
                rotateProps(_motorDirections, _dynamics->motorCount());
            }

            // Add mean to circular buffer for moving average
			//得到下一个索引:
            _bufferIndex = _motorBuffer->GetNextIndex(_bufferIndex);
			//添加值到:  _motorBuffer   ==> 值为平局转速:
            (*_motorBuffer)[_bufferIndex] = motorsum / _dynamics->motorCount();//总和/数量=>平局转速

            // Compute the mean motor value over the buffer frames
			//计算平均值: (平均转速)  
            float smoothedMotorMean = 0;
            for (uint8_t i = 0; i < _motorBuffer->Capacity(); ++i) {
				//20个平局值的总和:
                smoothedMotorMean += (*_motorBuffer)[i];
            }
			//总和(20个buffer的)/数量(20)==> 得到真正的平局值
            smoothedMotorMean /= _motorBuffer->Capacity();

            // Use the mean motor value to modulate the pitch and voume of the propeller sound
			//根据平局值==>来更新声音大小:
            _audioComponent->SetFloatParameter(FName("pitch"), smoothedMotorMean);
            _audioComponent->SetFloatParameter(FName("volume"), smoothedMotorMean);
        }

		//遍历获取相机的image:   ==> 可以替换为ue4的 RenderTarget处理否!??
        void grabImages(void)
        {
            for (uint8_t i = 0; i < _cameraCount; ++i) {
                _cameras[i]->grabImage();
            }
        }

		//创建相机和弹簧臂:  distanceMeters距离---elevationMeters高度
        void buildPlayerCameras(float distanceMeters, float elevationMeters)
        {
			//初始化弹簧臂:
            _bodyHorizontalSpringArm = _pawn->CreateDefaultSubobject<USpringArmComponent>(TEXT("BodyHorizontalSpringArm"));
            _bodyHorizontalSpringArm->SetupAttachment(_frameMeshComponent);
            _bodyHorizontalSpringArm->SetRelativeLocationAndRotation(FVector::ZeroVector, FRotator::ZeroRotator);
            _bodyHorizontalSpringArm->TargetArmLength = 0;
            _bodyHorizontalSpringArm->bEnableCameraLag = false;
            _bodyHorizontalSpringArm->bAbsoluteRotation = false;
            _bodyHorizontalSpringArm->bInheritPitch = false;
            _bodyHorizontalSpringArm->bInheritRoll = false;

            _playerCameraFollowMeters = distanceMeters;
            _playerCameraElevationMeters = elevationMeters;

            _playerCameraSpringArm = _pawn->CreateDefaultSubobject<USpringArmComponent>(TEXT("PlayerCameraSpringArm"));
            _playerCameraSpringArm->SetupAttachment(_bodyHorizontalSpringArm);

            _playerCameraSpringArm->bEnableCameraLag = false;
            _playerCameraSpringArm->bAbsoluteRotation = false;
            _playerCameraSpringArm->bInheritYaw = true;
            _playerCameraSpringArm->bInheritPitch = false;
            _playerCameraSpringArm->bInheritRoll = false;
            _playerCameraSpringArm->bEnableCameraRotationLag = true;

			//初始化相机:
            _playerCamera = _pawn->CreateDefaultSubobject<UCameraComponent>(TEXT("PlayerCamera"));
            _playerCamera->SetupAttachment(_playerCameraSpringArm, USpringArmComponent::SocketName);
        }

		//跟随相机模式:
        void playerCameraSetChaseView()
        {
            _playerController->SetViewTargetWithBlend(_pawn);
            _playerCameraSpringArm->SetRelativeLocationAndRotation(FVector(-_playerCameraFollowMeters, 0, _playerCameraElevationMeters)*100,
                    FRotator::ZeroRotator);;
            _playerCameraSpringArm->TargetArmLength = _playerCameraFollowMeters*100;

            _bodyHorizontalSpringArm->bInheritYaw = false ;
        }

		//前向相机模式:
        void playerCameraSetFrontView()
        {
            _playerController->SetViewTargetWithBlend(_pawn);
            _playerCameraSpringArm->SetRelativeLocationAndRotation(FVector::ZeroVector, FRotator::ZeroRotator);
            _playerCameraSpringArm->TargetArmLength = -30; // empircally determined to be far enough ahead of vehicle

            _bodyHorizontalSpringArm->bInheritYaw = true;
        }

		//地面相机模式:
        void playerCameraSetGroundView()
        {
            if (_groundCamera) _playerController->SetViewTargetWithBlend(_groundCamera);
        }

		//计算:  用XY计算角度==>
        float propStartAngle(float propX, float propY)
        {
            FVector vehicleCenter = _pawn->GetActorLocation();
            double theta = -atan2((propY - vehicleCenter.Y), (propX - vehicleCenter.X));
            return FMath::RadiansToDegrees(3.14159 / 2 - theta) + 57.5;
        }

		//更新每个旋翼的旋转角度
        void rotateProps(int8_t* motorDirections, uint8_t motorCount)
        {
			//静态变量:  每次调用转 motorDirections[i] * 200
            static float rotation;
            for (uint8_t i = 0; i < motorCount; ++i) {
				//i旋翼,   设置角度为  rotation * motorDirections[i] * 200
                setPropRotation(i, rotation * motorDirections[i] * 200);
            }
			//+1, 方便下次调用
            rotation++;
        }

    protected:

        APawn* _pawn = NULL;

        UStaticMeshComponent* _frameMeshComponent = NULL;

	    UStaticMeshComponent* _propellerMeshComponents[FFlightManager::MAX_MOTORS] = {};

        // Starts at zero and increases each time we add a propeller
        uint8_t _propCount;

        // Also set in constructor, but purely for visual effect
        int8_t _motorDirections[FFlightManager::MAX_MOTORS] = {};

    public:

		//_frameMeshComponent:  飞行器Mesh设置:
        void build(APawn* pawn, UStaticMesh* frameMesh)
        {
            _pawn = pawn;
            _frameMesh = frameMesh;

            _frameMeshComponent = _pawn->CreateDefaultSubobject<UStaticMeshComponent>(TEXT("FrameMesh"));
            _frameMeshComponent->SetStaticMesh(_frameMesh);
            _frameMeshComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Overlap);
            
            _pawn->SetRootComponent(_frameMeshComponent);

            _propCount = 0;
        }

		//飞行器上的:  机身Mehs + 声音 + 相机/弹簧臂
        void buildFull(APawn* pawn, UStaticMesh* frameMesh, float chaseCameraDistanceMeters, float chaseCameraElevationMeters)
        {
			//构建Mesh
            build(pawn, frameMesh);

            // Build the player-view cameras: 
			//相机/弹簧臂
            buildPlayerCameras(chaseCameraDistanceMeters, chaseCameraElevationMeters);

            // Get sound cue from Contents
	//声音组件:
            static ConstructorHelpers::FObjectFinder<USoundCue> soundCue(TEXT("/Game/Flying/Audio/MotorSoundCue"));

            // Store a reference to the Cue asset - we'll need it later.
            _soundCue = soundCue.Object;

            // Create an audio component, which wraps the sound cue, and allows us to ineract with it and its parameters from code
            _audioComponent = _pawn->CreateDefaultSubobject<UAudioComponent>(TEXT("PropellerAudioComp"));

            // Set the audio component's volume to zero
            _audioComponent->SetFloatParameter(FName("volume"), 0);

            // Attach the sound to the pawn's root, the sound follows the pawn around
            _audioComponent->SetupAttachment(_pawn->GetRootComponent());
    
	//平衡环相机:
            // Create a spring-arm for the gimbal
            _gimbalSpringArm = _pawn->CreateDefaultSubobject<USpringArmComponent>(TEXT("GimbalSpringArm"));
            _gimbalSpringArm->SetupAttachment(_pawn->GetRootComponent());
            _gimbalSpringArm->TargetArmLength = 0.f;
        }

		//添加其他Mesh:  子类中调用, 添加其他配件Mesh (螺旋桨等)
        void addMesh(UStaticMesh* mesh, const char* name, const FVector& location, const FRotator rotation, const FVector& scale)
        {
            UStaticMeshComponent* meshComponent =
                _pawn->CreateDefaultSubobject<UStaticMeshComponent>(FName(name));
            meshComponent->SetStaticMesh(mesh);
            meshComponent->SetupAttachment(_frameMeshComponent, USpringArmComponent::SocketName);
            meshComponent->AddRelativeLocation(location * 100); // m => cm
            meshComponent->AddLocalRotation(rotation);
            meshComponent->SetRelativeScale3D(scale);
        }

        void addMesh(UStaticMesh* mesh, const char* name, const FVector& location, const FRotator rotation)
        {
            addMesh(mesh, name, location, rotation, FVector(1, 1, 1));
        }

        void addMesh(UStaticMesh* mesh, const char* name)
        {
            addMesh(mesh, name, FVector(0, 0, 0), FRotator(0, 0, 0));
        }

        // z is set in editor
		//设置Mesh的: 相对位置/旋转
        UStaticMeshComponent * addProp(UStaticMesh* propMesh, float x, float y, float angle)
        {
            UStaticMeshComponent* propMeshComponent =
                _pawn->CreateDefaultSubobject<UStaticMeshComponent>(makeName("Prop", _propCount, "Mesh"));
            propMeshComponent->SetStaticMesh(propMesh);
            propMeshComponent->SetupAttachment(_frameMeshComponent, USpringArmComponent::SocketName);
            propMeshComponent->AddRelativeLocation(FVector(x, y, 0) * 100); // m => cm
            propMeshComponent->SetRelativeRotation(FRotator(0, angle, 0));
            _propellerMeshComponents[_propCount] = propMeshComponent;
            _propCount++;
            return propMeshComponent;
        }

		//设置:  位置/旋转
        void addProp(UStaticMesh* propMesh, float x, float y)
        {
            addProp(propMesh, x, y, propStartAngle(x,y));
        }

		//设置Mesh的: 相对旋转
        virtual void setPropRotation(uint8_t index, float angle)
        {
            _propellerMeshComponents[index]->SetRelativeRotation(FRotator(0, angle, 0));
        }

		//添加相机到弹簧臂:
        void addCamera(Camera* camera)
        {
            // Add camera to spring arm
            camera->addToVehicle(_pawn, _gimbalSpringArm, _cameraCount);

            // Increment the camera count for next time
            _cameras[_cameraCount++] = camera;
        }

        Vehicle(void)
        {
            _dynamics = NULL;
            _flightManager = NULL;
        }

        Vehicle(MultirotorDynamics* dynamics)
        {
            _dynamics = dynamics;

            for (uint8_t i = 0; i < dynamics->motorCount(); ++i) {
                _motorDirections[i] = dynamics->motorDirection(i);
            }

            _flightManager = NULL;
        }

        virtual ~Vehicle(void)
        {
        }

        void BeginPlay(FFlightManager* flightManager)
        {
            _flightManager = flightManager;

            // Player controller is useful for getting keyboard events, switching cameas, etc.
            _playerController = UGameplayStatics::GetPlayerController(_pawn->GetWorld(), 0);

            // Change view to player camera on start
            _playerController->SetViewTargetWithBlend(_pawn);

            // Make sure a map has been selected
			//是否选地图:
            _mapSelected = false;
            if (_pawn->GetWorld()->GetMapName().Contains("Untitled")) {
                error("NO MAP SELECTED");
                return;
            }
            _mapSelected = true;

            // Disable built-in physics
			//关闭物理:
            _frameMeshComponent->SetSimulatePhysics(false);

            // Start the audio for the propellers Note that because the
            // Cue Asset is set to loop the sound, once we start playing the sound, it
            // will play continiously...
			//播放:  这个声音cue本身是循环的 (cue内部,设置了音量参数)
            _audioComponent->Play();

            // Create circular queue for moving-average of motor values
			//循环队列TCircularBuffer:  可以放20个float元素
			//TCircularBuffer: 的特殊之处在于它内部存储数据的方式，内存空间不是动态增长的，而是循环使用的。
				//可以把 circular_buffer内部想象成一个首尾相连的环，当元素数量达到容器的容量上限时将自动重用最初的空间。
			//circular buffer可以使用assign库初始化
					//circular_buffer<int>cb = （list_of（1），2，3）；
					//
					//	print（cb）；//1，2，3，此时缓冲区已满
					//
					//	cb.push_back（4）；//4将覆盖最开始的1， ==> 指针,指向2
					//
					//	print（cb）；//2，3，4，begin（）从2开始  ==> 指针指向2
					//
					//	cb.push_back（5）；//5将覆盖最开始的2，
					//
					//	print（cb）；//3，4，5，begin（）从3开始
					//
					//	cb.pop_front（）；//弹出最开始的3
					//
					//	print（cb）；//4，5，现在circular_buffer只有两个元素
            _motorBuffer = new TCircularBuffer<float>(20);


            // Get vehicle ground-truth location for kinematic offset
			//获取初始飞行位置: 用于地面相机切换位置
            _startLocation = _pawn->GetActorLocation();

            // AGL offset will be set to a positve value the first time agl() is called
			//角度偏移
            _aglOffset = 0;

            // Get vehicle ground-truth rotation to initialize flight manager
			//起飞时,地面的初始旋转:
            FRotator startRotation = _pawn->GetActorRotation();

            // Initialize dynamics with initial rotation
            double rotation[3] = {
                FMath::DegreesToRadians(startRotation.Roll),
                FMath::DegreesToRadians(startRotation.Pitch),
                FMath::DegreesToRadians(startRotation.Yaw) };
			//初始化: _dynamics
            _dynamics->init(rotation);

            // Find the first cine camera in the viewport
            _groundCamera = NULL;
			//遍历pawn身上的: camera
            for (TActorIterator<ACameraActor> cameraItr(_pawn->GetWorld()); cameraItr; ++cameraItr) {

				//如果是CineCamera:
                ACameraActor * cameraActor = *cameraItr;
                if (cameraActor->GetName().StartsWith("CineCamera")) {
					//设置为地面camera
                    _groundCamera = cameraActor;
                }
            }

			//相机设置为: 追赶Chase模式
            playerCameraSetChaseView();
        }

        void Tick(float DeltaSeconds)
        {
            // Quit on ESCape key
			//按esc按键:  那么退出
            if (hitKey(EKeys::Escape)) GIsRequestingExit = true;

            // Run the game if a map has been selected
			//如果选择了地图:
            if (_mapSelected) {

                // Use 1/2 keys to switch player-camera view
				//检测按键输入: 切换视角
                setPlayerCameraView();

				//更新: 飞行姿态 ==> 通过_dynamics计算,物理学
                updateKinematics();

				//遍历获取相机的image:   ==> 可以替换为ue4的 RenderTarget处理否!??
                grabImages();

				//更新螺旋桨动画 和 旋转声音大小:
                animatePropellers();

				//_dynamics:  更新_dynamics的 AGL (离地高度)
                _dynamics->setAgl(agl());
            }
        }

		//更新相机模式3个:
        void setPlayerCameraView(void)
        {
            if (_groundCamera) {
                _groundCamera->SetActorRotation(
                        UKismetMathLibrary::FindLookAtRotation(_groundCamera->GetActorLocation(), _pawn->GetActorLocation()));
            }

            if (hitKey(EKeys::One)   || hitKey(EKeys::NumPadOne))   playerCameraSetFrontView();//向前视角
            if (hitKey(EKeys::Two)   || hitKey(EKeys::NumPadTwo))   playerCameraSetChaseView();//跟随视角
            if (hitKey(EKeys::Three) || hitKey(EKeys::NumPadThree)) playerCameraSetGroundView();//地面视角
        }

		//是否按下某个键
        bool hitKey(const FKey key)
        {
            return _playerController->IsInputKeyDown(key);
        }

        // Returns AGL when vehicle is level above ground, "infinity" otherwise
		//返回飞行器的:  当飞行器飞离地面时==>返回: AGL (地面高度(AGL))
			//(AGL: 超声波高度计, 可以获得离地面的高度)==> 传感器==> 这里直接用射线获取
        float agl(void)
        {
            // Start at the center of the vehicle
			//中心位置:  当前位置,为射线起始位置
            FVector startPoint = _pawn->GetActorLocation();
            startPoint.Z += 100;
            // End at a point an "infinite" distance below the start point: 
			//endpoint在startPoint无穷向下方:  向下发送射线
            FVector endPoint = FVector(startPoint.X, startPoint.Y, startPoint.Z - INF);

            //drawHorizontal(startPoint);
            //drawLine(startPoint, endPoint);

			//startPoint, endPoint 两个点之间产生射线:  hit到物体以后, 返回飞行器和hit点的距离
            float d = getImpactDistance(startPoint, endPoint);

            // The first time we measure, we need to set the offset
			//第一次(未起飞时)时我们需要:  测量偏移高度!!(如:  飞行器中心点, 到地面的offset) offset 
            if (_aglOffset == 0) {
                _aglOffset = d;
            }

			//返回:  hit距离 - 偏移高度
            return d - _aglOffset;
        }

        // Returns distance to mesh between points, or -1 if none found.
        // Eventually we may want to be able to specifiy an actor or actors to include or exclude
        // (other than the vehicle itself).
		//startPoint, endPoint 两个点之间产生射线:  hit到物体以后, 返回距离
        float getImpactDistance(FVector startPoint, FVector endPoint)
        {
            // Currently, the only collisions we ignore are with the pawn itself
            TArray<AActor*> actorsToIgnore;
			//不hit自己:
            actorsToIgnore.Add(_pawn);
            FCollisionQueryParams traceParams(FName(TEXT("Distance Trace")), true, actorsToIgnore[0]);
            traceParams.AddIgnoredActors(actorsToIgnore);

            FHitResult OutHit;
			//两个点之间, 发送射线:
            if (_pawn->GetWorld()->LineTraceSingleByChannel(OutHit, startPoint, endPoint, ECC_Visibility, traceParams)) {
				//如果射线hit到东西:
                if (OutHit.bBlockingHit) {
                    FVector impactPoint = OutHit.ImpactPoint;
					//返回开始点, 到hit点,之间的距离:
                    return (startPoint.Z - impactPoint.Z) / 100;
                }
            }
			//没有hit返回-1;
            return -1;
        }

		//画一条水平线:  drawLine
        void drawHorizontal(FVector point)
        {
            FVector lftPoint = FVector(point.X, point.Y - 100, point.Z);
            FVector rgtPoint = FVector(point.X, point.Y + 100, point.Z);
            drawLine(lftPoint, rgtPoint);
        }

        void drawLine(FVector point1, FVector point2)
        {
            DrawDebugLine(_pawn->GetWorld(), point1, point2, FColor::Green, false, .1, 0, 0.5);
        }

        void PostInitializeComponents()
        {
            // Add "Vehicle" tag for use by level blueprint
			//添加 "Vehicle" tag,  ==> 用于关卡蓝图:??
            _pawn->Tags.Add(FName("Vehicle"));

			//设置声音
            if (_soundCue->IsValidLowLevelFast()) {
                _audioComponent->SetSound(_soundCue);
            }
        }

		//设置平衡环的旋转
			//平衡环:  当飞行器旋转或者倾斜时:  相机保持水平不抖动
        void rotateGimbal(FQuat rotation)
        {
			//设置相机的旋转, 让其保持水平==>就达到了,平衡环效果
            _gimbalSpringArm->SetRelativeRotation(rotation);
        }


        UStaticMeshComponent* getFrameMesh(void)
        {
            return _frameMeshComponent;
        }

}; // class Vehicle
