
/*
 * Windows implementation of joystick/gamepad support for flight controllers
 *
 * Copyright (C) 2018 Simon D. Levy
 *
 * MIT License
 */

#ifdef _WIN32

#include "Joystick.h"

#define WIN32_LEAN_AND_MEAN

#undef TEXT
#include <shlwapi.h>
#include "joystickapi.h"

static void getAxes4(float axes[6], DWORD axis0, DWORD axis1, DWORD axis2, DWORD axis3)
{
	axes[0] = (float)axis0;
	axes[1] = (float)axis1;
	axes[2] = (float)axis2;
	axes[3] = (float)axis3;
}

static void getAxes5(float axes[6], uint8_t & naxes, DWORD axis0, DWORD axis1, DWORD axis2, DWORD axis3, DWORD axis4)
{
    naxes = 5;
	getAxes4(axes, axis0, axis1, axis2, axis3);
    axes[4] = (float)axis4;
}

#include <stdio.h>
#include "../../../../../../../Program Files/Epic Games/UE_4.23/Engine/Source/Runtime/Engine/Classes/Engine/Engine.h"
// XXX Should use a separate calibration program
static void adjustAxesInterlink(float * axes)
{
    axes[0] /= 0.575f;
    axes[1] /= 0.65f;
    axes[2] /= 0.58f;
    axes[3] /= 0.65f;
}

Joystick::Joystick(const char * devname)
{
    JOYCAPS joycaps = {};

    _productId = 0;

    _isGameController = false;

    // Grab the first available joystick
	//遍历:  直到 " == JOYERR_NOERROR"  ==> 即找到手柄型号!
    for (_joystickId=0; _joystickId<16; _joystickId++)
		//windowsAPI:  获取手柄设备的型号
        if (joyGetDevCaps(_joystickId, &joycaps, sizeof(joycaps)) == JOYERR_NOERROR)
            break;

	//找到:
    if (_joystickId < 16) {
		//设备id:
        _productId = joycaps.wPid;

        _isGameController = !(_productId==PRODUCT_TARANIS || _productId==PRODUCT_SPEKTRUM);
    }
}

// Convert InterLink aux switches to unique gamepad buttons
static void getAuxInterlink(float * axes, uint8_t buttons, uint8_t aux1, uint8_t aux2, float auxMid)
{
	axes[aux1] = -1;
	axes[aux2] = (buttons & 0x01) ? -1.f : +1.f;

	switch (buttons) {

	case 3:
	case 2:
		axes[aux1] = auxMid;
		break;

	case 19:
	case 18:
		axes[aux1] = 1;
	}
}

Joystick::error_t Joystick::pollProduct(float axes[6], uint8_t & buttons)
{
	//摇杆:
    JOYINFOEX joyState;
    joyState.dwSize=sizeof(joyState);
    joyState.dwFlags=JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
	//windows的API:   获取手柄状态joyState 和  _joystickId(设备id)
    joyGetPosEx(_joystickId, &joyState);

    // axes(轴): 0=Thr 1=Ael 2=Ele 3=Rud 4=Aux
    uint8_t naxes = 4;

	//没有获取到产品id: 报错, 没有检测到手柄
	if (_productId < 0)
	{
		return ERROR_MISSING;
	}

    switch (_productId) {

        case PRODUCT_SPEKTRUM:
            getAxes5(axes, naxes, joyState.dwYpos, joyState.dwZpos, joyState.dwVpos, joyState.dwXpos, joyState.dwUpos);
            break;

        case PRODUCT_TARANIS:
            getAxes5(axes, naxes, joyState.dwXpos, joyState.dwYpos, joyState.dwZpos, joyState.dwVpos, joyState.dwRpos);
            break;

        case PRODUCT_PS3_CLONE:      
        case PRODUCT_PS4:
            getAxes4(axes, joyState.dwYpos, joyState.dwZpos, joyState.dwRpos, joyState.dwXpos);
            break;

        case PRODUCT_F310:
            getAxes4(axes, joyState.dwYpos, joyState.dwZpos, joyState.dwRpos, joyState.dwXpos);
            break;

        case PRODUCT_XBOX360:  
        case PRODUCT_XBOX360_CLONE:
        case PRODUCT_XBOX360_CLONE2:
            getAxes4(axes, joyState.dwYpos, joyState.dwUpos, joyState.dwRpos, joyState.dwXpos);
            break;

        case PRODUCT_EXTREMEPRO3D:  
            getAxes4(axes, joyState.dwZpos, joyState.dwXpos, joyState.dwYpos, joyState.dwRpos);
            break;

        case PRODUCT_INTERLINK:
            getAxes4(axes, joyState.dwZpos, joyState.dwXpos, joyState.dwYpos, joyState.dwRpos);
			getAuxInterlink(axes, (uint8_t)joyState.dwButtons, AX_AU1, AX_AU2, AUX1_MID);
            break;

		//找不到这个产品id: (可能产品id发生了变化)===> 采取默认这个获取:
        default:
			//把4个状态:  joyState.dwYpos, joyState.dwUpos, joyState.dwRpos, joyState.dwXpos ==> 放到axes数组中
			getAxes4(axes, joyState.dwYpos, joyState.dwUpos, joyState.dwRpos, joyState.dwXpos);
            //return _productId ? ERROR_PRODUCT : ERROR_MISSING;
			//if (GEngine)
			//{
			//	GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Yellow, FString::Printf(TEXT("axes = %f, joyState.dwYpos = %d, \
			//		joyState.dwUpos = %d, joyState.dwRpos = %d, joyState.dwXpos = %d "), \
			//		axes, joyState.dwYpos, joyState.dwUpos, joyState.dwRpos, joyState.dwXpos));
			//}
			break;
    }

    // Normalize the axes to demands to [-1,+1]: 单位化到 [-1,+1]
    for (uint8_t k=0; k<naxes; ++k) {
        axes[k] = axes[k] / 32767 - 1;
    }

	//INTERLINK的手柄:
    if (_productId == PRODUCT_INTERLINK) {
        adjustAxesInterlink(axes);
    }

	//按钮状态:
    buttons = (uint8_t)joyState.dwButtons;

	return Joystick::ERROR_NOERROR;
}

#endif
