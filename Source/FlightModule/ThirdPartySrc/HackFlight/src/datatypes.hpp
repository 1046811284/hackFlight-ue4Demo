/*
   Datatype declarations

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#pragma once

namespace hf {

    enum {
        AXIS_ROLL = 0,
        AXIS_PITCH, 
        AXIS_YAW
    };

    typedef struct {

        float throttle;
        float roll;
        float pitch;
        float yaw;

    } demands_t;

    typedef struct {

        bool  armed;//�����??

        float location[3]; //λ��
        float rotation[3]; //��ת
        float angularVel[3]; //���ٶ�
        float bodyAccel[3];  //��������ϵ-���ٶ�
        float bodyVel[3]; //��������ϵ-�ٶ�
        float inertialVel[3]; //��������ϵ-�ٶ�

    } state_t;

} // namespace hf
