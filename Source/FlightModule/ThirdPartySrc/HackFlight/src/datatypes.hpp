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

        bool  armed;//起落架??

        float location[3]; //位置
        float rotation[3]; //旋转
        float angularVel[3]; //角速度
        float bodyAccel[3];  //本地坐标系-加速度
        float bodyVel[3]; //本地坐标系-速度
        float inertialVel[3]; //惯性坐标系-速度

    } state_t;

} // namespace hf
