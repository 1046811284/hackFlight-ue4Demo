/*
   Mixer subclass for X-configuration quadcopters following the ArduPilot numbering convention:

    3cw   1ccw
       \ /
        ^
       / \
    2ccw  4cw
 
   Copyright (c) 2019 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "board.hpp"
#include "mixer.hpp"
#include "datatypes.hpp"

namespace hf {


	
    class MixerQuadXAP : public Mixer {

        public:

            MixerQuadXAP(void) 
                : Mixer(4)
            {
				//和无人机的: 类型和旋翼位置有关    
					//这里是四旋翼"X"型:
                //                     Th  RR  PF  YR
                motorDirections[0] = { +1, -1, -1, -1 };    // 1 right front  右前  ===> 索引 1/2/3/4 ==> 分别对应的是哪个旋翼
                motorDirections[1] = { +1, +1, +1, -1 };    // 2 left rear  左后
                motorDirections[2] = { +1, +1, -1, +1 };    // 3 left front  左前
                motorDirections[3] = { +1, -1, +1, +1 };    // 4 right rear  右后

				//4个值分别是:
				//typedef struct motorMixer_t {
				//	int8_t throttle; // T  ==> th
				//	int8_t roll; 	 // A   ==>  rr
				//	int8_t pitch;	 // E   ==> pf
				//	int8_t yaw;	     // R    ==>yr
				//} motorMixer_t;
            }	//
    };

} // namespace
