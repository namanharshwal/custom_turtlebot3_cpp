// Copyright 2026 Naman Harshwal
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ==============================================================================
// Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
// Owner/Supervisor:     Pradeep Singh Chauhan
// Workspace:            psnh_turtlebot_cpp_ws
// Package:              custom_bot_cpp
// File:                 custom_bot_motor_encoder.hpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
// ==============================================================================

#ifndef CUSTOM_BOT_MOTOR_ENCODER_HPP_
#define CUSTOM_BOT_MOTOR_ENCODER_HPP_

#include <stdint.h>

void setup();
void loop();
void readSerialCommand();
void executeCommand(char cmd, int left_val, int right_val);
void setMotorSpeeds(int left_pwm, int right_pwm);
void hardStop();
void leftEncoderISR();
void rightEncoderISR();
int  applyDeadZone(int pwm);

#endif  // CUSTOM_BOT_MOTOR_ENCODER_HPP_
