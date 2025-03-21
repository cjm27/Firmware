/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file foil_params.c
 *
 * Circuit breaker parameters.
 * Analog to real aviation circuit breakers these parameters
 * allow to disable subsystems. They are not supported as standard
 * operation procedure and are only provided for development purposes.
 * To ensure they are not activated accidentally, the associated
 * parameter needs to set to the key (magic).
 */

/**
 * Foil aileron proportional gain
 *
 * This is the proportional gain for the foil aileron closed loop controller
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Foil Control
 */
PARAM_DEFINE_FLOAT(FOIL_AIL_P, 2.0f);

/**
 * Foil aileron integral gain
 *
 * This is the integral gain for the foil aileron closed loop controller
 *
 * @unit %m/s
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Foil Control
 */
PARAM_DEFINE_FLOAT(FOIL_AIL_I, 0.1f);

/**
 * Foil aileron proportional gain
 *
 * This is the derivative gain for the foil aileron closed loop controller
 *
 * @unit %m/s
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Foil Control
 */
PARAM_DEFINE_FLOAT(FOIL_AIL_D, 0.0f);