/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

/**
 * ID of WINGLET the calibration is for.
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_WINGLET0_ID, 0);

/**
 * WINGLET 0 enabled
 *
 * @boolean
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_WINGLET0_EN, 1);

/**
 * WINGLET W-axis ANGLE
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_WINGLET0_W, 0.0f);

/**
 * WINGLET X-axis ANGLE
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_WINGLET0_X, 0.0f);

/**
 * WINGLET Y-axis ANGLE
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_WINGLET0_Y, 0.0f);

/**
 * WINGLET Z-axis ANGLE
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_WINGLET0_Z, 0.0f);

