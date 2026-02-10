/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control_ppc.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#include "AttitudeControl/AttitudeControlMath.hpp"

using namespace matrix;

MulticopterAttitudeControlPPC::MulticopterAttitudeControlPPC(bool vtol) :
	ModuleParams(nullptr),/*初始化参数基类，传入nullptr表示该模块是参数基类的顶层列表*/
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),/*指定工作队列*/
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)
{
	parameters_updated();

	// Rate of change 5% per second -> 1.6 seconds to ramp to default 8% MPC_MANTHR_MIN
	/*设定油门变化率最小值，这里设定5%，则最大需要1.6s达到MPC_MANTHR_MIN(8%),即手动油门最小值*/
	/*MCP - Multi-rotor Position Controller, MANTHR = MANual THRottle（手动油门） */
	_manual_throttle_minimum.setSlewRate(0.05f);
	// Rate of change 50% per second -> 2 seconds to ramp to 100%
	_manual_throttle_maximum.setSlewRate(0.5f);
	// Rate of change 5% per second -> 6 seconds to ramp 30% if hover thrust parameter is off
	/*悬停推力变化率*/
	_hover_thrust_slew_rate.setSlewRate(0.05f);

	/*角速度*/
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	/* publish()会自动调用advertise()，这里提前注册的原因可能是在模块启动前，其他任务需要检查该主题的存在性 */
	/* 或者多实例情况下，占用实例编号等 */
	_controller_status_pub.advertise();
}

MulticopterAttitudeControlPPC::~MulticopterAttitudeControlPPC()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControlPPC::init()
{
	// if (!_vehicle_attitude_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterAttitudeControlPPC::parameters_updated()
{
	/*姿态*/
	// Store some of the parameters in a more convenient way & precompute often-used values
	// _attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
	// 				      _param_mc_yaw_weight.get());

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	// Update from hover thrust parameter if there's no valid estimate in use
	/*如果悬停推力估计值无效，则使用参数中的默认悬停推力值*/
	if (!PX4_ISFINITE(_hover_thrust_estimate)) {
		_hover_thrust_slew_rate.setForcedValue(_param_mpc_thr_hover.get());
	}

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());

	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	/*Vector3f：合并为一个三维向量*/
	// const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	/*角速度*/
	/*设置PID增益*/
	/*emult：逐元素乘*/
	// _rate_control.setPidGains(
	// 	rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
	// 	rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
	// 	rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	/*设置积分饱和*/
	/*rr：roll rate, pr：pitch rate, yr：yaw rate*/
	// _rate_control.setIntegratorLimit(
	// 	Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	/*设置前馈增益*/
	// _rate_control.setFeedForwardGain(
	// 	Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	/*设置特技模式最大角速率*/
	/*radians：角度转弧度*/
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	/*设置yaw低通滤波*/
	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}


/**
 * @brief 杆量映射函数，根据油门杆输入计算出对应的推力值。根据参数MPC_THR_CURVE选择三种映射模式
 *
 * @param throttle_stick_input
 * @return float
 */
float
MulticopterAttitudeControlPPC::throttle_curve(float throttle_stick_input)
{
	float thrust = 0.f;

	// throttle_stick_input is in range [-1, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling
		/*线性无重缩放，杆量(-1~1)线性映射到_manual_throttle_minimum~_param_mpc_thr_max*/
		thrust = math::interpolate(throttle_stick_input, -1.f, 1.f,
					   _manual_throttle_minimum.getState(), _param_mpc_thr_max.get());
		break;

	case 2: // rescale to hover thrust param at 0 stick input
		/*分段线性映射，将**摇杆中位**映射到用户设置的**悬停推力参数***/
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
		break;

	default: // 0 or other: rescale to HTE value
		/*默认(最常用)，中位摇杆映射为_hover_thrust_slew_rate，_hover_thrust_slew_rate由悬停推力估计器输出*/
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _hover_thrust_slew_rate.getState(), _param_mpc_thr_max.get()});
		break;
	}

	/*限幅，_manual_throttle_maximum可以保证开始时thrust不会突变(受限于_manual_throttle_maximum变化率)*/
	return math::min(thrust, _manual_throttle_maximum.getState());
}

void
MulticopterAttitudeControlPPC::generate_attitude_setpoint(const Quatf &q, float dt)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	// Avoid accumulating absolute yaw error with arming stick gesture
	const bool arming_gesture = (_manual_control_setpoint.throttle < -.9f) && (_param_mc_airmode.get() != 2);

	if (arming_gesture) {
		_yaw_setpoint_stabilized = NAN;
	}

	const float yaw = Eulerf(q).psi();
	const float yaw_stick_input = math::expo_deadzone(_manual_control_setpoint.yaw, .6f, _param_man_deadzone.get());
	_stick_yaw.generateYawSetpoint(attitude_setpoint.yaw_sp_move_rate, _yaw_setpoint_stabilized, yaw_stick_input, yaw, dt,
				       _unaided_heading);

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(roll*roll + pitch*pitch)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	_man_roll_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_pitch_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());

	// we want to fly towards the direction of (roll, pitch)
	Vector2f v = Vector2f(_man_roll_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max),
			      -_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max));
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);
	// Make sure there's a valid attitude quaternion with no yaw error when yaw is unlocked (NAN)
	const float yaw_setpoint = PX4_ISFINITE(_yaw_setpoint_stabilized) ? _yaw_setpoint_stabilized : yaw;
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	const Quatf q_sp_yaw(cosf(yaw_setpoint / 2.f), 0.f, 0.f, sinf(yaw_setpoint / 2.f));

	if (_vtol) {
		// Modify the setpoints for roll and pitch such that they reflect the user's intention even
		// if a large yaw error(yaw_sp - yaw) is present. In the presence of a yaw error constructing
		// an attitude setpoint from the yaw setpoint will lead to unexpected attitude behaviour from
		// the user's view as the tilt will not be aligned with the heading of the vehicle.

		AttitudeControlMath::correctTiltSetpointForYawError(q_sp_rp, q, q_sp_yaw);
	}

	// Align the desired tilt with the yaw setpoint
	Quatf q_sp = q_sp_yaw * q_sp_rp;

	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);

	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}


void MulticopterAttitudeControlPPC::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}


void
MulticopterAttitudeControlPPC::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// Update hover thrust for stick scaling
	if (_hover_thrust_estimate_sub.updated()) {
		hover_thrust_estimate_s hover_thrust_estimate;

		if (_hover_thrust_estimate_sub.update(&hover_thrust_estimate)) {
			if (hover_thrust_estimate.valid) {
				_hover_thrust_estimate = math::constrain(hover_thrust_estimate.hover_thrust, .05f, .9f);

			} else {
				// Possibly bad estimate before it got invalid, slew back to parameter
				_hover_thrust_estimate = _param_mpc_thr_hover.get();
			}
		}
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		vehicle_attitude_s v_att;
		_vehicle_attitude_sub.update(&v_att);
		const Quatf q{v_att.q};

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		/*飞手的直接输入,处于 `Manual` 或 `Stabilized` 模式，姿态设定点将直接从这里计算得出*/
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				/*车辆类型与状态获取，决定了后面的控制模块是否使用逻辑*/
				_vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol = vehicle_status.is_vtol;
				_vtol_in_transition_mode = vehicle_status.in_transition_mode;
				_vtol_tailsitter = vehicle_status.is_vtol_tailsitter;

				/*是否解锁*/
				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				/*_spooled_u怠速起转完成标志，当飞机解锁后，时间大于_param_com_spoolup_time参数设定的时间，置一*/
				/*这个标志会在后续失能积分器和限制最大油门，防止侧翻*/
				_spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
			}
		}

		/*落地检测*/
		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}


		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				/*_unaided_heading,未辅助的航向角，用于在GPS失效时保持航向稳定*/
				_unaided_heading = vehicle_local_position.unaided_heading;
			}
		}

		// // use rates setpoint topic
		// vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		// if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
		// 	// generate the rate setpoint from sticks
		// 	manual_control_setpoint_s manual_control_setpoint;

		// 	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		// 		// manual rates control - ACRO mode
		// 		const Vector3f man_rate_sp{
		// 			math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
		// 			math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
		// 			math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

		// 		_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
		// 		_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
		// 		_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

		// 		// publish rate setpoint
		// 		vehicle_rates_setpoint.roll = _rates_setpoint(0);
		// 		vehicle_rates_setpoint.pitch = _rates_setpoint(1);
		// 		vehicle_rates_setpoint.yaw = _rates_setpoint(2);
		// 		_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
		// 		vehicle_rates_setpoint.timestamp = hrt_absolute_time();

		// 		_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
		// 	}

		// } else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
		// 	if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
		// 		_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
		// 		_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
		// 		_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
		// 		_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
		// 	}
		// }


		// during transitions VTOL module generates attitude setpoints
		/*下面三行代码决定了飞机是否使用本模块进行姿态控制*/
		/*对于普通多旋翼，只需flag_control_attitude_enabled=true*/
		/*对于vtol，如果在旋翼模式(is_hovering)，由本模块控制*/
		/*特例 (Tailsitter)，尾座式vtol（像火箭一样立着飞）在转换期间依然使用多旋翼的姿态控制算法。*/
		const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);
		const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);

		const bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled
					  && (is_hovering || is_tailsitter_transition);
		Quatf qe{};
		Vector3f epsilon_q{};
		Matrix3f Q_e{};
		Matrix3f R_q{};

		if (run_att_ctrl) {
			// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
			/*纯手动模式*/
			/*开启手动控制，且没有高度、速度、位置控制的辅助*/
			/*Stabilized (自稳), Manual (手动), Acro (特技)。*/
			if (_vehicle_control_mode.flag_control_manual_enabled &&
			    !_vehicle_control_mode.flag_control_altitude_enabled &&
			    !_vehicle_control_mode.flag_control_velocity_enabled &&
			    !_vehicle_control_mode.flag_control_position_enabled) {

				generate_attitude_setpoint(q, dt);

			} else {
				/*自动模式*/
				/*Altitude (定高), Position (定点), Mission (航点), Offboard*/
				/*重置手动输入的滤波器（防止切换回手动时残留旧数据），然后等待接收外部传入的设定点*/
				_man_roll_input_filter.reset(0.f);
				_man_pitch_input_filter.reset(0.f);
				_yaw_setpoint_stabilized = NAN;
				_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
			}

			// Check for new attitude setpoint
			/*非纯手动模式，目标姿态通常由位置控制器（`mc_pos_control`）计算好并通过 uORB 发送过来*/
			if (_vehicle_attitude_setpoint_sub.updated()) {
				vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
				    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

					_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
					_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
					_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
				}
			}

			// Check for a heading reset
			if (_quat_reset_counter != v_att.quat_reset_counter) {
				const Quatf delta_q_reset(v_att.delta_q_reset);
				const float delta_psi = Eulerf(delta_q_reset).psi();

				// Only offset the yaw setpoint when the heading is locked
				if (PX4_ISFINITE(_yaw_setpoint_stabilized)) {
					_yaw_setpoint_stabilized = wrap_pi(_yaw_setpoint_stabilized + delta_psi);
				}

				_stick_yaw.ekfResetHandler(delta_psi);

				if (v_att.timestamp > _last_attitude_setpoint) {
					// adapt existing attitude setpoint unless it was generated after the current attitude estimate
					_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
				}

				_quat_reset_counter = v_att.quat_reset_counter;
			}

			Vector3f rates_sp = _attitude_control.update(q, qe, epsilon_q, Q_e, R_q);

			const hrt_abstime now1 = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;

			if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
				if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
				    && ((now1 - pid_autotune.timestamp) < 1_s)) {
					rates_sp += Vector3f(pid_autotune.rate_sp);
				}
			}

			// publish rate setpoint
			vehicle_rates_setpoint_s rates_setpoint{};
			rates_setpoint.roll = rates_sp(0);
			rates_setpoint.pitch = rates_sp(1);
			rates_setpoint.yaw = rates_sp(2);
			_thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
			rates_setpoint.timestamp = hrt_absolute_time();

			_vehicle_rates_setpoint_pub.publish(rates_setpoint);

		} else {
			_man_roll_input_filter.reset(0.f);
			_man_pitch_input_filter.reset(0.f);
			_yaw_setpoint_stabilized = NAN;
			_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
		}

		if (_landed) {
			_manual_throttle_minimum.update(0.f, dt);

		} else {
			_manual_throttle_minimum.update(_param_mpc_manthr_min.get(), dt);
		}

		if (_spooled_up) {
			_manual_throttle_maximum.update(1.f, dt);

		} else {
			_manual_throttle_maximum.setForcedValue(0.f);
		}

		if (PX4_ISFINITE(_hover_thrust_estimate)) {
			_hover_thrust_slew_rate.update(_hover_thrust_estimate, dt);
		}


		/*角速度控制*/
		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// =================================================================================
			// 自定义角速度控制算法 (移植区域)
			// =================================================================================
			Vector3f torque_setpoint;

			// float q_e0 = qe(0);
			Vector3f q_ev(qe(1), qe(2), qe(3));
			Vector3f z_3 = epsilon_q;

			// 2. 参数定义
			// 惯性张量 J_b (MATLAB: diag([0.16;0.16;0.32])),大致值即可
			const Matrix3f J_b = diag(Vector3f(0.16f, 0.16f, 0.32f));
			// 控制增益
			const Matrix3f k_w = diag(Vector3f(20.f, 20.f, 20.f));
			// 自适应参数
			const Vector3f lambda_w_vec(8.f, 8.f, 8.f);
			const Vector3f lambda_q_vec(8.f, 8.f, 8.f);
			// const Matrix3f lambda_q_mat = diag(lambda_q_vec);

			// 状态变量 (积分器)
			static Vector3f alpha_q(0.f, 0.f, 0.f);
			static Vector3f alpha_w(0.f, 0.f, 0.f);

			// Reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed) {
				alpha_q.setZero();
				alpha_w.setZero();
			}

			// ============================
			// 4. I&I 估计器更新
			// ============================

			// 当前机体角速度
			Vector3f w_b_b = rates;

			// --- Estimator 1: Attitude (d_qg part) ---
			// d_qg = alpha_q + lambda_q * q_ev
			Vector3f d_qg = alpha_q + lambda_q_vec.emult(q_ev);

			// dalpha_q = -lambda_q * (Q_e * (w_b_b + d_qg))
			Vector3f dalpha_q = -lambda_q_vec.emult(Q_e * (w_b_b + d_qg));

			// 积分 alpha_q
			if (PX4_ISFINITE(dalpha_q(0)) && !_landed) {
				alpha_q += dalpha_q * dt;
			}

			// --- Estimator 2: Angular Velocity (d_wg part) ---
			// z_4 = w_e - w_ec
			// w_e 是当前角速度 w_b_b
			// w_ec 是期望角速度，对应 _rates_setpoint
			Vector3f w_e = w_b_b + d_qg;
			Vector3f z_4 = w_e - _rates_setpoint;

			// d_wg = alpha_w + lambda_w * J_b * z_4
			Vector3f d_wg = alpha_w + lambda_w_vec.emult(J_b * z_4);

			// 计算力矩 tau
			// MATLAB: tau = -k_w*z_4 - Q_e.'*R_q.'*z_3 - d_wg;
			Vector3f coupling_term = Q_e.transpose() * R_q.transpose() * z_3;
			Vector3f tau = -k_w * z_4 - coupling_term - d_wg;

			// dalpha_w = -lambda_w * (tau + d_wg)
			Vector3f dalpha_w = -lambda_w_vec.emult(tau + d_wg);

			// 积分 alpha_w
			if (PX4_ISFINITE(dalpha_w(0)) && !_landed) {
				alpha_w += dalpha_w * dt;
			}

			// 5. 输出赋值
			// PX4 的 RateControl 输出通常是归一化的 torque (-1 to 1) 或者是物理力矩
			// 如果 tau 是物理力矩 (Nm)，需要除以最大力矩或进行缩放
			// 假设需要直接输出物理力矩，通过 mixer 处理，这里直接赋值
			torque_setpoint = tau;
			// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			// _rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
		}

		perf_end(_loop_perf);
	}
}

int MulticopterAttitudeControlPPC::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttitudeControlPPC *instance = new MulticopterAttitudeControlPPC(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterAttitudeControlPPC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControlPPC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_att_control_ppc_main(int argc, char *argv[])
{
	return MulticopterAttitudeControlPPC::main(argc, argv);
}
