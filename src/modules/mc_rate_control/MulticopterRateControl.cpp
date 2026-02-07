/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>



// 宏定义开关：1 使用自定义 I&I 控制算法，0 使用 PX4 原生 PID
#define USE_CUSTOM_RATE_CONTROL 0

using namespace matrix;
using namespace time_literals;
using math::radians;

/**
 * @brief 构造函数
 *
 * @param vtol：是否为VTOL飞行器
 */
MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	/* publish()会自动调用advertise()，这里提前注册的原因可能是在模块启动前，其他任务需要检查该主题的存在性 */
	/* 或者多实例情况下，占用实例编号等 */
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	/*Vector3f：合并为一个三维向量*/
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	/*设置PID增益*/
	/*emult：逐元素乘*/
	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	/*设置积分饱和*/
	/*rr：roll rate, pr：pitch rate, yr：yaw rate*/
	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	/*设置前馈增益*/
	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	/*设置特技模式最大角速率*/
	/*radians：角度转弧度*/
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	/*设置yaw低通滤波*/
	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}

void
MulticopterRateControl::Run()
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

		/*同步数据*/
		updateParams();
		/*应用配置*/
		parameters_updated();
	}

// #if USE_CUSTOM_RATE_CONTROL
// 	// 定义局部订阅器，因为 MulticopterRateControl.hpp 未修改，无法添加成员变量
// 	// 注意：在 Run() 循环外定义以保持连接，但 Run() 本身是基于回调或循环调度的
// 	// 如果 Run 是每次回调触发，这里需要 static 或者成员变量。
// 	// PX4 WorkItem 调度中 Run() 是被调用的。为了简单演示，这里使用 static 模拟成员变量的效果
// 	static uORB::Subscription v_att_sub{ORB_ID(vehicle_attitude)};
// 	static uORB::Subscription v_att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
// #endif

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

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

#if USE_CUSTOM_RATE_CONTROL
			// =================================================================================
			// 自定义角速度控制算法 (移植区域)
			// =================================================================================
			Vector3f torque_setpoint;
			// 1. 获取必要的姿态数据 (用于计算 estimators 和 coupling terms)
			vehicle_attitude_s att;
			vehicle_attitude_setpoint_s att_sp;
			v_att_sub.update(&att);
			v_att_sp_sub.update(&att_sp);

			// 如果没有姿态数据，暂时使用 PID 作为 fallback 或者 0
			// 这里假设数据是存在的
			Quatf q(att.q);
			Quatf qd(att_sp.q_d);

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

			// 3. 重算姿态误差项 (q_ev, Q_e, R_q, z_3)
			// 为了得到 Q_e 和 z_3，我们需要运行一部分 Attitude Logic

			// 3.1 计算 q_e
			// Quatf q_e = q.inversed() * qd;
			// q_e.canonicalize();
			// float q_e0 = q_e(0);
			// Vector3f q_ev(q_e(1), q_e(2), q_e(3));

			// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
			const Vector3f e_z = q.dcm_z();
			const Vector3f e_z_d = qd.dcm_z();
			Quatf qd_red(e_z, e_z_d);

			if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
				// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
				// full attitude control anyways generates no yaw input and directly takes the combination of
				// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
				qd_red = qd;

			} else {
				// Transform rotation from current to desired thrust vector into a world frame reduced desired attitude.
				// This is a right multiplication as the tilt error quaternion is obtained from two Z vectors expressed in the world frame.
				qd_red *= q;
			}

			// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
			// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
			Quatf qd_dyaw = qd_red.inversed() * qd;
			qd_dyaw.canonicalize();
			// catch numerical problems with the domain of acosf and asinf
			qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
			qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);
			float _yaw_w = 0.4f;
			// scale the delta yaw angle and re-combine the desired attitude
			qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

			// quaternion attitude control law, qe is rotation from q to qd
			// const Quatf qe = q.inversed() * qd;
			const Quatf qe = qd.inversed() * q;

			float q_e0 = qe(0);
			Vector3f q_ev(qe(1), qe(2), qe(3));

			// 3.2 计算 PPF (Performance Prescribed Function)
			// 为了计算 z_3 (epsilon_q)，需要 eta_Theta
			// 使用静态时间基准
			static uint64_t start_time_us = 0;

			if (start_time_us == 0 && _vehicle_control_mode.flag_armed) {
				start_time_us = now; // 解锁时重置时间
			}

			float t = (_vehicle_control_mode.flag_armed) ? (now - start_time_us) * 1e-6f : 0.f;

			// PPF 参数
			const Vector3f eta_Theta_0(1.1f, 1.1f, 1.1f);
			const Vector3f eta_ThetaTf(0.0005f, 0.0005f, 0.0005f);
			const Vector3f h_Theta1(0.f, 0.f, 0.f);
			const Vector3f h_Theta2(0.2f, 0.2f, 0.2f);
			const Vector3f Tf_Theta(20.f, 20.f, 20.f);

			// Vector3f eta_Theta;
			// Vector3f deta_Theta; // 需要导数计算 n_q，进而算 tau 里的项 (虽然后面 tau 公式里只有 z_3)
			// // MATLAB 中 tau = -k_w*z_4 - Q_e.'*R_q.'*z_3 - d_wg;
			// // 这里的 Q_e.'*R_q.'*z_3 就是耦合项

			// // 简化的 PPF 计算循环
			// for (int i = 0; i < 3; i++) {
			// 	float lota = 1.0f / (1.0f + expf(-(h_Theta1(i) + h_Theta2(i))));
			// 	float temp = 1.0f / (1.0f + expf(-(h_Theta1(i) - h_Theta2(i))));
			// 	float eta0_val = (eta_Theta_0(i) - eta_ThetaTf(i)) / (lota - temp);

			// 	if (t >= 0 && t < Tf_Theta(i)) {
			// 		float arg = t * (float)M_PI / Tf_Theta(i);
			// 		float varpi = h_Theta1(i) - h_Theta2(i) * cosf(arg);
			// 		float sig = 1.0f / (1.0f + expf(-varpi));
			// 		eta_Theta(i) = eta0_val * (lota - sig) + eta_ThetaTf(i);
			// 		// deta_Theta 计算省略，如果不需要 n_q 在角速度环显示使用

			// 	} else {
			// 		eta_Theta(i) = eta_ThetaTf(i);
			// 	}
			// }

			// // 3.3 计算 z_3 (epsilon_q) 和 R_q
			// Vector3f z_3; // epsilon_q
			// Matrix3f R_q = Matrix3f::identity();

			// for (int i = 0; i < 3; i++) {
			// 	float ratio = math::constrain(q_ev(i) / eta_Theta(i), -0.99f, 0.99f);
			// 	z_3(i) = 0.5f * logf((1.0f + ratio) / (1.0f - ratio));
			// 	R_q(i, i) = 1.0f / ((1.0f + ratio) * (1.0f - ratio));
			// }

			// // 3.4 计算 Q_e
			// Matrix3f hat_q_ev; // Skew-symmetric
			// hat_q_ev(0, 0) = 0.f;       hat_q_ev(0, 1) = -q_ev(2);  hat_q_ev(0, 2) = q_ev(1);
			// hat_q_ev(1, 0) = q_ev(2);   hat_q_ev(1, 1) = 0.f;       hat_q_ev(1, 2) = -q_ev(0);
			// hat_q_ev(2, 0) = -q_ev(1);  hat_q_ev(2, 1) = q_ev(0);   hat_q_ev(2, 2) = 0.f;

			// Matrix3f Q_e = (Matrix3f::identity() * q_e0 + hat_q_ev) * 0.5f;


			// 4. PPF (Performance Prescribed Function) 计算
			// 对应 MATLAB 代码段:
			// varpi_Theta=h_Theta1-h_Theta2.*cos(t*pi./Tf_Theta); ...
			Vector3f eta_Theta;
			Vector3f deta_Theta;

			// 辅助计算变量
			Vector3f lota_Theta, temp_Theta, eta_Theta0_val;

			for (int i = 0; i < 3; i++) {
				// 计算常数项 (对应 MATLAB: lota_Theta, temp_Theta)
				lota_Theta(i) = 1.0f / (1.0f + expf(-(h_Theta1(i) + h_Theta2(i))));
				temp_Theta(i) = 1.0f / (1.0f + expf(-(h_Theta1(i) - h_Theta2(i))));
				eta_Theta0_val(i) = (eta_Theta_0(i) - eta_ThetaTf(i)) / (lota_Theta(i) - temp_Theta(i));

				if (t >= 0 && t < Tf_Theta(i)) {
					float arg_cos = t * (float)M_PI / Tf_Theta(i);
					float varpi = h_Theta1(i) - h_Theta2(i) * cosf(arg_cos);
					float exp_neg_varpi = expf(-varpi);
					float sigmoid = 1.0f / (1.0f + exp_neg_varpi);

					// eta_Theta
					eta_Theta(i) = eta_Theta0_val(i) * (lota_Theta(i) - sigmoid) + eta_ThetaTf(i);

					// // deta_Theta (导数)
					// // MATLAB: -pi*h2*eta0*exp(-varpi)*sin(...)/ (Tf * (1+exp)^2)
					// float num = -(float)M_PI * h_Theta2(i) * eta_Theta0_val(i) * exp_neg_varpi * sinf(arg_cos);
					// float den = Tf_Theta(i) * powf(1.0f + exp_neg_varpi, 2.0f);
					// deta_Theta(i) = num / den;

				} else {
					eta_Theta(i) = eta_ThetaTf(i);
					// deta_Theta(i) = 0.f;
				}
			}

			// 5. 误差转换 (Error Transformation)
			// MATLAB: epsilon_q(ii) = 0.5*log((1+q_ev(ii)/eta)/(1-q_ev(ii)/eta));
			Vector3f epsilon_q;
			Vector3f r_q;
			// Vector3f n_q;

			Matrix3f R_q{}; // R_q 是对角阵

			for (int i = 0; i < 3; i++) {
				// 安全保护：防止除以0或log负数
				// 实际上 PPF 保证了 |q_ev| < eta，但数值计算需要保险
				float ratio = math::constrain(q_ev(i) / eta_Theta(i), -0.99f, 0.99f);

				epsilon_q(i) = 0.5f * logf((1.0f + ratio) / (1.0f - ratio));

				// 计算辅助变量 r_q, n_q
				// MATLAB: r_q = 1/((1+ratio)*(1-ratio)); Note: extra eta division was in comment, checking logic
				// MATLAB Code: r_q(ii) = 1/((1+q/eta)*(1-q/eta));
				// 这里的公式分母其实是 (1 - ratio^2)
				r_q(i) = 1.0f / ((1.0f + ratio) * (1.0f - ratio));

				// MATLAB: n_q(ii) = -r_q(ii)*q_ev(ii)*deta_Theta(ii)/eta_Theta(ii);
				// n_q(i) = -r_q(i) * q_ev(i) * deta_Theta(i) / eta_Theta(i);

				// 构建 R_q 矩阵 (对角线元素)
				R_q(i, i) = r_q(i);
			}

			Vector3f z_3 = epsilon_q;
			Matrix3f hat_q_ev; // q_ev 的反对称矩阵 (vx)
			hat_q_ev(0, 0) = 0.f;       hat_q_ev(0, 1) = -q_ev(2);  hat_q_ev(0, 2) = q_ev(1);
			hat_q_ev(1, 0) = q_ev(2);   hat_q_ev(1, 1) = 0.f;       hat_q_ev(1, 2) = -q_ev(0);
			hat_q_ev(2, 0) = -q_ev(1);  hat_q_ev(2, 1) = q_ev(0);   hat_q_ev(2, 2) = 0.f;

			// Q_e = 0.5 * (q_e0 * I + hat(q_ev))
			Matrix3f Q_e = (matrix::eye<float, 3>() * q_e0 + hat_q_ev) * 0.5f;



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

#else

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller
			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);
#endif
			// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
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
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
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

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

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

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
