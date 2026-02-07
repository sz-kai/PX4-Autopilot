/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

#include <drivers/drv_hrt.h> // 引入时间库，用于计算 t

using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

// 定义宏开关：1 使用您的自定义算法，0 使用PX4原生算法
#define USE_CUSTOM_ATTITUDE_CONTROL 0

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
#if USE_CUSTOM_ATTITUDE_CONTROL
	// =================================================================================
	// 自定义姿态控制算法移植区域
	// =================================================================================

	// 1. 获取当前时间 t (单位：秒)
	// 为了实现PPF函数，需要相对于系统启动或解锁的时间。
	// 这里使用静态变量记录第一次调用的时间作为 t=0 的基准。
	static uint64_t start_time_us = 0;
	static uint64_t last_run_us = 0;
	uint64_t now_us = hrt_absolute_time();

	// 初始化时间基准
	if (start_time_us == 0) {
		start_time_us = now_us;
		last_run_us = now_us;
	}

	float dt = (now_us - last_run_us) * 1e-6f; // 计算时间步长，用于积分

	if (dt < 0.0001f) { dt = 0.004f; } // 防止dt过小或除零，默认给一个近似值 (250Hz)

	last_run_us = now_us;

	float t = (now_us - start_time_us) * 1e-6f; // 当前运行时间 t

	// 2. 参数定义 (来自您的MATLAB初始条件)
	// 注意：矩阵在PX4中通常使用 Vector3f 或 Matrix3f

	// PPF 参数
	const Vector3f eta_Theta_0(1.1f, 1.1f, 1.1f);
	const Vector3f eta_ThetaTf(0.0005f, 0.0005f, 0.0005f);
	const Vector3f h_Theta1(0.f, 0.f, 0.f);
	const Vector3f h_Theta2(0.2f, 0.2f, 0.2f);
	const Vector3f Tf_Theta(20.f, 20.f, 20.f);

	// 控制增益
	// MATLAB: k_Theta = diag([0.5;0.5;0.5]);
	const Vector3f k_Theta_vec(5.0f, 5.0f, 5.0f);
	// 直接使用 diag 将向量转为对角矩阵
	Matrix3f k_Theta = diag(k_Theta_vec);
	//k_Theta(0, 0) = k_Theta_vec(0); k_Theta(1, 1) = k_Theta_vec(1); k_Theta(2, 2) = k_Theta_vec(2);

	// 自适应参数
	// MATLAB: lambda_q = diag([8;8;8]);
	const Vector3f lambda_q_vec(8.f, 8.f, 8.f);
	// alpha_q 需要积分，使用静态变量保存状态
	static Vector3f alpha_q(0.f, 0.f, 0.f);

	// // 3. 计算姿态误差
	// // PX4中 _attitude_setpoint_q 是目标姿态 q_d
	// // q 是当前机体姿态 q_b
	// Quatf qd = _attitude_setpoint_q;

	// // 计算误差四元数 q_e
	// // MATLAB代码看起来使用的是 q_e = q_d * inv(q_b) 或者类似的变换
	// // PX4标准做法是 q_e = q.inversed() * qd (表示从当前转到期望)
	// // 为了匹配您的算法逻辑 q_ev = q_e(2:4)，我们使用PX4的标准差值计算
	// Quatf q_e = q.inversed() * qd;
	// q_e.canonicalize(); // 确保四元数规范化


	Quatf qd = _attitude_setpoint_q;

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

	// scale the delta yaw angle and re-combine the desired attitude
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	// const Quatf qe = q.inversed() * qd;
	const Quatf qe = qd.inversed() * q;

	float q_e0 = qe(0);
	Vector3f q_ev(qe(1), qe(2), qe(3));

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

			// deta_Theta (导数)
			// MATLAB: -pi*h2*eta0*exp(-varpi)*sin(...)/ (Tf * (1+exp)^2)
			float num = -(float)M_PI * h_Theta2(i) * eta_Theta0_val(i) * exp_neg_varpi * sinf(arg_cos);
			float den = Tf_Theta(i) * powf(1.0f + exp_neg_varpi, 2.0f);
			deta_Theta(i) = num / den;

		} else {
			eta_Theta(i) = eta_ThetaTf(i);
			deta_Theta(i) = 0.f;
		}
	}

	// 5. 误差转换 (Error Transformation)
	// MATLAB: epsilon_q(ii) = 0.5*log((1+q_ev(ii)/eta)/(1-q_ev(ii)/eta));
	Vector3f epsilon_q;
	Vector3f r_q;
	Vector3f n_q;

	Matrix3f R_q_mat{}; // R_q 是对角阵

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
		n_q(i) = -r_q(i) * q_ev(i) * deta_Theta(i) / eta_Theta(i);

		// 构建 R_q 矩阵 (对角线元素)
		R_q_mat(i, i) = r_q(i);
	}

	// 6. I&I 自适应律更新 (简化的)
	// MATLAB: d_qg = alpha_q + lambda_q * q_ev;
	// MATLAB: Q_e = 0.5*(q_e0*eye(3)+vx(q_ev));
	// MATLAB: dalpha_q = -lambda_q*(Q_e*(w_b_b+d_qg));

	// [重要问题]: update函数此时只有 const q，没有当前的机体角速度 w_b_b。
	// 在PX4架构中，AttitudeControl通常只负责计算期望角速度，不直接读取陀螺仪数据。
	// 为了使代码能跑通，这里有两种选择：
	// 1. 假设 w_b_b 为0或者使用期望值 (不准确)
	// 2. 暂时注释掉依赖 w_b_b 的动态更新部分，只保留结构。
	// 这里我们先计算静态部分。如果必须移植完整I&I，需要修改函数签名传入角速度。

	Matrix3f hat_q_ev; // q_ev 的反对称矩阵 (vx)
	hat_q_ev(0, 0) = 0.f;       hat_q_ev(0, 1) = -q_ev(2);  hat_q_ev(0, 2) = q_ev(1);
	hat_q_ev(1, 0) = q_ev(2);   hat_q_ev(1, 1) = 0.f;       hat_q_ev(1, 2) = -q_ev(0);
	hat_q_ev(2, 0) = -q_ev(1);  hat_q_ev(2, 1) = q_ev(0);   hat_q_ev(2, 2) = 0.f;

	// Q_e = 0.5 * (q_e0 * I + hat(q_ev))
	Matrix3f Q_e = (matrix::eye<float, 3>() * q_e0 + hat_q_ev) * 0.5f;

	// 尝试更新 alpha_q (由于缺少 w_b_b，这里暂时仅做示例，实际需传入 w_b_b)
	// Vector3f w_b_b = _rates_prev; // 假设有一个变量存了角速度，或者此处设为0
	// Vector3f d_qg = alpha_q + lambda_q_vec.emult(q_ev); // emult是对应元素相乘
	// Vector3f dalpha_q = -lambda_q_vec.emult(Q_e * (w_b_b + d_qg));
	// alpha_q += dalpha_q * dt; // 欧拉积分

	// 7. 计算输出期望角速度 (Step 1)
	// MATLAB: z_3 = epsilon_q;
	// MATLAB: w_ec = -Q_e^(-1) * R_q^(-1) * (k_Theta*z_3 + n_q);

	// 计算 Q_e 的逆
	Matrix3f Q_e_inv = inv(Q_e);

	// 计算 R_q 的逆 (因为是对角阵，直接取倒数)
	Matrix3f R_q_inv{};
	R_q_inv(0, 0) = 1.0f / R_q_mat(0, 0);
	R_q_inv(1, 1) = 1.0f / R_q_mat(1, 1);
	R_q_inv(2, 2) = 1.0f / R_q_mat(2, 2);

	Vector3f z_3 = epsilon_q;

	// 计算控制项
	Vector3f term = k_Theta * z_3 + n_q;

	// 最终输出: w_ec
	Vector3f w_ec = -Q_e_inv * R_q_inv * term;

	// 处理Feed forward yaw (保留PX4原有逻辑以兼容手动航向)
	if (std::isfinite(_yawspeed_setpoint)) {
		w_ec += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// 限制输出幅值
	for (int i = 0; i < 3; i++) {
		w_ec(i) = math::constrain(w_ec(i), -_rate_limit(i), _rate_limit(i));
	}

	return w_ec;

#elif !USE_CUSTOM_ATTITUDE_CONTROL
	Quatf qd = _attitude_setpoint_q;

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

	// scale the delta yaw angle and re-combine the desired attitude
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	const Vector3f eq = 2.f * qe.canonical().imag();

	// calculate angular rates setpoint
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
#endif
}
