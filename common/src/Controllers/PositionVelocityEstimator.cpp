/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "Controllers/PositionVelocityEstimator.h"

/*!
 * Initialize the state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup()
{
	T dt = this->_stateEstimatorData.parameters->controller_dt;
	_xhat.setZero();
	_ps.setZero();
	_vs.setZero();
	_A.setZero();
	_A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
	_A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
	_A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
	_A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();
	_B.setZero();
	_B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
	C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
	C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
	_C.setZero();
	_C.block(0, 0, 3, 6) = C1;
	_C.block(3, 0, 3, 6) = C1;
	_C.block(6, 0, 3, 6) = C1;
	_C.block(9, 0, 3, 6) = C1;
	_C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();
	_C.block(12, 0, 3, 6) = C2;
	_C.block(15, 0, 3, 6) = C2;
	_C.block(18, 0, 3, 6) = C2;
	_C.block(21, 0, 3, 6) = C2;
	_C(27, 17) = T(1);
	_C(26, 14) = T(1);
	_C(25, 11) = T(1);
	_C(24, 8) = T(1);
	_P.setIdentity();
	_P = T(100) * _P;
	_Q0.setIdentity();
	_Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();		 // 位置估计到噪声？？？？
	_Q0.block(3, 3, 3, 3) = (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity(); // 速度噪声？？
	_Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();			 // 站立高度噪声？？
	_R0.setIdentity();

	// realsense T265的初始化
	//  Add pose stream
	//    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	//    // Start pipeline with chosen configuration
	//    pipe.start(cfg);
}

// template <typename T>
// LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator() {}

template <typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator() //: myLCM ( getLcmUrl ( 255 ) )
{
	_t265LcmThread = std::thread(&LinearKFPositionVelocityEstimator<T>::handleInterfaceLCM, this);
	if (!myLCM.good())
	{
		printf("my lcm _interfaceLCM failed to initialize\n");
	}

	myLCM.subscribe("t265_position_msg", &LinearKFPositionVelocityEstimator<T>::handleT265LCM, this);
}
template <typename T>
void LinearKFPositionVelocityEstimator<T>::handleInterfaceLCM()
{
	while (!_interfaceLcmQuit)
	{
		myLCM.handle();
	}
}

template <typename T>
void LinearKFPositionVelocityEstimator<T>::handleT265LCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
														 const T265position_t *msg)
{
	(void)rbuf;
	(void)chan;
	t265_position_x = msg->posBody[0];
	t265_position_y = msg->posBody[1];
	t265_velocity_x = msg->velBody[0];
	t265_velocity_y = msg->velBody[1];
	printf("received t265: %.2f\t%.2f\n", t265_position_x, t265_position_y);
}
/*!
 * Run state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::run()
{
	T process_noise_pimu =
		this->_stateEstimatorData.parameters->imu_process_noise_position;
	T process_noise_vimu =
		this->_stateEstimatorData.parameters->imu_process_noise_velocity;
	T process_noise_pfoot =
		this->_stateEstimatorData.parameters->foot_process_noise_position;
	T sensor_noise_pimu_rel_foot =
		this->_stateEstimatorData.parameters->foot_sensor_noise_position;
	T sensor_noise_vimu_rel_foot =
		this->_stateEstimatorData.parameters->foot_sensor_noise_velocity;
	T sensor_noise_zfoot =
		this->_stateEstimatorData.parameters->foot_height_sensor_noise;

	Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();	   // 过程噪声协方差矩阵
	Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;	   //  位置估计到误差
	Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;	   //  速度估计到误差
	Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot; // 腿估计到位置误差

	Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();				  // 测量噪声 协方差矩阵
	R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;	  // imu传感器测量位置的噪声协方差
	R.block(12, 12, 12, 12) = _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot; // imu测量速度
	R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

	int qindex = 0;
	int rindex1 = 0;
	int rindex2 = 0;
	int rindex3 = 0;

	Vec3<T> g(0, 0, T(-9.81));
	Mat3<T> Rbod = this->_stateEstimatorData.result->rBody.transpose();
	// in old code, Rbod * se_acc + g
	Vec3<T> a = this->_stateEstimatorData.result->aWorld + g; // aWorld: 0 0 9.8 加上后z方向加速度应该为0（一般情况下）
															  //   std::cout << "A WORLD\n" << a << "\n";
	Vec4<T> pzs = Vec4<T>::Zero();
	Vec4<T> trusts = Vec4<T>::Zero();
	Vec3<T> p0, v0;
	p0 << _xhat[0], _xhat[1], _xhat[2]; // 当前估计到最合理的位置
	v0 << _xhat[3], _xhat[4], _xhat[5]; // 当前估计到最合理的速度

	// 这部分通过机器人本体感受器来计算 测量值,即躯干位置、躯干速度、躯干高度到测量值
	for (int i = 0; i < 4; i++)
	{
		int i1 = 3 * i;
		Quadruped<T> &quadruped = *(this->_stateEstimatorData.legControllerData->quadruped);
		Vec3<T> ph = quadruped.getHipLocation(i); // hip positions relative to CoM //hip位置
		// hw_i->leg_controller->leg_datas[i].p;
		Vec3<T> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p; // 躯干坐标系下足端位置
		// hw_i->leg_controller->leg_datas[i].v;
		Vec3<T> dp_rel = this->_stateEstimatorData.legControllerData[i].v; // 躯干坐标系下足段速度
		Vec3<T> p_f = Rbod * p_rel;										   // 世界坐标系下到足端位置
		Vec3<T> dp_f =													   // 世界坐标下足端速度
			Rbod * (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);

		qindex = 6 + i1;   // 过程噪声协方差矩阵Q中对应到第i条腿到索引
		rindex1 = i1;	   // 测量噪声协方差矩阵R中对应到第i条腿索引
		rindex2 = 12 + i1; // 速度测量噪声索引
		rindex3 = 24 + i;  // 躯干高度测量噪声索引

		T trust = T(1);
		T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1)); // 第i条腿到触地与否，从支撑开始为0，到支撑结束为1，摆动为0
		// T trust_window = T(0.25);
		T trust_window = T(0.2); // 相当于一个阈值

		if (phase < trust_window)
		{								  // 刚进入支撑相位前20%
			trust = phase / trust_window; // trust设置为
		}
		else if (phase > (T(1) - trust_window))
		{ // 支撑相位最后20%
			trust = (T(1) - phase) / trust_window;
		} //                       -----------        1   支撑
		// 以上操作使得trust成为    -               -
		//                      -                 -    0   摆动
		// T high_suspect_number(1000);
		T high_suspect_number(100);

		// printf("Trust %d: %.3f\n", i, trust);
		Q.block(qindex, qindex, 3, 3) =
			(T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);	 // 这使得摆动相和触地开始/结束时刻到噪声很大（1->100）倍
		R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);				 // 位置测量噪声
		R.block(rindex2, rindex2, 3, 3) =													 // 速度测量噪声
			(T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3); // 同样摆动相位时候噪声设置很大
		R(rindex3, rindex3) =
			(T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3); // 抬腿高度，同样是摆动相噪声很大，不值得信任

		trusts(i) = trust;

		_ps.segment(i1, 3) = -p_f;									// 世界坐标下到足端位置 和 躯干到运动是相对运动 观测位置数据  基于运动学（每条腿位置）计算的位置
		_vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f); //     观测速度数据  基于运动学（每条腿位置）计算的速度
		pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));					// 世界坐标系下当前躯干高度+当前腿i的z方向高度,乘以前面到信任度后，得到抬腿高度估计
	}

	Eigen::Matrix<T, 28, 1> y;
	y << _ps, _vs, pzs; // 运动学测量直，四个腿计算到躯干位置（每个腿 对应xyz三个位置， 四个腿就是12个位置，_ps是12x1，_vs也是12x1）
	_xhat = _A * _xhat + _B * a;
	Eigen::Matrix<T, 18, 18> At = _A.transpose();
	Eigen::Matrix<T, 18, 18> Pm = _A * _P * At + Q;
	Eigen::Matrix<T, 18, 28> Ct = _C.transpose();
	Eigen::Matrix<T, 28, 1> yModel = _C * _xhat;
	Eigen::Matrix<T, 28, 1> ey = y - yModel;
	Eigen::Matrix<T, 28, 28> S = _C * Pm * Ct + R;

	// todo compute LU only once
	Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);
	_xhat += Pm * Ct * S_ey;

	Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);
	_P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

	Eigen::Matrix<T, 18, 18> Pt = _P.transpose();
	_P = (_P + Pt) / T(2);

	if (_P.block(0, 0, 2, 2).determinant() > T(0.000001))
	{ // 如果xy方向的位置协方差行列式大于一个阈值
		_P.block(0, 2, 2, 16).setZero();
		_P.block(2, 0, 16, 2).setZero();
		_P.block(0, 0, 2, 2) /= T(10);
	}

	this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
	this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
	this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;
	/*
	//  //使用 T265的数据替换计算到数据
	//    this->_stateEstimatorData.result->vBody(0) = t265_velocity_x;
	//    this->_stateEstimatorData.result->vBody(1) = t265_velocity_y;
	//    this->_stateEstimatorData.result->position(0) = t265_position_x;
	//    this->_stateEstimatorData.result->position(1) = t265_position_y;
	//    this->_stateEstimatorData.result->vWorld = this->_stateEstimatorData.result->rBody.transpose()*this->_stateEstimatorData.result->vBody;

	//     std::cout<<"Esti_position: "<<this->_stateEstimatorData.result->position<<"\t \n Esti-vel:"<<this->_stateEstimatorData.result->vBody<<std::endl;

	//    pipe_cout++;
	//    if(pipe_cout%4==0){
	//        auto frames = pipe.wait_for_frames();
	//        // Get a frame from the pose stream
	//        auto f = frames.first_or_default(RS2_STREAM_POSE);
	//        // Cast the frame to pose_frame and get its data
	//        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
	//        if(pipe_cout%40==0)
	//        {
	//            printf("posxyz: %.3f\t%.3f\t%.3f\t%.3f\n",-pose_data.translation.z,-pose_data.translation.x,-pose_data.velocity.z,-pose_data.velocity.x);
	//            printf("kalman: %.3f\t%.3f\t%.3f\t%.3f\n",this->_stateEstimatorData.result->position(0),this->_stateEstimatorData.result->position(1),
	//                   this->_stateEstimatorData.result->vBody(0),this->_stateEstimatorData.result->vBody(1));
	//        }
	//    }
	*/
}

template class LinearKFPositionVelocityEstimator<float>;
template class LinearKFPositionVelocityEstimator<double>;

/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterPositionVelocityEstimator<T>::run()
{
	this->_stateEstimatorData.result->position = this->_stateEstimatorData.cheaterState->position.template cast<T>();
	this->_stateEstimatorData.result->vWorld =
		this->_stateEstimatorData.result->rBody.transpose().template cast<T>() * this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
	this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
}

template class CheaterPositionVelocityEstimator<float>;
template class CheaterPositionVelocityEstimator<double>;
