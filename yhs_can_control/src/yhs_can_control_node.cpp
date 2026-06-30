#include "yhs_can_control/yhs_can_control_node.hpp"

namespace yhs
{

	CanControl::CanControl(rclcpp::Node::SharedPtr node)
			: node_(node), if_name_("can0"), can_socket_(-1), wheel_base_(0.6),
			  odom_frame_("odom"), base_link_frame_("base_link"), tf_used_(false),
			  last_imu_time_(0, 0, node_->get_clock()->get_clock_type()), imu_roll_(0.0), imu_pitch_(0.0), imu_yaw_(0.0),
			  odom_x_(0.0), odom_y_(0.0), odom_theta_(0.0), odom_last_time_(node_->now())
	{

		READ_PARAM(std::string, "can_name", (if_name_), "can0");
		READ_PARAM(double, "wheel_base", (wheel_base_), 0.6);
		READ_PARAM(std::string, "odom_frame", (odom_frame_), "odom");
		READ_PARAM(std::string, "base_link_frame", (base_link_frame_), "base_link");
		READ_PARAM(bool, "tf_used", (tf_used_), false);

		node_->declare_parameter<std::vector<int64_t>>("ultrasonic_number", std::vector<int64_t>{});
		node_->get_parameter("ultrasonic_number", ultrasonic_number_);

		node_->declare_parameter<bool>("io_cmd.enable", true);
		node_->declare_parameter<bool>("io_cmd.lower_beam", false);
		node_->declare_parameter<bool>("io_cmd.upper_beam", false);
		node_->declare_parameter<int>("io_cmd.turn_lamp", 0);
		node_->declare_parameter<bool>("io_cmd.braking_lamp", false);
		node_->declare_parameter<bool>("io_cmd.clearance_lamp", false);
		node_->declare_parameter<bool>("io_cmd.fog_lamp", false);
		node_->declare_parameter<bool>("io_cmd.speaker", false);
		node_->declare_parameter<bool>("io_cmd.discharge", false);

		current_io_cmd_.io_cmd_enable = node_->get_parameter("io_cmd.enable").as_bool();
		current_io_cmd_.io_cmd_lower_beam_headlamp = node_->get_parameter("io_cmd.lower_beam").as_bool();
		current_io_cmd_.io_cmd_upper_beam_headlamp = node_->get_parameter("io_cmd.upper_beam").as_bool();
		current_io_cmd_.io_cmd_turn_lamp = static_cast<uint8_t>(node_->get_parameter("io_cmd.turn_lamp").as_int());
		current_io_cmd_.io_cmd_braking_lamp = node_->get_parameter("io_cmd.braking_lamp").as_bool();
		current_io_cmd_.io_cmd_clearance_lamp = node_->get_parameter("io_cmd.clearance_lamp").as_bool();
		current_io_cmd_.io_cmd_fog_lamp = node_->get_parameter("io_cmd.fog_lamp").as_bool();
		current_io_cmd_.io_cmd_speaker = node_->get_parameter("io_cmd.speaker").as_bool();
		current_io_cmd_.io_cmd_discharge = node_->get_parameter("io_cmd.discharge").as_bool();

		io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::IoCmd>(
				"io_cmd",
				1,
				std::bind(&CanControl::io_cmd_callback, this, std::placeholders::_1));

		remote_torque_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::RemoteTorqueCtrlCmd>(
				"remote_torque_ctrl_cmd",
				1,
				std::bind(&CanControl::remote_torque_ctrl_cmd_callback, this, std::placeholders::_1));

		auto_torque_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::AutoTorqueCtrlCmd>(
				"auto_torque_ctrl_cmd",
				1,
				std::bind(&CanControl::auto_torque_ctrl_cmd_callback, this, std::placeholders::_1));
				
		auto_spd_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::AutoSpdCtrlCmd>(
				"auto_spd_ctrl_cmd",
				1,
				std::bind(&CanControl::auto_spd_ctrl_cmd_callback, this, std::placeholders::_1));

		imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
				"imu_data",
				5,
				std::bind(&CanControl::imu_data_callback, this, std::placeholders::_1));

		chassis_info_fb_publisher_ = node_->create_publisher<yhs_can_interfaces::msg::ChassisInfoFb>("chassis_info_fb", 1);

		odo_fb_pub_ = node_->create_publisher<yhs_can_interfaces::msg::OdoFb>("odo_fb", 1);

		odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
		io_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CanControl::io_timer_callback, this));
	}

	void CanControl::io_cmd_callback(const yhs_can_interfaces::msg::IoCmd::SharedPtr io_cmd_msg)
	{
		std::lock_guard<std::mutex> lock(io_mutex_);
		current_io_cmd_ = *io_cmd_msg;
	}

	void CanControl::imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
	{
		std::lock_guard<std::mutex> lock(imu_mutex_);
		last_imu_time_ = node_->now();

		tf2::Quaternion quaternion;
		quaternion.setX(imu_msg->orientation.x);
		quaternion.setY(imu_msg->orientation.y);
		quaternion.setZ(imu_msg->orientation.z);
		quaternion.setW(imu_msg->orientation.w);
		tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
	}

	void CanControl::io_timer_callback()
	{
		send_io_command();
	}

	void CanControl::send_io_command()
	{
		static unsigned char count = 0;
		yhs_can_interfaces::msg::IoCmd msg;
		{
			std::lock_guard<std::mutex> lock(io_mutex_);
			msg = current_io_cmd_;
		}

		unsigned char sendDataTemp[8] = {0};

		sendDataTemp[0] = msg.io_cmd_enable;

		// 近光灯开关（预留）
		if (msg.io_cmd_lower_beam_headlamp)
			sendDataTemp[1] |= 0x01;

		// 远光灯开关（预留）
		if (msg.io_cmd_upper_beam_headlamp)
			sendDataTemp[1] |= 0x02;

		if (msg.io_cmd_turn_lamp == 0)
			sendDataTemp[1] |= 0x00;
		if (msg.io_cmd_turn_lamp == 1)
			sendDataTemp[1] |= 0x04;
		if (msg.io_cmd_turn_lamp == 2)
			sendDataTemp[1] |= 0x08;

		// 制动灯开关（预留）
		if (msg.io_cmd_braking_lamp)
			sendDataTemp[1] |= 0x10;

		if (msg.io_cmd_clearance_lamp)
			sendDataTemp[1] |= 0x20;

		// 雾灯开关（预留）
		if (msg.io_cmd_fog_lamp)
			sendDataTemp[1] |= 0x40;

		sendDataTemp[2] = msg.io_cmd_speaker;

		sendDataTemp[5] = msg.io_cmd_discharge;

		count++;
		if (count == 16)
			count = 0;

		sendDataTemp[6] = count << 4;

		sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

		can_frame send_frame;

		send_frame.can_id = 0x18C4D7D0 | CAN_EFF_FLAG;
		send_frame.can_dlc = 8;

		memcpy(send_frame.data, sendDataTemp, 8);

		int ret = write(can_socket_, &send_frame, sizeof(send_frame));
		if (ret <= 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
		}
	}

	void CanControl::remote_torque_ctrl_cmd_callback(const yhs_can_interfaces::msg::RemoteTorqueCtrlCmd::SharedPtr remote_Torque_ctrl_cmd_msg)
	{
		yhs_can_interfaces::msg::RemoteTorqueCtrlCmd msg = *remote_Torque_ctrl_cmd_msg;
		const short angular = msg.ctrl_cmd_steering * 100;
		const unsigned char gear = msg.ctrl_cmd_gear;
		const unsigned char brake = msg.ctrl_cmd_brake;

		static unsigned char count = 0;
		unsigned char sendDataTemp[8] = {0};

		sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

		sendDataTemp[0] = sendDataTemp[0] | ((0x03 & msg.ctrl_cmd_acc_running_mode) << 4);

		sendDataTemp[0] = sendDataTemp[0] | ((0x03 & msg.ctrl_cmd_energy_recovery_mode) << 6);

		sendDataTemp[1] = sendDataTemp[1] | msg.ctrl_cmd_acc_pedal_opening;

		sendDataTemp[2] = sendDataTemp[2] | (angular & 0xff);

		sendDataTemp[3] = (angular >> 8) & 0xff;

		sendDataTemp[4] = sendDataTemp[4] | (brake & 0xff);

		count++;

		if (count == 16)
			count = 0;

		sendDataTemp[6] = count << 4;

		sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

		can_frame send_frame;

		send_frame.can_id = 0x18C4D0D0 | CAN_EFF_FLAG;
		send_frame.can_dlc = 8;

		memcpy(send_frame.data, sendDataTemp, 8);

		int ret = write(can_socket_, &send_frame, sizeof(send_frame));
		if (ret <= 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
		}
	}

	void CanControl::auto_torque_ctrl_cmd_callback(const yhs_can_interfaces::msg::AutoTorqueCtrlCmd::SharedPtr auto_Torque_ctrl_cmd_msg)
	{
		yhs_can_interfaces::msg::AutoTorqueCtrlCmd msg = *auto_Torque_ctrl_cmd_msg;
		const short angular = msg.ctrl_cmd_steering * 100;
		const unsigned char gear = msg.ctrl_cmd_gear;
		const unsigned char brake = msg.ctrl_cmd_brake;

		static unsigned char count = 0;
		unsigned char sendDataTemp[8] = {0};

		sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

		sendDataTemp[0] = sendDataTemp[0] | ((0x03 & msg.ctrl_cmd_acc_running_mode) << 4);

		sendDataTemp[0] = sendDataTemp[0] | ((0x03 & msg.ctrl_cmd_energy_recovery_mode) << 6);

		sendDataTemp[1] = sendDataTemp[1] | msg.ctrl_cmd_acc_pedal_opening;

		sendDataTemp[2] = sendDataTemp[2] | (angular & 0xff);

		sendDataTemp[3] = (angular >> 8) & 0xff;

		sendDataTemp[4] = sendDataTemp[4] | (brake & 0xff);

		count++;

		if (count == 16)
			count = 0;

		sendDataTemp[6] = count << 4;

		sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

		can_frame send_frame;

		send_frame.can_id = 0x18C4D1D0 | CAN_EFF_FLAG;
		send_frame.can_dlc = 8;

		memcpy(send_frame.data, sendDataTemp, 8);

		int ret = write(can_socket_, &send_frame, sizeof(send_frame));
		if (ret <= 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
		}
	}

	void CanControl::auto_spd_ctrl_cmd_callback(const yhs_can_interfaces::msg::AutoSpdCtrlCmd::SharedPtr auto_spd_ctrl_cmd_msg)
	{
		yhs_can_interfaces::msg::AutoSpdCtrlCmd msg = *auto_spd_ctrl_cmd_msg;
		const unsigned short vel = msg.ctrl_cmd_velocity * 1000;
		const short angular = msg.ctrl_cmd_steering * 100;
		const unsigned char gear = msg.ctrl_cmd_gear;
		const unsigned char brake = msg.ctrl_cmd_brake;

		static unsigned char count = 0;
		unsigned char sendDataTemp[8] = {0};

		if (msg.ctrl_cmd_velocity < 0)
			return;

		sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

		sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((vel & 0x0f) << 4));

		sendDataTemp[1] = (vel >> 4) & 0xff;

		sendDataTemp[2] = sendDataTemp[2] | (0x0f & (vel >> 12));

		sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

		sendDataTemp[3] = (angular >> 4) & 0xff;

		sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((brake & 0x0f) << 4));

		sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

		sendDataTemp[5] = (brake >> 4) & 0x0f;

		count++;

		if (count == 16)
			count = 0;

		sendDataTemp[6] = count << 4;

		sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

		can_frame send_frame;

		send_frame.can_id = 0x18C4D2D0 | CAN_EFF_FLAG;
		send_frame.can_dlc = 8;

		memcpy(send_frame.data, sendDataTemp, 8);

		int ret = write(can_socket_, &send_frame, sizeof(send_frame));
		if (ret <= 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
		}
	}

	bool CanControl::wait_for_can_frame()
	{
		struct timeval tv;
		fd_set rdfs;
		FD_ZERO(&rdfs);
		FD_SET(can_socket_, &rdfs);
		tv.tv_sec = 0;
		tv.tv_usec = 30000; // 15ms

		int ret = select(can_socket_ + 1, &rdfs, NULL, NULL, &tv);
		if (ret == -1)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Error waiting for CAN frame: " << std::strerror(errno));
			return false;
		}
		else if (ret == 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Timeout waiting for CAN frame! Please check whether the can0 setting is correct,\
whether the can line is connected correctly, and whether the chassis is powered on.");
			return false;
		}
		else
		{
			return true;
		}
		return false;
	}

	void CanControl::can_data_recv_callback()
	{

		can_frame recv_frame;
		yhs_can_interfaces::msg::ChassisInfoFb chassis_info_msg;

		while (rclcpp::ok())
		{
			if (!wait_for_can_frame())
				continue;

			if (read(can_socket_, &recv_frame, sizeof(recv_frame)) >= 0)
			{
				switch (recv_frame.can_id)
				{
				case 0x18C4D2EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::CtrlFb msg;

					msg.ctrl_fb_gear = 0x0f & recv_frame.data[0];

					msg.ctrl_fb_velocity = static_cast<float>(static_cast<unsigned int>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 | (recv_frame.data[0] & 0xf0) >> 4)) / 1000;

					msg.ctrl_fb_steering = static_cast<float>(static_cast<short>((recv_frame.data[4] & 0x0f) << 12 | recv_frame.data[3] << 4 | (recv_frame.data[2] & 0xf0) >> 4)) / 100;

					msg.ctrl_fb_brake = (recv_frame.data[4] & 0xf0) >> 4 | (recv_frame.data[5] & 0x0f) << 4;

					msg.ctrl_fb_mode = (recv_frame.data[5] & 0x30) >> 4;

					msg.ctrl_fb_acc_running_mode = recv_frame.data[6] & 0x30;

					msg.ctrl_fb_energy_recovery_mode = (recv_frame.data[6] & 0x0c) >> 2;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.header.stamp = node_->get_clock()->now();
						chassis_info_msg.ctrl_fb = msg;
						chassis_info_fb_publisher_->publish(chassis_info_msg);
						if (msg.ctrl_fb_gear == 2)
							msg.ctrl_fb_velocity = -msg.ctrl_fb_velocity;
						publish_odom(msg.ctrl_fb_velocity, msg.ctrl_fb_steering / 180 * 3.1415);
					}

					break;
				}

				case 0x18C4D7EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::LrWheelFb msg;

					msg.lr_wheel_fb_velocity = static_cast<float>(static_cast<float>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

					msg.lr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.lr_wheel_fb = msg;
					}

					break;
				}

				case 0x18C4D8EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::RrWheelFb msg;

					msg.rr_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

					msg.rr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.rr_wheel_fb = msg;
					}

					break;
				}

				case 0x18C4DAEF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::IoFb msg;

					msg.io_fb_enable = (recv_frame.data[0] & 0x01) != 0;
					
					// 近光灯开关状态反馈（预留）
					msg.io_fb_lower_beam_headlamp = (recv_frame.data[1] & 0x01) != 0;
					
					// 远光灯开关状态反馈（预留）
					msg.io_fb_upper_beam_headlamp = (recv_frame.data[1] & 0x02) != 0;

					msg.io_fb_turn_lamp = (0x0c & recv_frame.data[1]) >> 2;

					msg.io_fb_braking_lamp = (0x10 & recv_frame.data[1]) != 0;
					msg.io_fb_clearance_lamp = (0x20 & recv_frame.data[1]) != 0;
					
					// 雾灯开关状态反馈（预留）
					msg.io_fb_fog_lamp = (0x40 & recv_frame.data[1]) != 0;
					
					msg.io_fb_speaker = (0x01 & recv_frame.data[2]) != 0;
					
					// 前左防撞条开关状态反馈(预留)
					msg.io_fb_fl_impact_sensor = (0x01 & recv_frame.data[3]) != 0;
					
					msg.io_fb_fm_impact_sensor = (0x02 & recv_frame.data[3]) != 0;
					
					// 前右防撞条开关状态反馈（预留）
					msg.io_fb_fr_impact_sensor = (0x04 & recv_frame.data[3]) != 0;
					
					// 后左防撞条开关状态反馈（预留）
					msg.io_fb_rl_impact_sensor = (0x08 & recv_frame.data[3]) != 0;
					
					msg.io_fb_rm_impact_sensor = (0x10 & recv_frame.data[3]) != 0;
					
					// 后右防撞条开关状态反馈（预留）
					msg.io_fb_rr_impact_sensor = (0x20 & recv_frame.data[3]) != 0;
					
					// 前左跌落传感器状态反馈（预留）
					msg.io_fb_fl_drop_sensor = (0x01 & recv_frame.data[4]) != 0;
					// 前中跌落传感器状态反馈（预留）
					msg.io_fb_fm_drop_sensor = (0x02 & recv_frame.data[4]) != 0;
					// 前右跌落传感器状态反馈（预留）
					msg.io_fb_fr_drop_sensor = (0x04 & recv_frame.data[4]) != 0;
					// 后左跌落传感器状态反馈（预留）
					msg.io_fb_rl_drop_sensor = (0x08 & recv_frame.data[4]) != 0;
					// 后中跌落传感器状态反馈（预留）
					msg.io_fb_rm_drop_sensor = (0x10 & recv_frame.data[4]) != 0;
					// 后右跌落传感器状态反馈（预留）
					msg.io_fb_rr_drop_sensor = (0x20 & recv_frame.data[4]) != 0;
     
					msg.io_fb_discharge = 0x01 & recv_frame.data[5];
      		        msg.io_fb_chargeen = 0x02 & recv_frame.data[5];

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.io_fb = msg;
					}

					break;
				}

				// 里程计反馈
          		case 0x18C4DEEF | CAN_EFF_FLAG:
          		{
            		yhs_can_interfaces::msg::OdoFb msg;
            		msg.odo_fb_accumulative_mileage = static_cast<float>(static_cast<int>(recv_frame.data[3] << 24 | recv_frame.data[2] << 16 | recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

            		// 累计角度（预留）
					msg.odo_fb_accumulative_angular = static_cast<float>(static_cast<int>(recv_frame.data[7] << 24 | recv_frame.data[6] << 16 | recv_frame.data[5] << 8 | recv_frame.data[4])) / 1000;

            		odo_fb_pub_->publish(msg);

            		break;
          		}
				
				case 0x18C4E1EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::BmsInfoFb msg;

					msg.bms_info_voltage = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

					msg.bms_info_current = static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

					msg.bms_info_remaining_capacity = static_cast<float>(static_cast<unsigned short>(recv_frame.data[5] << 8 | recv_frame.data[4])) / 100;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.bms_info_fb = msg;
					}

					break;
				}

				case 0x18C4E2EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::BmsFlagInfoFb msg;

					msg.bms_flag_info_soc = recv_frame.data[0];

					msg.bms_flag_info_single_ov = (recv_frame.data[1] & 0x01) != 0;
					msg.bms_flag_info_single_uv = (recv_frame.data[1] & 0x02) != 0;
					msg.bms_flag_info_ov = (recv_frame.data[1] & 0x04) != 0;
					msg.bms_flag_info_uv = (recv_frame.data[1] & 0x08) != 0;
					msg.bms_flag_info_charge_ot = (recv_frame.data[1] & 0x10) != 0;
					msg.bms_flag_info_charge_ut = (recv_frame.data[1] & 0x20) != 0;
					msg.bms_flag_info_discharge_ot = (recv_frame.data[1] & 0x40) != 0;
					msg.bms_flag_info_discharge_ut = (recv_frame.data[1] & 0x80) != 0;

					msg.bms_flag_info_charge_oc = (recv_frame.data[2] & 0x01) != 0;
					msg.bms_flag_info_discharge_oc = (recv_frame.data[2] & 0x02) != 0;
					msg.bms_flag_info_short = (recv_frame.data[2] & 0x04) != 0;
					msg.bms_flag_info_ic_error = (recv_frame.data[2] & 0x08) != 0;
					msg.bms_flag_info_lock_mos = (recv_frame.data[2] & 0x10) != 0;
					msg.bms_flag_info_charge_state = (recv_frame.data[2] & 0x60) >> 5;
					
					// 预留
					msg.reserved = ((recv_frame.data[2] & 0x80) >> 7 ) | ( (recv_frame.data[3] & 0x0f) << 1) ;

					msg.bms_flag_info_hight_temperature = static_cast<float>(static_cast<short>(recv_frame.data[4] << 4 | recv_frame.data[3] >> 4)) / 10;
					msg.bms_flag_info_low_temperature = static_cast<float>(static_cast<short>((recv_frame.data[6] & 0x0f) << 8 | recv_frame.data[5])) / 10;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.bms_flag_info_fb = msg;
					}

					break;
				}

				case 0x18C4DCEF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::DriveMcuEcodeFb msg;

					msg.drive_fb_mcuecode = static_cast<int>(recv_frame.data[3] << 24 | recv_frame.data[2] << 16 | recv_frame.data[1] << 8 | recv_frame.data[0]);

					int16_t torque_raw = static_cast<int16_t>(recv_frame.data[5] << 8 | recv_frame.data[4]);
					msg.drive_fb_mcutorque = static_cast<int>(torque_raw);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.drive_mcu_ecode_fb = msg;
					}

					break;
				}

				case 0x18C4EAEF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::VehDiagFb msg;

					msg.veh_fb_fault_level = 0x0f & recv_frame.data[0];

					msg.veh_fb_auto_spd_can_ctrl_cmd = (recv_frame.data[0] & 0x10) != 0;
					msg.veh_fb_auto_io_can_cmd = (recv_frame.data[0] & 0x20) != 0;
					msg.veh_fb_remote_trq_can_ctrl_cmd = (0x40 & recv_frame.data[0]) != 0;
					msg.veh_fb_auto_trq_can_ctrl_cmd = (0x80 & recv_frame.data[0]) != 0;
					
					msg.veh_fb_eps_dis_on_line = (recv_frame.data[1] & 0x01) != 0;			
					msg.veh_fb_eps_fault = (recv_frame.data[1] & 0x02) != 0;
					msg.veh_fb_eps_mosf_et_ot = (recv_frame.data[1] & 0x04) != 0;
					msg.veh_fb_eps_warning = (recv_frame.data[1] & 0x08) != 0;
					msg.veh_fb_eps_dis_work = (recv_frame.data[1] & 0x10) != 0;
					msg.veh_fb_eps_over_current = (recv_frame.data[1] & 0x20) != 0;
					
					msg.veh_fb_ehb_mcu_fault_fb = ((recv_frame.data[3] << 4) | ((recv_frame.data[2] & 0xf0) >> 4));
					msg.veh_fb_drv_mcu_fault_fb = ((recv_frame.data[5] & 0x0f) << 8) | (recv_frame.data[4]);
					
					msg.veh_fb_aux_bms_dis_on_line = (recv_frame.data[5] & 0x10) != 0;
					msg.veh_fb_aux_scram = recv_frame.data[5] & 0x20;
					msg.veh_fb_aux_remote_close = recv_frame.data[5] & 0x40;
					msg.veh_fb_aux_remote_dis_on_line = (recv_frame.data[5] & 0x80) != 0;
					// 辅件故障预留
					msg.veh_fb_aux_reserve = recv_frame.data[6] & 0x0f;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.veh_diag_fb = msg;
					}

					break;
				}

				static unsigned short ultra_data[8] = {0};
				case 0x18C4E8EF | CAN_EFF_FLAG:
				{
					ultra_data[0] = (unsigned short)((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]);
					ultra_data[1] = (unsigned short)(recv_frame.data[2] << 4 | ((recv_frame.data[1] & 0xf0) >> 4));

					ultra_data[2] = (unsigned short)((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]);
					ultra_data[3] = (unsigned short)(recv_frame.data[5] << 4 | ((recv_frame.data[4] & 0xf0) >> 4));
					break;
				}

				case 0x18C4E9EF | CAN_EFF_FLAG:
				{
					ultra_data[4] = (unsigned short)((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]);
					ultra_data[5] = (unsigned short)(recv_frame.data[2] << 4 | ((recv_frame.data[1] & 0xf0) >> 4));

					ultra_data[6] = (unsigned short)((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]);
					ultra_data[7] = (unsigned short)(recv_frame.data[5] << 4 | ((recv_frame.data[4] & 0xf0) >> 4));

					yhs_can_interfaces::msg::Ultrasonic ultra_msg;

          			ultra_msg.ultrasonic_fb_01 = ultra_data[ultrasonic_number_[0]];
          			ultra_msg.ultrasonic_fb_02 = ultra_data[ultrasonic_number_[1]];
          			ultra_msg.ultrasonic_fb_03 = ultra_data[ultrasonic_number_[2]];
          			ultra_msg.ultrasonic_fb_04 = ultra_data[ultrasonic_number_[3]];

          			ultra_msg.ultrasonic_fb_05 = ultra_data[ultrasonic_number_[4]];
          			ultra_msg.ultrasonic_fb_06 = ultra_data[ultrasonic_number_[5]];
          			ultra_msg.ultrasonic_fb_07 = ultra_data[ultrasonic_number_[6]];
          			ultra_msg.ultrasonic_fb_08 = ultra_data[ultrasonic_number_[7]];

					chassis_info_msg.ultrasonic = ultra_msg;

					break;
				}

				case 0x18FFFFFB | CAN_EFF_FLAG:
                {
                    chassis_info_msg.ebox_state_fb.diag_ebox_chg_mos_h_tem_limt   = (recv_frame.data[0] & 0x01) != 0;
                    chassis_info_msg.ebox_state_fb.diag_ebox_chg_mos_h_tem_close  = (recv_frame.data[0] & 0x02) != 0;
                    chassis_info_msg.ebox_state_fb.diag_ebox_ebox_mos_h_tem_limt  = (recv_frame.data[0] & 0x04) != 0;
                    chassis_info_msg.ebox_state_fb.diag_ebox_ebox_mos_h_tem_close = (recv_frame.data[0] & 0x08) != 0;
                    
                    chassis_info_msg.ebox_state_fb.diag_ebox_h_light_fault        = (recv_frame.data[1] & 0x01) != 0;
                    chassis_info_msg.ebox_state_fb.diag_ebox_b_light_fault        = (recv_frame.data[1] & 0x02) != 0;
                    chassis_info_msg.ebox_state_fb.diag_ebox_l_light_fault        = (recv_frame.data[1] & 0x04) != 0;
                    chassis_info_msg.ebox_state_fb.diag_ebox_r_light_fault        = (recv_frame.data[1] & 0x08) != 0;
                    break;
                }

                case 0x18FFFFFC | CAN_EFF_FLAG:
                {
                    uint64_t raw_data = 0;
                    for (int i = 0; i < 8; i++) {
                        raw_data |= (static_cast<uint64_t>(recv_frame.data[i]) << (i * 8));
                    }

                    chassis_info_msg.ebox_state_fb.mcu_48v_voltage = ((raw_data >> 0) & 0x3FF) * 0.1f;

                    int16_t mcu_c_signed = static_cast<int16_t>((raw_data >> 10) & 0xFFF);
                    mcu_c_signed = (mcu_c_signed << 4) >> 4;
                    chassis_info_msg.ebox_state_fb.mcu_current = mcu_c_signed * 0.1f;

                    chassis_info_msg.ebox_state_fb.ebox_48v_voltage = ((raw_data >> 22) & 0x3FF) * 0.1f;

                    int16_t ebox_c_signed = static_cast<int16_t>((raw_data >> 32) & 0xFFF);
                    ebox_c_signed = (ebox_c_signed << 4) >> 4;
                    chassis_info_msg.ebox_state_fb.ebox_current = ebox_c_signed * 0.1f;

                    chassis_info_msg.ebox_state_fb.cus_48v_voltage = ((raw_data >> 44) & 0x3FF) * 0.1f;

                    int16_t cus_48c_signed = static_cast<int16_t>((raw_data >> 54) & 0xFFF);
                    cus_48c_signed = (cus_48c_signed << 4) >> 4;
                    chassis_info_msg.ebox_state_fb.cus_48v_current = cus_48c_signed * 0.1f;
                    
                    break;
                }

                case 0x18FFFFFD | CAN_EFF_FLAG:
                {
                    uint64_t raw_data = 0;
                    for (int i = 0; i < 8; i++) {
                        raw_data |= (static_cast<uint64_t>(recv_frame.data[i]) << (i * 8));
                    }

                    chassis_info_msg.ebox_state_fb.cus_24v_voltage = ((raw_data >> 0) & 0xFF) * 0.1f;

                    int16_t cus_24c_signed = static_cast<int16_t>((raw_data >> 8) & 0x3FF);
                    cus_24c_signed = (cus_24c_signed << 6) >> 6;
                    chassis_info_msg.ebox_state_fb.cus_24v_current = cus_24c_signed * 0.1f;

                    chassis_info_msg.ebox_state_fb.cus_12v_voltage = ((raw_data >> 18) & 0xFF) * 0.1f;

                    int16_t cus_12c_signed = static_cast<int16_t>((raw_data >> 26) & 0x3FF);
                    cus_12c_signed = (cus_12c_signed << 6) >> 6; 
                    chassis_info_msg.ebox_state_fb.cus_12v_current = cus_12c_signed * 0.1f;

                    chassis_info_msg.ebox_state_fb.bat_48v_voltage = ((raw_data >> 36) & 0x3FF) * 0.1f;

                    chassis_info_msg.ebox_state_fb.ebox_mos_tem = static_cast<float>((raw_data >> 48) & 0xFF) - 50.0f;

                    chassis_info_msg.ebox_state_fb.chg_mos_tem = static_cast<float>((raw_data >> 56) & 0xFF) - 50.0f;

                    break;
                }

				default:
					break;
				}
			}
		}
	}

	void CanControl::publish_odom(const double velocity, const double steering)
	{
		rclcpp::Time current_time = node_->now();
		double dt = (current_time - odom_last_time_).seconds();
		if (dt <= 0.0)
		{
			return;
		}

		const double linear_vel = velocity;
		const double angular_vel = linear_vel * std::tan(steering) / wheel_base_;

		bool is_imu_active = false;
		double imu_roll = 0.0;
		double imu_pitch = 0.0;
		double imu_yaw = 0.0;
		{
			std::lock_guard<std::mutex> lock(imu_mutex_);
			is_imu_active = (current_time - last_imu_time_).seconds() < 0.2;
			imu_roll = imu_roll_;
			imu_pitch = imu_pitch_;
			imu_yaw = imu_yaw_;
		}

		if (is_imu_active)
		{
			odom_theta_ = imu_yaw;
		}
		else
		{
			odom_theta_ += angular_vel * dt;
		}

		odom_x_ += linear_vel * std::cos(odom_theta_) * dt;
		odom_y_ += linear_vel * std::sin(odom_theta_) * dt;

		const double x_mid = odom_x_ + wheel_base_ / 2.0 * std::cos(odom_theta_);
		const double y_mid = odom_y_ + wheel_base_ / 2.0 * std::sin(odom_theta_);

		nav_msgs::msg::Odometry odom_msg;
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = odom_frame_;
		odom_msg.child_frame_id = base_link_frame_;

		geometry_msgs::msg::PoseWithCovariance pose_cov;
		pose_cov.pose.position.x = x_mid;
		pose_cov.pose.position.y = y_mid;
		pose_cov.pose.position.z = 0.0;
		tf2::Quaternion quat;
		if (is_imu_active)
		{
			quat.setRPY(imu_roll, imu_pitch, odom_theta_);
		}
		else
		{
			quat.setRPY(0.0, 0.0, odom_theta_);
		}
		pose_cov.pose.orientation.x = quat.x();
		pose_cov.pose.orientation.y = quat.y();
		pose_cov.pose.orientation.z = quat.z();
		pose_cov.pose.orientation.w = quat.w();
		odom_msg.pose = pose_cov;

		geometry_msgs::msg::TwistWithCovariance twist_cov;
		twist_cov.twist.linear.x = linear_vel;
		twist_cov.twist.linear.y = 0.0;
		twist_cov.twist.linear.z = 0.0;
		twist_cov.twist.angular.x = 0.0;
		twist_cov.twist.angular.y = 0.0;
		twist_cov.twist.angular.z = angular_vel;
		odom_msg.twist = twist_cov;

		odom_msg.pose.covariance[0] = 0.1;
		odom_msg.pose.covariance[7] = 0.1;
		odom_msg.pose.covariance[35] = 0.2;
		odom_msg.pose.covariance[14] = 1e10;
		odom_msg.pose.covariance[21] = 1e10;
		odom_msg.pose.covariance[28] = 1e10;

		if (tf_used_)
		{
			geometry_msgs::msg::TransformStamped odom_tf;
			odom_tf.header.stamp = current_time;
			odom_tf.header.frame_id = odom_frame_;
			odom_tf.child_frame_id = base_link_frame_;
			odom_tf.transform.translation.x = x_mid;
			odom_tf.transform.translation.y = y_mid;
			odom_tf.transform.translation.z = 0.0;
			odom_tf.transform.rotation.x = quat.x();
			odom_tf.transform.rotation.y = quat.y();
			odom_tf.transform.rotation.z = quat.z();
			odom_tf.transform.rotation.w = quat.w();
			tf_broadcaster_->sendTransform(odom_tf);
		}

		odom_pub_->publish(odom_msg);

		odom_last_time_ = current_time;
	}

	CanControl::~CanControl()
	{
	}

	bool CanControl::run()
	{
		can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (can_socket_ < 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to open socket: " << strerror(errno));
			return false;
		}

		struct ifreq ifr;
		strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
		ifr.ifr_name[IFNAMSIZ - 1] = '\0';
		if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to get interface index: " << strerror(errno) << " ==> " << if_name_.c_str());
			return false;
		}

		struct sockaddr_can addr;
		memset(&addr, 0, sizeof(addr));
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to bind socket: " << strerror(errno));
			return false;
		}

		thread_ = std::thread(&CanControl::can_data_recv_callback, this);

		return true;
	}

	void CanControl::stop()
	{
		if (can_socket_ >= 0)
		{
			close(can_socket_);
			can_socket_ = -1;
		}

		if (thread_.joinable())
		{
			thread_.join();
		}
	}
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("yhs_can_control_node");

	yhs::CanControl cancontrol(node);
	if (!cancontrol.run())
	{
		RCLCPP_ERROR(node->get_logger(), "Failed to initialize yhs_can_control_node");
		return 0;
	}

	RCLCPP_INFO(node->get_logger(), "yhs_can_control_node initialized successfully");

	rclcpp::spin(node);

	cancontrol.stop();
	RCLCPP_INFO(node->get_logger(), "yhs_can_control_node stopped");

	rclcpp::shutdown();

	return 0;
}
