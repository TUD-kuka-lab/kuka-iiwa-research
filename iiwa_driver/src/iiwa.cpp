//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
//|              Bernardo Fichera
//|              Walid Amanhoud
//|    email:    costashatz@gmail.com
//|              bernardo.fichera@epfl.ch
//|              walid.amanhoud@epfl.ch
//|    Other contributors:
//|              Yoan Mollard (yoan@aubrune.eu)
//|    website:  lasa.epfl.ch
//|
//|    This file is part of iiwa_ros.
//|
//|    iiwa_ros is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    iiwa_ros is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
#include <iiwa_driver/iiwa.h>

// ROS Headers
#include <control_toolbox/filters.h>
#include <controller_manager/controller_manager.h>

#include <urdf/model.h>

// FRI Headers
#include <kuka/fri/ClientData.h>

#include <thread>

namespace iiwa_ros {
    Iiwa::Iiwa(ros::NodeHandle& nh)
    {
        init(nh);
    }

    Iiwa::~Iiwa()
    {
        // Disconnect from robot
        _disconnect_fri();

        // Delete FRI message data
        if (_fri_message_data)
            delete _fri_message_data;
    }

    void Iiwa::init(ros::NodeHandle& nh)
    {
        _nh = nh;
        _load_params(); // load parameters
        _init(); // initialize
        _commanding_status_pub = _nh.advertise<std_msgs::Bool>("commanding_status", 100);
        _controller_manager.reset(new controller_manager::ControllerManager(this, _nh));

        if (_init_fri())
            _initialized = true;
        else
            _initialized = false;
    }

    void Iiwa::run()
    {
        if (!_initialized) {
            ROS_ERROR_STREAM("Not connected to the robot. Cannot run!");
            return;
        }

        std::thread t1(&Iiwa::_ctrl_loop, this);
        t1.join();
    }

    bool Iiwa::initialized()
    {
        return _initialized;
    }

    void Iiwa::_init()
    {
        // Get joint names
        _num_joints = _joint_names.size();

        // Resize vectors
        _joint_position.resize(_num_joints);
        _joint_velocity.resize(_num_joints);
        _joint_effort.resize(_num_joints);
        _joint_position_command.resize(_num_joints);
        _joint_velocity_command.resize(_num_joints);
        _joint_effort_command.resize(_num_joints);

        // Get the URDF XML from the parameter server
        urdf::Model urdf_model;
        std::string urdf_string;

        // search and wait for robot_description on param server
        while (urdf_string.empty()) {
            ROS_INFO_ONCE_NAMED("Iiwa", "Iiwa is waiting for model"
                                        " URDF in parameter [%s] on the ROS param server.",
                _robot_description.c_str());

            _nh.getParam(_robot_description, urdf_string);

            usleep(100000);
        }
        ROS_INFO_STREAM_NAMED("Iiwa", "Received urdf from param server, parsing...");

        const urdf::Model* const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : nullptr;
        if (urdf_model_ptr == nullptr)
            ROS_WARN_STREAM_NAMED("Iiwa", "Could not read URDF from '" << _robot_description << "' parameters. Joint limits will not work.");

        // Initialize Controller
        for (int i = 0; i < _num_joints; ++i) {
            _joint_position[i] = _joint_velocity[i] = _joint_effort[i] = 0.;
            // Create joint state interface
            hardware_interface::JointStateHandle joint_state_handle(_joint_names[i], &_joint_position[i], &_joint_velocity[i], &_joint_effort[i]);
            _joint_state_interface.registerHandle(joint_state_handle);

            // Get joint limits from URDF
            bool has_soft_limits = false;
            bool has_limits = urdf_model_ptr != nullptr;
            joint_limits_interface::JointLimits limits;
            joint_limits_interface::SoftJointLimits soft_limits;

            if (has_limits) {
                auto urdf_joint = urdf_model_ptr->getJoint(_joint_names[i]);
                if (!urdf_joint) {
                    ROS_WARN_STREAM_NAMED("Iiwa", "Could not find joint '" << _joint_names[i] << "' in URDF. No limits will be applied for this joint.");
                    continue;
                }

                getJointLimits(urdf_joint, limits);
                if (getSoftJointLimits(urdf_joint, soft_limits))
                    has_soft_limits = true;
            }

            // Create position joint interface
            hardware_interface::JointHandle joint_position_handle(joint_state_handle, &_joint_position_command[i]);

            if (has_soft_limits) {
                joint_limits_interface::PositionJointSoftLimitsHandle joint_limits_handle(joint_position_handle, limits, soft_limits);
                _position_joint_limits_interface.registerHandle(joint_limits_handle);
            }
            else {
                joint_limits_interface::PositionJointSaturationHandle joint_limits_handle(joint_position_handle, limits);
                _position_joint_saturation_interface.registerHandle(joint_limits_handle);
            }

            _position_joint_interface.registerHandle(joint_position_handle);

            // Create effort joint interface
            hardware_interface::JointHandle joint_effort_handle(joint_state_handle, &_joint_effort_command[i]);

            if (has_soft_limits) {
                joint_limits_interface::EffortJointSoftLimitsHandle joint_limits_handle(joint_effort_handle, limits, soft_limits);
                _effort_joint_limits_interface.registerHandle(joint_limits_handle);
            }
            else if (has_limits) {
                joint_limits_interface::EffortJointSaturationHandle joint_limits_handle(joint_effort_handle, limits);
                _effort_joint_saturation_interface.registerHandle(joint_limits_handle);
            }

            _effort_joint_interface.registerHandle(joint_effort_handle);

            // Create velocity joint interface
            hardware_interface::JointHandle joint_velocity_handle(joint_state_handle, &_joint_velocity_command[i]);

            if (has_soft_limits) {
                joint_limits_interface::VelocityJointSoftLimitsHandle joint_limits_handle(joint_velocity_handle, limits, soft_limits);
                _velocity_joint_limits_interface.registerHandle(joint_limits_handle);
            }
            else {
                joint_limits_interface::VelocityJointSaturationHandle joint_limits_handle(joint_velocity_handle, limits);
                _velocity_joint_saturation_interface.registerHandle(joint_limits_handle);
            }

            _velocity_joint_interface.registerHandle(joint_velocity_handle);
        }

        registerInterface(&_joint_state_interface);
        registerInterface(&_position_joint_interface);
        registerInterface(&_effort_joint_interface);
        registerInterface(&_velocity_joint_interface);

        _additional_pub.init(_nh, "additional_outputs", 20);
        _additional_pub.msg_.external_torques.layout.dim.resize(1);
        _additional_pub.msg_.external_torques.layout.data_offset = 0;
        _additional_pub.msg_.external_torques.layout.dim[0].size = _num_joints;
        _additional_pub.msg_.external_torques.layout.dim[0].stride = 0;
        _additional_pub.msg_.external_torques.data.resize(_num_joints);
        _additional_pub.msg_.commanded_torques.layout.dim.resize(1);
        _additional_pub.msg_.commanded_torques.layout.data_offset = 0;
        _additional_pub.msg_.commanded_torques.layout.dim[0].size = _num_joints;
        _additional_pub.msg_.commanded_torques.layout.dim[0].stride = 0;
        _additional_pub.msg_.commanded_torques.data.resize(_num_joints);
        _additional_pub.msg_.commanded_positions.layout.dim.resize(1);
        _additional_pub.msg_.commanded_positions.layout.data_offset = 0;
        _additional_pub.msg_.commanded_positions.layout.dim[0].size = _num_joints;
        _additional_pub.msg_.commanded_positions.layout.dim[0].stride = 0;
        _additional_pub.msg_.commanded_positions.data.resize(_num_joints);
    }

    void Iiwa::_ctrl_loop()
    {
        static ros::Rate rate(_control_freq);
        while (ros::ok()) {
            ros::Time time = ros::Time::now();

            // TO-DO: Get real elapsed time?
            auto elapsed_time = ros::Duration(1. / _control_freq);

            _read(elapsed_time);
            _controller_manager->update(ros::Time::now(), elapsed_time);
            _write(elapsed_time);

            // publish additional outputs
            if (_additional_pub.trylock()) {
                _additional_pub.msg_.header.stamp = ros::Time::now();
                for (unsigned i = 0; i < _num_joints; i++) {
                    _additional_pub.msg_.external_torques.data[i] = _robot_state.getExternalTorque()[i];
                    _additional_pub.msg_.commanded_torques.data[i] = _robot_state.getCommandedTorque()[i];
                    _additional_pub.msg_.commanded_positions.data[i] = _robot_state.getCommandedJointPosition()[i];
                }
                _additional_pub.unlockAndPublish();
            }

            _publish();
            rate.sleep();
        }
    }

    void Iiwa::_publish()
    {
        std_msgs::Bool msg;
        msg.data = _commanding;
        _commanding_status_pub.publish(msg);
    }

    void Iiwa::_load_params()
    {
        ros::NodeHandle n_p("~");
        
        std::string myns = ros::this_node::getNamespace();
        
        n_p.param(myns+"/fri/port", _port, 30200); // Default port is 30200
        n_p.param<std::string>(myns+"/fri/robot_ip", _remote_host, "192.170.10.2"); // Default robot ip is 192.170.10.2
        n_p.param<std::string>(myns+"/fri/robot_description", _robot_description, "/robot_description");

        n_p.param(myns+"/hardware_interface/control_freq", _control_freq, 1000.);
        n_p.getParam(myns+"/hardware_interface/joints", _joint_names);
    }

    void Iiwa::_read(ros::Duration elapsed_time)
    {
        // Read data from robot (via FRI)
        kuka::fri::ESessionState fri_state;
        _read_fri(fri_state);

        switch (fri_state) {
        case kuka::fri::MONITORING_WAIT:
        case kuka::fri::MONITORING_READY:
        case kuka::fri::COMMANDING_WAIT:
            _idle = false;
            _commanding = false;
            break;
        case kuka::fri::COMMANDING_ACTIVE:
            _idle = false;
            _commanding = true;
            break;
        case kuka::fri::IDLE: // if idle, do nothing
        default:
            _idle = true;
            _commanding = false;
            return;
        }

        // Update ROS structures
        _joint_position_prev = _joint_position;

        for (int i = 0; i < _num_joints; i++) {
            _joint_position[i] = _robot_state.getMeasuredJointPosition()[i];
            _joint_velocity[i] = filters::exponentialSmoothing((_joint_position[i] - _joint_position_prev[i]) / elapsed_time.toSec(), _joint_velocity[i], 0.2);
            _joint_effort[i] = _robot_state.getMeasuredTorque()[i];
        }
    }

    void Iiwa::_write(ros::Duration elapsed_time)
    {
        if (_idle) // if idle, do nothing
            return;

        // enforce limits
        _position_joint_limits_interface.enforceLimits(elapsed_time);
        _position_joint_saturation_interface.enforceLimits(elapsed_time);
        _effort_joint_limits_interface.enforceLimits(elapsed_time);
        _effort_joint_saturation_interface.enforceLimits(elapsed_time);
        _velocity_joint_limits_interface.enforceLimits(elapsed_time);
        _velocity_joint_saturation_interface.enforceLimits(elapsed_time);

        // reset commmand message
        _fri_message_data->resetCommandMessage();

        if (_robot_state.getClientCommandMode() == kuka::fri::TORQUE) {
            // Implements dithering to trigger the friction observer.
            // If the physical robot is at the commanded positions, KUKA turns off friction
            // compensation.
            for (size_t i = 0; i < _num_joints; i++)
            {
                _joint_position_command.at(i) = _joint_position.at(i) + 0.1 * std::sin(ros::Time::now().toSec());
            }
            _robot_command.setJointPosition(_joint_position_command.data());
            _robot_command.setTorque(_joint_effort_command.data());
        }
        else if (_robot_state.getClientCommandMode() == kuka::fri::POSITION)
            _robot_command.setJointPosition(_joint_position_command.data());
        // else ERROR

        _write_fri();
    }

    bool Iiwa::_init_fri()
    {
        _idle = true;
        _commanding = false;

        // Create message/client data
        _fri_message_data = new kuka::fri::ClientData(_robot_state.NUMBER_OF_JOINTS);

        // link monitoring and command message to wrappers
        _robot_state.set_message(&_fri_message_data->monitoringMsg);
        _robot_command.set_message(&_fri_message_data->commandMsg);

        // set specific message IDs
        _fri_message_data->expectedMonitorMsgID = _robot_state.monitoring_message_id();
        _fri_message_data->commandMsg.header.messageIdentifier = _robot_command.command_message_id();

        if (!_connect_fri())
            return false;

        return true;
    }

    bool Iiwa::_connect_fri()
    {
        if (_fri_connection.isOpen()) {
            // TO-DO: Use ROS output
            // printf("Warning: client application already connected!\n");
            return true;
        }
        
        return _fri_connection.open(_port, _remote_host.c_str());
    }

    void Iiwa::_disconnect_fri()
    {
        if (_fri_connection.isOpen())
            _fri_connection.close();
    }

    bool Iiwa::_read_fri(kuka::fri::ESessionState& current_state)
    {
        if (!_fri_connection.isOpen()) {
            // TO-DO: Use ROS output
            // printf("Error: client application is not connected!\n");
            return false;
        }

        // **************************************************************************
        // Receive and decode new monitoring message
        // **************************************************************************
        _message_size = _fri_connection.receive(_fri_message_data->receiveBuffer, kuka::fri::FRI_MONITOR_MSG_MAX_SIZE);

        if (_message_size <= 0) { // TODO: size == 0 -> connection closed (maybe go to IDLE instead of stopping?)
            // TO-DO: Use ROS output
            // printf("Error: failed while trying to receive monitoring message!\n");
            return false;
        }

        if (!_fri_message_data->decoder.decode(_fri_message_data->receiveBuffer, _message_size)) {
            return false;
        }

        // check message type (so that our wrappers match)
        if (_fri_message_data->expectedMonitorMsgID != _fri_message_data->monitoringMsg.header.messageIdentifier) {
            // TO-DO: Use ROS output
            // printf("Error: incompatible IDs for received message (got: %d expected %d)!\n",
            //     (int)_fri_message_data->monitoringMsg.header.messageIdentifier,
            //     (int)_fri_message_data->expectedMonitorMsgID);
            return false;
        }

        current_state = (kuka::fri::ESessionState)_fri_message_data->monitoringMsg.connectionInfo.sessionState;

        if (_fri_message_data->lastState != current_state) {
            _on_fri_state_change(_fri_message_data->lastState, current_state);
            _fri_message_data->lastState = current_state;
        }

        return true;
    }

    bool Iiwa::_write_fri()
    {
        // **************************************************************************
        // Encode and send command message
        // **************************************************************************

        _fri_message_data->lastSendCounter++;
        // check if its time to send an answer
        if (_fri_message_data->lastSendCounter >= _fri_message_data->monitoringMsg.connectionInfo.receiveMultiplier) {
            _fri_message_data->lastSendCounter = 0;

            // set sequence counters
            _fri_message_data->commandMsg.header.sequenceCounter = _fri_message_data->sequenceCounter++;
            _fri_message_data->commandMsg.header.reflectedSequenceCounter = _fri_message_data->monitoringMsg.header.sequenceCounter;

            if (!_fri_message_data->encoder.encode(_fri_message_data->sendBuffer, _message_size)) {
                return false;
            }

            if (!_fri_connection.send(_fri_message_data->sendBuffer, _message_size)) {
                // TO-DO: Use ROS output
                // printf("Error: failed while trying to send command message!\n");
                return false;
            }
        }

        return true;
    }
} // namespace iiwa_ros
