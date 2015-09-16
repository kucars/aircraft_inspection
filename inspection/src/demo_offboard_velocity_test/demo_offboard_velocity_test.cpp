/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file demo_offboard_position_Setpoints.cpp
 *
 * Demo for sending offboard position setpoints to mavros to show offboard position control in SITL
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
*/

#include "demo_offboard_velocity_test.h"

#include <platforms/px4_middleware.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
DemoOffboardVelocityTest::DemoOffboardVelocityTest() :
	_n(),
    _vel_sp_pub(_n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10))
{
}


int DemoOffboardVelocityTest::main()
{
	px4::Rate loop_rate(10);

	while (ros::ok()) {
		loop_rate.sleep();
		ros::spinOnce();

        /* Publish example velocity*/
        geometry_msgs::TwistStamped vs;

        vs.twist.linear.x=0.0;
        vs.twist.linear.y=10.0;
        vs.twist.linear.z=50.0;

        vs.twist.angular.x=0.0;
        vs.twist.angular.y=0.0;
        vs.twist.angular.z=0.0;
        _vel_sp_pub.publish(vs);
//        ros::Duration(20).sleep();

//        vs.twist.linear.x=0;
//        vs.twist.linear.y=15;
//        vs.twist.linear.z=0;
//        _vel_sp_pub.publish(vs);

//        ros::Duration(7).sleep();
//        vs.twist.angular.x=0;
//        vs.twist.angular.y=0;
//        vs.twist.angular.z=90;
//        _vel_sp_pub.publish(vs);
	}

	return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_offboard_velocity_test");
    DemoOffboardVelocityTest d;
	return d.main();
}
