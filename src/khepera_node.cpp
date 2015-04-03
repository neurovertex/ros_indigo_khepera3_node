#include <iostream>
#include <unistd.h>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING 1 // screw you boost I never asked for this
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <libplayerc++/playerc++.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace ros;
using namespace PlayerCc;

// Player won't tell me how much that is so I had to set up a constant
#define URG_LX04_SCANNING_FREQ 10
#define CMD_PERSISTENCY 20

static Position2dProxy *pos2d_ptr;
static int seq, last_seq;

void cmd_callback(const geometry_msgs::Twist& cmd) {
	cout << "\nReceived cmd : " << cmd << endl;
	pos2d_ptr->SetSpeed(cmd.linear.x, cmd.linear.y, cmd.angular.z);
	last_seq = seq;
}

int main (int argc, char *argv[]) {
	init(argc, argv, "khepera_node");
	cout << "args : " << argc <<" "<< argv[0] << endl;
	NodeHandle node;
	string name = this_node::getName(), server;
	NodeHandle priv = NodeHandle("~");
	bool urg;
	priv.param("urg", urg, true);
	priv.param("ip", server, string("localhost"));
	if (!urg)
		ROS_INFO("URG reading disabled");

	geometry_msgs::TransformStamped transformStamped, laserKhepera;
	nav_msgs::Odometry odom;
	tf::Quaternion q = tf::createIdentityQuaternion();

	transformStamped.header.frame_id = "odom";
	transformStamped.child_frame_id = "base_link";
	transformStamped.transform.translation.z = 0.0;

	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	odom.pose.pose.position.z = 0;
	odom.twist.twist.angular.x =
		odom.twist.twist.angular.y =
		odom.twist.twist.linear.z = 0;

	laserKhepera.header.frame_id = "base_link";
	laserKhepera.child_frame_id = "laser";
	laserKhepera.transform.translation.x = 0.02;
	laserKhepera.transform.translation.y = 0.0;
	laserKhepera.transform.translation.z = 0.0;

	laserKhepera.transform.rotation.x = q.x();
	laserKhepera.transform.rotation.y = q.y();
	laserKhepera.transform.rotation.z = q.z();
	laserKhepera.transform.rotation.w = q.w();

	cout << "Connecting to server at "<< server << endl;
	seq = 0;
	last_seq = -255;

	try {
		PlayerClient robot(server);
		Position2dProxy pos2d(&robot,0);
		pos2d_ptr = &pos2d;
		Rate rate(10.0);

		LaserProxy *laser = NULL;
		sensor_msgs::LaserScan scan;
		scan.header.frame_id = "laser";

		if (urg) {
			laser = new LaserProxy(&robot,0);
			do {
				robot.Read();
				laser->RequestConfigure();
				rate.sleep();
			} while (!laser->IsValid() && node.ok());
			cout << "Obtained robot informations : " << laser->GetCount() << " readings on the laser" << endl;

			scan.angle_min = laser->GetMinAngle();
			scan.angle_max = -scan.angle_min;
			scan.angle_increment = (scan.angle_max - scan.angle_min) / laser->GetCount();
			float scan_freq = URG_LX04_SCANNING_FREQ;
			scan.scan_time = 1.0/scan_freq;
			scan.time_increment = scan.scan_time / laser->GetCount();
			scan.range_min = laser->GetMinLeft();
			scan.range_max = laser->GetMaxRange();
			scan.ranges.resize(laser->GetCount());
			scan.intensities.resize(0);
		}

		tf2_ros::TransformBroadcaster tfb;
		Publisher scanb = node.advertise<sensor_msgs::LaserScan>("/scan", 10);
		Publisher odomb = node.advertise<nav_msgs::Odometry>("/odom", 10);

		Subscriber cmdsub = node.subscribe("cmd_vel", 1, cmd_callback);

		while (node.ok()) {
			transformStamped.header.stamp =
				laserKhepera.header.stamp =
						scan.header.stamp =
						odom.header.stamp = Time::now();
			transformStamped.header.seq =
				laserKhepera.header.seq =
						scan.header.seq =
						odom.header.seq = seq;
			spinOnce();
			if ((seq-last_seq) == CMD_PERSISTENCY) {
				pos2d.SetSpeed(0, 0, 0);
				cout << "\nResetting speed" << endl;
			}

			try {
				robot.Read();
				odom.pose.pose.position.x = transformStamped.transform.translation.x = pos2d.GetXPos();
				odom.pose.pose.position.y = transformStamped.transform.translation.y = pos2d.GetYPos();

				transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(pos2d.GetYaw());
				odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pos2d.GetYaw());

				odom.twist.twist.linear.x = pos2d.GetXSpeed();
				odom.twist.twist.linear.y = pos2d.GetYSpeed();
				odom.twist.twist.angular.z = pos2d.GetYawSpeed();
//				cout << transformStamped.transform.translation.x <<":"<< transformStamped.transform.translation.y << endl;

				if (urg) {
					scan.range_min = laser->GetMinLeft();
					scan.range_max = laser->GetMaxRange();
					for (int i = 0; i < laser->GetCount(); i ++) {
						if ((*laser)[i] < scan.range_min)
							scan.range_min = scan.ranges[i] = (*laser)[i];
						else if ((*laser)[i] <= scan.range_max)
							scan.ranges[i] = (*laser)[i];
						else
							scan.ranges[i] = (float)NAN;
					}
				}
			} catch (const PlayerError &e) {
				cout << endl << "PlayerError : "<< e << endl;
			}
			printf("\r% 7d (% 7d)\t%3.3f:%3.3f", seq, seq-last_seq, transformStamped.transform.translation.x, transformStamped.transform.translation.y);
			fflush(stdout);
			tfb.sendTransform(transformStamped);
			if (urg) {
				tfb.sendTransform(laserKhepera);
				scanb.publish(scan);
			}
			odomb.publish(odom);
			rate.sleep();
			seq ++;
		}
	} catch (const PlayerError &e) {
		cout << "\nPlayerError : "<< e << endl;
	}
	cout << "Exitting" << endl;
}
