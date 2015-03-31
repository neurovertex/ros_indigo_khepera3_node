#include <iostream>
#include <unistd.h>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING 1 // screw you boost I never asked for this
#include <libplayerc++/playerc++.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace PlayerCc;

// Player won't tell me how much that is so I had to set up a constant
#define URG_LX04_SCANNING_FREQ 10

int main (int argc, char *argv[]) {
	ros::init(argc, argv, "khepera_node");
	ros::NodeHandle node;

	geometry_msgs::TransformStamped transformStamped, laserKhepera;
	tf2::Quaternion q;
	q.setRPY(0, 0, 0);

	transformStamped.header.frame_id = "odom";
	transformStamped.child_frame_id = "khepera";
	transformStamped.transform.translation.z = 0.0;

	laserKhepera.header.frame_id = "khepera";
	laserKhepera.child_frame_id = "laser";
	laserKhepera.transform.translation.x = 0.02;
	laserKhepera.transform.translation.y = 0.0;
	laserKhepera.transform.translation.z = 0.10;

	laserKhepera.transform.rotation.x = q.x();
	laserKhepera.transform.rotation.y = q.y();
	laserKhepera.transform.rotation.z = q.z();
	laserKhepera.transform.rotation.w = q.w();

	string server = "localhost";
	if (argc > 1) {
		server = argv[1];
	}
	cout << "Connecting to server at "<< server << endl;
	int seq = 0;

	try {
		PlayerClient robot(server);
		Position2dProxy pos2d(&robot,0);
		LaserProxy laser(&robot,0);
		ros::Rate rate(10.0);
		sensor_msgs::LaserScan scan;
		scan.header.frame_id = "laser";

		do {
			robot.Read();
			laser.RequestConfigure();
			rate.sleep();
		} while (!laser.IsValid() && node.ok());
		cout << "Obtained robot informations : " << laser.GetCount() << " readings on the laser" << endl;

		scan.angle_min = laser.GetMinAngle();
		scan.angle_max = -scan.angle_min;
		scan.angle_increment = (scan.angle_max - scan.angle_min) / laser.GetCount();
		float scan_freq = URG_LX04_SCANNING_FREQ;
		scan.scan_time = 1.0/scan_freq;
		scan.time_increment = scan.scan_time / laser.GetCount();
		scan.range_min = laser.GetMinLeft();
		scan.range_max = laser.GetMaxRange();
		scan.ranges.resize(laser.GetCount());
		scan.intensities.resize(0);

		tf2_ros::TransformBroadcaster tfb;
		ros::Publisher scanb = node.advertise<sensor_msgs::LaserScan>("scan", 10);

		while (node.ok()) {
			transformStamped.header.stamp = ros::Time::now();
			transformStamped.header.seq = seq ++;
			laserKhepera.header.stamp = ros::Time::now();
			laserKhepera.header.seq = seq ++;
			scan.header.stamp = ros::Time::now();
			scan.header.seq = seq ++;
			
			try {
				robot.Read();
				transformStamped.transform.translation.x = pos2d.GetXPos();
				transformStamped.transform.translation.y = pos2d.GetYPos();
				q.setRPY(0, 0, pos2d.GetYaw());
				transformStamped.transform.rotation.x = q.x();
				transformStamped.transform.rotation.y = q.y();
				transformStamped.transform.rotation.z = q.z();
				transformStamped.transform.rotation.w = q.w();
//				cout << transformStamped.transform.translation.x <<":"<< transformStamped.transform.translation.y << endl;

				scan.range_min = laser.GetMinLeft();
				scan.range_max = laser.GetMaxRange();
				for (int i = 0; i < laser.GetCount(); i ++) {
					if (laser[i] <= scan.range_max && laser[i] >= scan.range_min)
						scan.ranges[i] = laser[i];
					else
						scan.ranges[i] = (float)NAN;
				}
			} catch (const PlayerError &e) {
				cout << endl << "PlayerError : "<< e << endl;
			}
			cout << '\r' << seq << '\t' << transformStamped.transform.translation.x <<":"<< transformStamped.transform.translation.y << flush;
			tfb.sendTransform(transformStamped);
			tfb.sendTransform(laserKhepera);
			scanb.publish(scan);
			rate.sleep();
		}
	} catch (const PlayerError &e) {
		cout << endl << "PlayerError : "<< e << endl;
	}
	cout << "Exitting" << endl;
}
