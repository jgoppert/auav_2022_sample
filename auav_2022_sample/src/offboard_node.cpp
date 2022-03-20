#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define PI acos(-1)

mavros_msgs::State current_state;
// geometry_msgs::PoseStamped current_position;
geometry_msgs::PoseStamped drone_position;

float current_box_x;
float current_box_y;
float prev_box_y;
float chasedis,chasecon;
int lost;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
/*void getpointfdb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("x: [%f]", msg->pose.position.x);
    ROS_INFO("y: [%f]", msg->pose.position.y);
    ROS_INFO("z: [%f]", msg->pose.position.z);
    current_position = *msg;
}*/

typedef struct
{
        double w, x, y, z;
}Quat_t;

typedef struct
{
        double Pitch;
        double Roll;
        double Yaw;
}Euler_t;

int Conversion_Quaternion_to_Euler(geometry_msgs::PoseStamped quat,  Euler_t *euler)
{
        double q0, q1, q2, q3;
        q0 = quat.pose.orientation.w;
        q1 = quat.pose.orientation.x;
        q2 = quat.pose.orientation.y;
        q3 = quat.pose.orientation.z;
        euler->Pitch = (double)(asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3); // pitch
        euler->Roll = (double)(atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3); // roll
        euler->Yaw = (double)(atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3);
        return 0;
}

int Conversion_Euler_to_Quaternion(Quat_t *q1, Euler_t euler_angle)
{
        euler_angle.Yaw = euler_angle.Yaw *  PI / 180;
        euler_angle.Pitch = euler_angle.Pitch * PI / 180;
        euler_angle.Roll = euler_angle.Roll * PI / 180;
        double c1 = cos(euler_angle.Yaw / 2);
        double s1 = sin(euler_angle.Yaw / 2);
        double c2 = cos(euler_angle.Pitch / 2);
        double s2 = sin(euler_angle.Pitch / 2);
        double c3 = cos(euler_angle.Roll / 2);
        double s3 = sin(euler_angle.Roll / 2);
        double c1c2 = c1 * c2;
        double s1s2 = s1 * s2;
        q1->w = (c1 * c2 * c3 - s1 * s2 * s3);
        q1->x = (c1 * c2 * s3 + s1 * s2 * c3);
        q1->y = (c1 * s2 * c3 - s1 * c2 * s3);
        q1->z = (s1 * c2 * c3 + c1 * s2 * s3);
        
        return 0;
}

void boxfdb(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
    current_box_x = (msg->quaternion.x + msg->quaternion.z) / 2;
    current_box_y = (msg->quaternion.y + msg->quaternion.w) / 2;
    
}
void droneposfdb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //ROS_INFO("x: [%f]", msg->pose.position.x);
    //ROS_INFO("y: [%f]", msg->pose.position.y);
    //ROS_INFO("z: [%f]", msg->pose.position.z);
    drone_position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
            
    //ros::Subscriber get_point = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, getpointfdb);
    // ros::Subscriber targetIn = nh.subscribe<geometry_msgs::Point>("world_rover_pos", 10, getpointfdb);
    /* Subscribers used to read:
        - Bounding box
        - 
    */
    ros::Subscriber box_sub = nh.subscribe<geometry_msgs::QuaternionStamped>("rover/bounding_box", 10, boxfdb);
    ros::Subscriber drone_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, droneposfdb);

    // Publisher used to set the setpoints
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

    // ROS services used for arming the drone
    // MAKE SURE YOU DO NOT SET ARMING OR VEHICLE MODE, THIS WILL BE HANDLED BY THE PILOT
    // FOR SIMULATION, YOU CAN USE ANOTHER SCRIPT TO DO THIS

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0f);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    Euler_t angtmp;
    Quat_t qngtmp;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.85;
    pose.pose.orientation.w = 0;
    pose.pose.orientation.x = 1;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;    
    
    

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    //Set a flag for starting
    int i = 0;

    while(ros::ok()){
        pose.pose.position.x = drone_position.pose.position.x;
        pose.pose.position.y = drone_position.pose.position.y;
        pose.pose.position.z = 0.85;
        Conversion_Quaternion_to_Euler(drone_position, &angtmp);   
        
        // pose.pose.rotation.x = drone_pos
        // if(i==0){
        //     i++;        // }
        ROS_INFO("Box y: [%f]", current_box_y);
        ROS_INFO("Box x: [%f]", current_box_x);
	if (current_box_x < 0 || current_box_y < 0){
	   angtmp.Yaw = angtmp.Yaw - 2.5;

	}
	else{

          if (current_box_x > 0.6) {
            angtmp.Yaw = angtmp.Yaw - 20;
	    ROS_INFO("right");

          } else if (current_box_x < 0.4) {
            angtmp.Yaw = angtmp.Yaw + 20;
	    ROS_INFO("Left");

          }
        
          if (current_box_y > 0.90) {
	    chasedis = 3*(current_box_y - 0.9);
            pose.pose.position.x -= chasedis*cos(angtmp.Yaw);
	    //if(angtmp.Yaw<0){
	      pose.pose.position.y -= chasedis*sin(angtmp.Yaw);
	    //}
	    //else{
	      //pose.pose.position.y -= 2*sin(angtmp.Yaw);
	    //}
       	     ROS_INFO("backward"); 
            } 
	  else {
            chasedis = 2*(1-current_box_y)/0.9;
            pose.pose.position.x += chasedis*cos(angtmp.Yaw);
	    //if(angtmp.Yaw<0){
	      //pose.pose.position.y -= 2*sin(angtmp.Yaw);
	      //}
	   // else {
	      pose.pose.position.y += chasedis*sin(angtmp.Yaw);
	     // }
	     ROS_INFO("foreward");
            }
	}
        //angtmp.Yaw = angtmp.Yaw + 1;
	Conversion_Euler_to_Quaternion(&qngtmp, angtmp);
	pose.pose.orientation.x = qngtmp.x;
	pose.pose.orientation.y = qngtmp.y;
	pose.pose.orientation.z = qngtmp.z;
	pose.pose.orientation.w = qngtmp.w;
        
        ROS_INFO("orientation.x:[%f]",angtmp.Yaw);
        ROS_INFO("pos.x:[%f]", 0.6*sin(angtmp.Yaw));
	ROS_INFO("pos.y:[%f]",pose.pose.position.y);
	ROS_INFO("pos.z:[%f]",pose.pose.position.z);

	
	// prev_box_y = current_box_y;
        // if (current_box_x < 0.33) {
        //     pose.rotation.
        // }

        /* if((abs(current_position.pose.position.x-pose.pose.position.x)<0.5f)&&(abs(current_position.pose.position.y-pose.pose.position.y)<0.5f)&&(abs(current_position.pose.position.y-pose.pose.position.y)<0.5f))
        {
            //pose.pose.position.x += 5;
            //pose.pose.position.y = 20*sin(pose.pose.position.x/40*PI);
            //pose.pose.position.z = 3;
            // flight_dist*cos(atan(dfy/dfx));
            // pose.pose.position.y -= flight_dist*sin(atan(dfy/dfx));
            pose.pose.position.z = 3;
        } */

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

