#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define PI acos(-1)

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_position;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
/*void getpointfdb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("x: [%f]", msg->pose.position.x);
    ROS_INFO("y: [%f]", msg->pose.position.y);
    ROS_INFO("z: [%f]", msg->pose.position.z);
    current_position = *msg;
}*/

void getpointfdb(const geometry_msgs::Point::ConstPtr& msg){
    ROS_INFO("x: [%f]", msg->x);
    ROS_INFO("y: [%f]", msg->y);
    ROS_INFO("z: [%f]", msg->z);
    current_position.pose.position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
            
    //ros::Subscriber get_point = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, getpointfdb);
    ros::Subscriber targetIn = nh.subscribe<geometry_msgs::Point>("world_rover_pos", 10, getpointfdb);
        
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
	    ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0f);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
    
    

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        /*if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0f))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0f))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }*/
        
        if((abs(current_position.pose.position.x-pose.pose.position.x)<0.5f)&&(abs(current_position.pose.position.y-pose.pose.position.y)<0.5f)&&(abs(current_position.pose.position.y-pose.pose.position.y)<0.5f))
        {
            //pose.pose.position.x += 5;
            //pose.pose.position.y = 20*sin(pose.pose.position.x/40*PI);
            //pose.pose.position.z = 3;
	    pose.pose.position.x = current_position.pose.position.x;
	    pose.pose.position.y = current_position.pose.position.y;
	    pose.pose.position.z = 3;
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
