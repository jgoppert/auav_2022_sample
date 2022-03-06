#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>

#define PI acos(-1)

mavros_msgs::State current_state;
geometry_msgs::PoseStamped target_position;
geometry_msgs::PoseStamped current_position;
visualization_msgs::MarkerArray goal_position_set;
std::vector<visualization_msgs::Marker> marker_arr;
visualization_msgs::Marker marker_cur;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void getpointdr(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("drone x: [%f]", msg->pose.position.x);
    ROS_INFO("drone y: [%f]", msg->pose.position.y);
    ROS_INFO("drone z: [%f]", msg->pose.position.z);
    current_position = *msg;
}

void getpointfdb(const geometry_msgs::Point::ConstPtr& msg){
    ROS_INFO("targ x: [%f]", msg->x);
    ROS_INFO("targ y: [%f]", msg->y);
    ROS_INFO("targ z: [%f]", msg->z);
    target_position.pose.position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    marker_arr.push_back(marker_cur);
        

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
            
    ros::Subscriber get_point = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, getpointdr);
    
    ros::Subscriber targetIn = nh.subscribe<geometry_msgs::Point>("world_rover_pos", 10, getpointfdb);
        
    ros::Publisher goal_pos_pub = nh.advertise<visualization_msgs::MarkerArray>("input/goal_position", 10);
    
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;
    pose.pose.orientation.w = 0;
    pose.pose.orientation.x = -1;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;    
    
    

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(pose);
        
        ROS_INFO("At least got this far");
        ROS_INFO("At least got this far");
        
        goal_position_set.markers = marker_arr;
        ROS_INFO("At least got this far");
        marker_arr[0].pose = pose.pose;
        ROS_INFO("At least got this far");
        goal_position_set.markers[0].pose = pose.pose;
       
        ROS_INFO("Got this far");
        goal_pos_pub.publish(goal_position_set);
        ROS_INFO("Didn't get this far");

        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();

    //Set a flag for starting
    int i = 0;

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

	if(i==0){
	    pose.pose.position.x = current_position.pose.position.x;
        pose.pose.position.y = current_position.pose.position.y;
	    pose.pose.position.z = 3;
	    i++;
	}
	
	float dfx = target_position.pose.position.x-current_position.pose.position.x;
	float dfy = target_position.pose.position.y-current_position.pose.position.y;
        ROS_INFO("dfx: [%f]", dfx);
        ROS_INFO("dfy: [%f]", dfy);
        
        target_position.pose.position.x = target_position.pose.position.x - 5*cos(atan(dfy/dfx));
        target_position.pose.position.y = target_position.pose.position.y - 5*sin(atan(dfy/dfx));
        
	    if((abs(current_position.pose.position.x-pose.pose.position.x)<0.5f)&&(abs(current_position.pose.position.y-pose.pose.position.y)<0.5f)&&(abs(current_position.pose.position.y-pose.pose.position.y)<0.5f))
        {
            //pose.pose.position.x += 5;
            //pose.pose.position.y = 20*sin(pose.pose.position.x/40*PI);
            //pose.pose.position.z = 3;
            pose.pose.position.x = 0; // target_position.pose.position.x;
            pose.pose.position.y = 0; // target_position.pose.position.y;
            pose.pose.position.z = 3;
        }
        ROS_INFO("pos x: [%f]", pose.pose.position.x);
        ROS_INFO("pos y: [%f]", pose.pose.position.y);
        ROS_INFO("pos z: [%f]", pose.pose.position.z);
        
        // local_pos_pub.publish(pose);
        ROS_INFO("At least got this far");
        
        goal_position_set.markers = marker_arr;
        goal_position_set.markers[0].pose = pose.pose;
        
        goal_pos_pub.publish(goal_position_set);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
