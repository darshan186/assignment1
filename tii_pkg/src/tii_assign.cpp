#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <vector>
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>

// Script used to control manual pose of drone inside simulator
using namespace std;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
bool ofbFlag = true;

tf2::Quaternion myQuaternion;
int setPoint = 0;

//Set-points
vector<vector<float> > vect = { {0, 0, 1.5, 0}, {2, 1, 1.5, 45}, {0, 2, 1.5, 225}};      //{x, y, z, yaw_in_degrees}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void set_point_call()
{
    myQuaternion.setRPY(0,0,vect[0][3]);      //Converting euler to Quaternion

    cout<<"Moving to set_point "<<setPoint<<": [x= "<< vect[0][0] <<", y= "<<vect[0][1] <<", z= "<<vect[0][2] << ", yaw= "<<vect[0][3]<<"]"<<endl;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = vect[0][0];
    pose.pose.position.y = vect[0][1];
    pose.pose.position.z = vect[0][2];
    pose.pose.orientation.z = myQuaternion.getZ();
    pose.pose.orientation.w = myQuaternion.getW();
}


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "geometry_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 2, state_cb);  

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    ros::Rate rate(20.0);  //Main loop Hz

    ros::Rate rate_1(3.0);  //Setpoint publishing Hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();          //allows to run loop, return immidiately once any one codition become false
        rate.sleep();             //maintaines sleep with defined Hz
    }

    //Set OFFBOARD mode param
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //Set LAND mode param
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    //set arm service
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        int count = 0;
        // cout<<"State: "<<current_state.mode<<", vetor_size: "<<vect.size()<<", FLag: "<<ofbFlag<<endl;
        if ((current_state.mode == "AUTO.LAND" || current_state.mode == "AUTO.LOITER" || current_state.mode == "AUTO.RTL") && vect.size() > 0 && ofbFlag)
        {
            set_mode_client.call(offb_set_mode);   //Calling offboard mode
        }

        if(offb_set_mode.response.mode_sent && vect.size() > 0 && (ros::Time::now() - last_request > ros::Duration(5.0)))     //At every 5sec once get into body
        {
            last_request = ros::Time::now();
            ROS_INFO("Set to OFFBOARD mode");

                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                        // sleep(2); //wait 1sec to complete arm action
                        
                        set_point_call();
                        while(ros::ok() && count < 20)
                        {
                            local_pos_pub.publish(pose);
                            rate_1.sleep();
                            count = count + 1;
                        }
                        setPoint++;
                        ofbFlag = false;       //Keep publishing offboard mode untill achieve 1st set_point 
                    }
                
                else
                {
                    vect.erase(vect.begin());                      //Erase first elemet from vector
                    if (vect.size() > 0)
                        {
                            set_point_call();
                            while(ros::ok() && count < 20)
                            {
                                local_pos_pub.publish(pose);
                                rate_1.sleep();
                                count = count + 1;
                            }
                            setPoint++;
                        }

                    else
                    {
                        //Set last point to avoid hard landing(Set to 0.2m in realtime)
                        pose.pose.position.z = 0.00;
                        ROS_INFO("Going to LAND!");
                        while(ros::ok() && count < 5)
                        {
                            local_pos_pub.publish(pose);
                            rate_1.sleep();
                            count = count + 1;
                        }
                        set_mode_client.call(land_set_mode);
                        // ofbFlag = false;

                        break;          //Breaking the ROS framework connection
                    }
                }       
        }

        //To keep hold pose
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Control back to the simulated autopilot and exit");
    return 0;
}



