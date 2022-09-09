// Script used to control target pose of drone inside simulator using Manual Joystick overriding concept + P Controller
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/State.h>
#include<mavros_msgs/AttitudeTarget.h>
#include <std_msgs/String.h>
#include <vector>
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


using namespace std;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped current_pose;
mavros_msgs::ManualControl attitude;        //to control thrust & body-rates


float thrust = 550;  //min thrust required to takeoff drone
float roll_rate = 160;  //min bodyrate to move drone along roll axis
float pitch_rate = 160; //min bodyrate to move drone along pitch axis
float yaw_rate = 180;   //min bodyrate rotate along z-axis

// Controller gains (Increase these gains to perform faster)
float Kp_z = 1.5;   
float Kp_c = 1.0;    //constant
float Kp_yaw = 1.5;
float Kp_r = 1.5;
float Kp_p = 1.5;


//Logical statement switching purpose
int Zframes = 0;
int Rframes = 0;
int Pframes = 0;
int Yframes = 0;
    
bool ZFlag = true;
bool RFlag = true;
bool PFlag = true;
bool YFlag = true;


vector<vector<float> > vect = { {1.0, 0.0, 1.0, 0.0}, {3.0, 2.0, 2.0, 1.57}, {0.0, 2.0, 1.0, 0.0}};      //{x, y, z, yaw_in_radian}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;

}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "tii_assign_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 2, state_cb);  

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // ros::Publisher attitude_pos_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher att_pub = nh.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 2, pose_cb);

    ros::Rate rate(20.0);  //Main loop Hz


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

    //Set LAND mode param
    mavros_msgs::SetMode hold_set_mode;
    hold_set_mode.request.custom_mode = "AUTO.LOITER";

    //set arm service
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // Default 
    attitude.z = thrust;
    attitude.buttons = 1;


    while(ros::ok())
    {
        int count = 0;       

        set_mode_client.call(offb_set_mode);
        if (offb_set_mode.response.mode_sent && vect.size() > 0 )
        {
            if (!arm_cmd.response.success)
                arming_client.call(arm_cmd);

            cout<<"---Moving at set_point------ "<<": [x= "<< vect[0][0] <<", y= "<<vect[0][1] <<", z= "<<vect[0][2] << ", yaw= "<<vect[0][3]<<"]"<<endl;

            //Altitide controller
            float Z_error = vect[0][2] - current_pose.pose.position.z;        //calculate linear z error
            if (Z_error > 0.1 && arm_cmd.response.success)
            {
                
                attitude.z = Kp_z * thrust;
                att_pub.publish(attitude);        //Share thrust value in z-axis
                
            }
            else if (Z_error < 0.1)
            {
                attitude.z = Kp_c * thrust;          //Sharing constant thrust to hold z-axis
                att_pub.publish(attitude);
                Zframes++;
                if (Zframes > 25)
                {
                    ZFlag = false;
                    cout<<"Z holding completed"<<endl;
                }


                //yaw rate controller
                double yaw_degree = tf::getYaw(current_pose.pose.orientation);
                float Yaw_error = vect[0][3] - yaw_degree;     //calculate yaw error
                attitude.z = Kp_c * thrust;
                if (Yaw_error > 0.1 )          
                {
                    attitude.r = Kp_yaw * -yaw_rate;             //CW
                    att_pub.publish(attitude);                  //Share yaw bodyrate value in drone body z-axis
                }

                else if (Yaw_error > -0.1 && Yaw_error < 0.1)         // 5 degree tolerence, Hold axis if error comes in threshold range
                {
                    attitude.r = 0;
                    att_pub.publish(attitude);
                    Yframes++;
                    if (Yframes > 25)
                    {
                        YFlag = false;
                        // cout<<"Target yaw : "<<vect[0][3]<<" yaw_degree : "<<yaw_degree<<endl;
                        cout<<"Yaw holding completed"<<endl;
                    }

                }

                else
                {  
                    attitude.r = Kp_yaw * yaw_rate;         //CCW
                    att_pub.publish(attitude);
                    
                }
                

                //Pitch rate controller
                if (!YFlag)
                {
                    // Swaping pitch movement with roll movement
                    if(vect[0][3] > 0.707 && vect[0][3] < 2.277)
                    {
                        double Xa = current_pose.pose.position.x;      
                        float X_error = vect[0][0] - Xa;               //Calculating error
                        if (X_error > 0.1 )
                        {
                            attitude.y = Kp_p * pitch_rate;            //Since swapped, share data in drone body pitching axis
                            att_pub.publish(attitude);               //publish Pcontroller processed pitch rate value in +ve axis[forward]
                        }

                        else if (X_error > -0.1 && X_error < 0.1 )         //Hold axis if error comes in threshold range
                        {
                            attitude.y = 0;                                   
                            att_pub.publish(attitude);
                            Pframes++;
                            if (Pframes > 25)                   //Keep check min 25 frames to declare axis holds successfully
                            {
                                PFlag = false;
                                // cout<<"Target X "<<vect[0][0]<<" Actual x "<<Xa<<endl;
                                cout<<"x-axis + pitching holding completed "<<endl;
                            }
                        }

                        else
                        {
                            attitude.y = Kp_p * -pitch_rate;                   
                            att_pub.publish(attitude);             //publish Pcontroller processed roll rate value in -ve axis[backward]
                        }


                    }

                    
                    else
                    {
                        double Xa = current_pose.pose.position.x;
                        float X_error = vect[0][0] - Xa;

                        if (X_error > 0.1)
                        {
                            attitude.x = Kp_p * pitch_rate;                    //Share body-rate in drone body roll axis               
                            att_pub.publish(attitude);
                        }

                        else if (X_error > -0.1 && X_error < 0.1)
                        {
                            attitude.x = 0;                                   
                            
                            att_pub.publish(attitude);
                            Pframes++;
                            if (Pframes > 25)
                            {
                                PFlag = false;
                                // cout<<"Target X "<<vect[0][0]<<" Actual x "<<Xa<<endl;
                                cout<<"y-axis +  pitching holding completed "<<endl;
                            }
                        }

                        else
                        {
                            attitude.x = Kp_p * -pitch_rate;                  
                            att_pub.publish(attitude);
                        }
                    }

                    //Roll rate controller
                    if(!PFlag && !YFlag)
                    {
                        // Swaping roll movement with pitch movement
                        if(vect[0][3] > 0.707 && vect[0][3] < 2.277)
                            {
                                double Ya = current_pose.pose.position.y;
                                float Y_error = vect[0][1] - Ya;                        //Calculate error
                                if (Y_error > 0.1 )
                                {
                                    attitude.x = Kp_r * roll_rate;                    //Share bodyrate data in drone roll axis           
                                    att_pub.publish(attitude);
                                }

                                else if (Y_error > -0.1 && Y_error < 0.1)
                                {
                                    attitude.x = 0;                                    
                                    att_pub.publish(attitude);
                                    Rframes++;
                                    if (Rframes > 25)
                                    {
                                        RFlag = false;
                                        // cout<<"Target Y "<<vect[0][1]<<" current y "<<Ya<<endl;
                                        cout<<"x-axis + rolling completed" <<endl;
                                    }
                                }

                                else
                                {
                                    attitude.x = Kp_r * -roll_rate;                 
                                    att_pub.publish(attitude);
                                }

                            }

                            else
                            {

                                double Ya = current_pose.pose.position.y;

                                //roll rate controller
                                float Y_error = vect[0][1] - Ya;
                                if (Y_error > 0.1 )
                                {
                                    attitude.y = Kp_r * -roll_rate;             //Share bodyrate data in drone pitch axis
                                    att_pub.publish(attitude);
                                }

                                else if (Y_error > -0.1 && Y_error < 0.1)
                                {
                                    attitude.y = 0;                                 
                                    // cout<<"y-axis + rolling holding : "<<Y_error<<endl;
                                    att_pub.publish(attitude);
                                    Rframes++;
                                    if (Rframes > 25)
                                    {
                                        RFlag = false;
                                        // cout<<"Target Y "<<vect[0][1]<<" current y "<<Ya<<endl;
                                        cout<<"y-axis + rolling holding completed"<<endl;
                                    }
                                }

                                else
                                {
                                    attitude.y = Kp_r * roll_rate;                 
                                    att_pub.publish(attitude);
                                }

                            }
                        
                }

                }
               
            }
            
        // cout<<"Z "<<ZFlag<<", P "<<PFlag<<", R "<<RFlag <<", Y "<<YFlag<<endl;

        if (!ZFlag && !PFlag  && !YFlag && !RFlag && (ros::Time::now() - last_request > ros::Duration(10.0)) && vect.size()>0)
        {
            cout<<"---Reached at set_point------ "<<": [x= "<< vect[0][0] <<", y= "<<vect[0][1] <<", z= "<<vect[0][2] << ", yaw= "<<vect[0][3]<<"]"<<endl;
            last_request = ros::Time::now();
            vect.erase(vect.begin());
            sleep(0.1);

            ZFlag = true;
            RFlag = true;
            PFlag = true;
            YFlag = true;

        }


        if (vect.size() == 0)          //When drone reached all points call position hold mode
        {
            set_mode_client.call(hold_set_mode);
            cout<<"Position Hold Mode Activated!"<<endl;
        }
    }

        //To keep hold pose
        att_pub.publish(attitude);

        ros::spinOnce();
        rate.sleep();
    }
    cout<<"Control back to the simulated autopilot and exit"<<endl;
    return 0;
    
}



