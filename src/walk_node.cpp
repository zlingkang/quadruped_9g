#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "urdf/model.h"

#include "quadruped_9g/leg_ik.hpp"

using namespace quadruped_9g;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_test_node");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    std::string robot_desc_string;
    n.param("robot_description", robot_desc_string, std::string());

    std::vector<std::string> joint_name = {"shoulder_joint_lf", "elbow_joint_lf", "wrist_joint_lf", "ankle_joint_lf", "shoe_joint_lf", 
                                           "shoulder_joint_rf", "elbow_joint_rf", "wrist_joint_rf", "ankle_joint_rf", "shoe_joint_rf",  
                                           "shoulder_joint_lb", "elbow_joint_lb", "wrist_joint_lb", "ankle_joint_lb", "shoe_joint_lb",
                                           "shoulder_joint_rb", "elbow_joint_rb", "wrist_joint_rb", "ankle_joint_rb", "shoe_joint_rb"};
    
    // for joints pos pub
    sensor_msgs::JointState joint_state;
    // for odom pub
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // initialize four legs
    LegIK leg_lf("lf");
    LegIK leg_rf("rf");
    LegIK leg_lb("lb");
    LegIK leg_rb("rb");
    std::vector<LegIK> legs{leg_lf, leg_rf, leg_lb, leg_rb};
    
    std::vector<std::vector<double>> end_pose(4, std::vector<double>(6, 0.0)); // 4 chain, end pose has 6dof
    std::vector<std::vector<double>> result(4, std::vector<double>(5, 0.0)); // 4 legs, every leg has 5 joints 
   
    bool first = true;
    int flag = -1;
    int cycle_cnt = 0; // walk cycle cnt
    double CYCLE_L = 0.015; // (1/4)cyle length
    double STEP_L = 0.001; // move distance in every loop
    double x_trans_total = 0;
    // for base_link pose
    double x_trans = 0;
    double y_trans = 0;
    double z_trans = 0;
    // for shoe_link pose
    double x_trans_shoe = 0;
    double y_trans_shoe = 0;
    double z_trans_shoe = 0;
    while(ros::ok())
    {
        if(x_trans_total >= CYCLE_L*(cycle_cnt+1))
        {
            cycle_cnt += 1;
        }
        if(x_trans >= 2*CYCLE_L)
        {
            x_trans = 0.0;
            x_trans_shoe = -2*CYCLE_L;
            first = false;
        }

        ROS_INFO("cycle #%d", cycle_cnt);

        // set leg status: up2down or down2up
        if(cycle_cnt%4 == 0 || cycle_cnt%4 == 1)
        {
            legs[0].setUp2Down(true); 
            legs[1].setUp2Down(false); 
            legs[2].setUp2Down(false); 
            legs[3].setUp2Down(true); 
        }
        else
        {
            legs[0].setUp2Down(false); 
            legs[1].setUp2Down(true); 
            legs[2].setUp2Down(true); 
            legs[3].setUp2Down(false); 
        }

        // set end pose
        x_trans += STEP_L;
        x_trans_total += STEP_L;
        if(!first)
        {
            x_trans_shoe += STEP_L;
        }
        if(cycle_cnt%4 == 0 || cycle_cnt%4 == 2)
        {
            z_trans_shoe += STEP_L;
        }
        else
        {
            z_trans_shoe -= STEP_L;
        }
        ROS_INFO("x_trans: %lf", x_trans);
        ROS_INFO("z_trans_shoe: %lf", z_trans_shoe);

        for(int i = 0; i < 4; i ++)
        {
            if(legs[i].Up2Down())
            {
                std::vector<double> pose{x_trans_shoe, y_trans_shoe, z_trans_shoe, 0, 0, 0}; // x y z r y p 
                end_pose[i] = pose;
            }
            else
            {
                std::vector<double> pose{x_trans, y_trans, z_trans, 0, 0, 0}; // x y z r y p 
                end_pose[i] = pose;
            }
        }

        // calculate the IK
        for(int i = 0; i < 4; i ++)
        {
            legs[i].setEndPose(end_pose[i]);
            if(!legs[i].getJntArray(result[i]))
            {
                std::cout <<"not success" << std::endl;
                return -1;
            }
            else
            {
                std::cout << "success" << std::endl;
            }
        }

        // update joint_state
        ROS_INFO("update joint state");
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(JNTS_NUM);
        joint_state.position.resize(JNTS_NUM);
        std::vector<double> new_pos;
        for(int i = 0; i < 4; i ++)
        {
            for(int j = 0; j < 5; j ++)
            {
                new_pos.push_back(result[i][j]);
            } 
        } 
        for(size_t i = 0; i < JNTS_NUM; i ++) 
        { 
            joint_state.name[i] = joint_name[i]; 
            joint_state.position[i] = new_pos[i]; 
        } 
        
        // update odom transform ROS_INFO("update odom trans");
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x_trans_total;
        odom_trans.transform.translation.y = y_trans;
        odom_trans.transform.translation.z = CLEG_L + z_trans;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

        ROS_INFO("pub joint state");
        joint_pub.publish(joint_state);
        ROS_INFO("pub odom trans");
        broadcaster.sendTransform(odom_trans);

        loop_rate.sleep();
       
    }
}
