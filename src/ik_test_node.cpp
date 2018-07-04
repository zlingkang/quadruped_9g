#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "urdf/model.h"

const double PI = 3.14159;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_test_node");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(5);

    std::string robot_desc_string;
    n.param("robot_description", robot_desc_string, std::string());

    KDL::Tree my_tree;
    if(!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }

    std::vector<std::string> joint_name = {"shoulder_joint_lf", "elbow_joint_lf", "wrist_joint_lf", "ankle_joint_lf",
                                           "shoulder_joint_rf", "elbow_joint_rf", "wrist_joint_rf", "ankle_joint_rf", 
                                           "shoulder_joint_lb", "elbow_joint_lb", "wrist_joint_lb", "ankle_joint_lb",
                                           "shoulder_joint_rb", "elbow_joint_rb", "wrist_joint_rb", "ankle_joint_rb" };
    std::vector<double> joint_pos = {0, PI/6.0, -PI/3.0, -PI/3.0,
                                     0, PI/6.0, -PI/3.0, -PI/3.0,
                                     0, PI/6.0, -PI/3.0, -PI/3.0,
                                     0, PI/6.0, -PI/3.0, -PI/3.0};
    // for joints pos pub
    sensor_msgs::JointState joint_state;
    // for odom pub
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // ik stuff initialization
    std::string urdf_param = "/robot_description";
    double timeout = 0.005;
    double eps = 1e-5;
    std::string chain_start = "shoulder_link_lf"; 
    std::string chain_end = "feet_link_lf"; 
    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
    KDL::Chain chain;
    KDL::JntArray ll, ul; //joint lower limits, joint upper limits
    bool valid = tracik_solver.getKDLChain(chain);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
    }
    valid = tracik_solver.getKDLLimits(ll, ul);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found");
    }
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    ROS_INFO("joints number: %d", chain.getNrOfJoints());
    KDL::JntArray nominal(chain.getNrOfJoints()); // starting joints config
    for(size_t j = 0; j < nominal.data.size(); j ++)
    {
        nominal(j) = (ll(j) + ul(j))/2.0;
    }
    KDL::JntArray q(chain.getNrOfJoints()); // target joints config
    q(0) = 0.4;
    q(1) = -0.8;
    KDL::Frame end_effector_pose;
    KDL::JntArray result;

    bool flag = true;
    double x_trans = 0;
    
    auto print_frame_lambda = [](KDL::Frame f)
    {
        double x, y, z, roll, pitch, yaw;
        x = f.p.x();
        y = f.p.y();
        z = f.p.z();
        f.M.GetRPY(roll, pitch, yaw);
        std::cout << "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
    };

    while(ros::ok())
    {
        // ik computation
        if(flag)
        {
            fk_solver.JntToCart(q, end_effector_pose);
            int rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
            print_frame_lambda(end_effector_pose);
        }
        else
        {
            fk_solver.JntToCart(nominal, end_effector_pose); 
            int rc = tracik_solver.CartToJnt(q, end_effector_pose, result);
            print_frame_lambda(end_effector_pose);
        }
        flag = !flag;

        // update joint_state
        ROS_INFO("update joint state");
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(16);
        joint_state.position.resize(16);
        for(size_t i = 0; i < 16; i ++)
        {
            joint_state.name[i] = joint_name[i];
            joint_state.position[i] = joint_pos[i];
        }
        joint_state.position[1] = result(0);
        joint_state.position[2] = result(1);

        // update odom transform
        ROS_INFO("update odom trans");
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x_trans;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0.0866;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

        ROS_INFO("pub joint state");
        joint_pub.publish(joint_state);
        ROS_INFO("pub odom trans");
        broadcaster.sendTransform(odom_trans);

        x_trans += 0.001;

        loop_rate.sleep();
    }

    return 0;
}
