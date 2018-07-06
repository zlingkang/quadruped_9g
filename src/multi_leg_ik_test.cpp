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
const double SHOU_L = 0.04;
const double CLEG_L = 0.086;
const double HBDY_L = 0.05;
const int JNTS_NUM = 20;
enum leg_type {LF, RF, LB, RB};

class LegIK{
    public:
        LegIK(const std::string _name);
        LegIK(const std::string _name, bool _up2down, std::string _urdf_param, double _timeout, double _eps);
        void init();
        void setUp2Down(const bool _up2down); // set leg chain direction
        void setEndPose(const std::vector<double>& _pose); // x, y, z, R, P, Y
        bool getJntArray(std::vector<double>& _jnts);
    private:
        std::string name_;
        leg_type legt_;
        bool up2down_; // the chain direction: from base to shoe, or inverse
        std::string urdf_param_;
        double timeout_;
        double eps_;

        // base to shoe 
        TRAC_IK::TRAC_IK* tracik_solver_ptr_;
        KDL::ChainFkSolverPos_recursive* fk_solver_ptr_;
        KDL::Chain chain_;
        KDL::JntArray jnt_array_; // current joint angle array
        KDL::JntArray ll_, ul_; // joint angle lower bound and upper bound
        std::string chain_start_;
        std::string chain_end_;
        std::vector<double> start2end_; // start to end transformation x y z R P Y, for setEndPose preprocess 
        
        // shoe to base 
        TRAC_IK::TRAC_IK* inv_tracik_solver_ptr_;
        KDL::ChainFkSolverPos_recursive* inv_fk_solver_ptr_;
        KDL::Chain inv_chain_;
        KDL::JntArray inv_jnt_array_; // current joint angle array
        KDL::JntArray inv_ll_, inv_ul_; // joint angle lower bound and upper bound
        std::string inv_chain_start_;
        std::string inv_chain_end_;
        std::vector<double> inv_start2end_; // start to end transformation x y z R P Y, for setEndPose preprocess 

        KDL::Frame target_frame_;
};

void LegIK::init()
{
    if(name_ == "lf")
    {
        legt_ = LF; 
    }
    else if(name_ == "rf")
    {
        legt_ = RF;
    }
    else if(name_ == "lb")
    {
        legt_ = LB;
    }
    else if(name_ == "rb")
    {
        legt_ = RB;
    }

    // base to shoe
    chain_start_ = "base_link";
    chain_end_ = "shoe_link_" + name_;
    tracik_solver_ptr_ = new TRAC_IK::TRAC_IK(chain_start_, chain_end_, urdf_param_, timeout_, eps_);
    bool valid = tracik_solver_ptr_->getKDLChain(chain_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
    }
    valid = tracik_solver_ptr_->getKDLLimits(ll_, ul_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found");
    }
    fk_solver_ptr_ = new KDL::ChainFkSolverPos_recursive(chain_);
    jnt_array_ = KDL::JntArray(chain_.getNrOfJoints());
    std::vector<std::vector<double>> base2shoe {{HBDY_L, SHOU_L, -CLEG_L, 0, 0, 0},
                                                {HBDY_L, -SHOU_L, -CLEG_L, 0, 0, 0},
                                                {-HBDY_L, SHOU_L, -CLEG_L, 0, 0, 0},
                                                {-HBDY_L, -SHOU_L, -CLEG_L, 0, 0, 0}};
    jnt_array_(0) = 0; // shoulder joint
    jnt_array_(1) = PI/6.0;
    jnt_array_(2) = -PI/3.0;
    jnt_array_(3) = -PI/3.0;
    jnt_array_(4) = 0; // shoe joint
    switch(legt_)
    {
        case LF: start2end_ = base2shoe[0]; break;
        case RF: start2end_ = base2shoe[1]; break;
        case LB: start2end_ = base2shoe[2]; break;
        case RB: start2end_ = base2shoe[3]; break;
    }
   
    
    // shoe to base
    inv_chain_start_ = "shoe_link_" + name_;
    inv_chain_end_ = "base_link"; 
    inv_tracik_solver_ptr_ = new TRAC_IK::TRAC_IK(inv_chain_start_, inv_chain_end_, urdf_param_, timeout_, eps_);
    valid = inv_tracik_solver_ptr_->getKDLChain(inv_chain_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
    }
    valid = inv_tracik_solver_ptr_->getKDLLimits(inv_ll_, inv_ul_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found");
    }
    inv_fk_solver_ptr_ = new KDL::ChainFkSolverPos_recursive(chain_);
    inv_jnt_array_ = KDL::JntArray(chain_.getNrOfJoints());
    std::vector<std::vector<double>> shoe2base {{-HBDY_L, -SHOU_L, CLEG_L, 0, 0, 0},
                                                {-HBDY_L, SHOU_L, CLEG_L, 0, 0, 0},
                                                {HBDY_L, -SHOU_L, CLEG_L, 0, 0, 0},
                                                {HBDY_L, SHOU_L, CLEG_L, 0, 0, 0}};
    inv_jnt_array_(0) = 0;
    inv_jnt_array_(1) = -PI/3.0;
    inv_jnt_array_(2) = -PI/3.0;
    inv_jnt_array_(3) = PI/6.0;
    inv_jnt_array_(4) = 0;
    switch(legt_)
    {
        case LF: inv_start2end_ = shoe2base[0]; break;
        case RF: inv_start2end_ = shoe2base[1]; break;
        case LB: inv_start2end_ = shoe2base[2]; break;
        case RB: inv_start2end_ = shoe2base[3]; break;
    }

}

LegIK::LegIK(const std::string _name):
    name_(_name),
    up2down_(true),
    urdf_param_("/robot_description"),
    timeout_(0.005),
    eps_(2e-5)
{
    init();
}

LegIK::LegIK(const std::string _name, bool _up2down, std::string _urdf_param, double _timeout, double _eps):
    name_(_name),
    up2down_(_up2down),
    urdf_param_(_urdf_param),
    timeout_(_timeout),
    eps_(_eps)
{
    init();
}

void LegIK::setUp2Down(const bool _up2down)
{
    up2down_ = _up2down;
}

void LegIK::setEndPose(const std::vector<double>& _pose) // x, y, z, R, P, Y
{
    std::vector<double> bias;
    if(up2down_)
    {
        bias = start2end_;
    }
    else
    {
        bias = inv_start2end_;
    }
    double x = _pose[0] + bias[0];
    double y = _pose[1] + bias[1];
    double z = _pose[2] + bias[2];
    double R = _pose[3] + bias[3];
    double P = _pose[4] + bias[4];
    double Y = _pose[5] + bias[5];
    KDL::Vector v(x, y, z);
    target_frame_ = KDL::Frame(KDL::Rotation::RPY(R, P, Y), v);
}

bool LegIK::getJntArray(std::vector<double>& _jnts)
{
    KDL::JntArray result; 
    int rc = 0; 
    if(up2down_)
    {
        rc = tracik_solver_ptr_->CartToJnt(jnt_array_, target_frame_, result);
        for(size_t i = 0, j = chain_.getNrOfJoints()-1; i < chain_.getNrOfJoints(); i ++, j--)
        {
            inv_jnt_array_(j) = jnt_array_(i);         
        }
    }
    else
    {
        rc = inv_tracik_solver_ptr_->CartToJnt(inv_jnt_array_, target_frame_, result);
        for(size_t i = 0, j = chain_.getNrOfJoints()-1; i < chain_.getNrOfJoints(); i ++, j--)
        {
            jnt_array_(j) = inv_jnt_array_(i); 
        }
    }
    if(rc < 0)
    {
        return false;
    }
    else
    {
        _jnts.clear();
        if(up2down_)
        {
            for(size_t i = 0; i < chain_.getNrOfJoints(); i ++)
            {
                _jnts.push_back(result(i));
            }
        }
        else
        {
            for(int i = chain_.getNrOfJoints()-1; i > -1; i --)
            {
                _jnts.push_back(result(i)); 
            }
        }
        return true;
    }
}

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
    
    int flag = -1;
    double x_trans = 0;
    double y_trans = 0;
    double z_trans = 0;
    double yaw_rot = 0;
    while(ros::ok())
    {
        // set leg status: up2down or down2up    
        legs[0].setUp2Down(false); 
        legs[1].setUp2Down(false); 
        legs[2].setUp2Down(false); 
        legs[3].setUp2Down(false); 

        // set end pose
        if(x_trans > 0.02 || x_trans < -0.02)
        {
            flag = -flag;
        }
        x_trans += 0.001 * flag;
        //y_trans += 0.0005 * flag;
        //z_trans += 0.0002 * flag;
        //yaw_rot += 0.0001 * flag;
        for(int i = 0; i < 4; i ++)
        {
            std::vector<double> pose{x_trans, y_trans, z_trans, 0, 0, 0}; // x y z r y p 
            end_pose[i] = pose;
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
        odom_trans.transform.translation.x = x_trans;
        odom_trans.transform.translation.y = y_trans;
        odom_trans.transform.translation.z = CLEG_L + z_trans;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_rot);

        ROS_INFO("pub joint state");
        joint_pub.publish(joint_state);
        ROS_INFO("pub odom trans");
        broadcaster.sendTransform(odom_trans);

        loop_rate.sleep();
       
    }
}
