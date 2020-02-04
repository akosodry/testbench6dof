#include <exception>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

#include <termios.h>
#include <stdio.h>      
#include <stdlib.h>     
#include <time.h>       

#define JOINT_NO 6

int extAccelFlag = 0;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr<arm_control_client>  arm_control_client_Ptr;

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

// Create a ROS action client to move the testbench
void createArmClient(arm_control_client_Ptr& actionClient)
{
    ROS_INFO("Creating action client to testbench controller ...");

    actionClient.reset( new arm_control_client("/testenv_arm_joint_controller/follow_joint_trajectory") );

    int iterations = 0, max_iterations = 5;
    // Wait for arm controller action server to come up
    while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
        ROS_DEBUG("Waiting for the arm_controller_action server to come up");
        ++iterations;
    }

    if ( iterations == max_iterations )
        throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}

double randomDouble(double a, double b)
{
    double random = ((double) rand()) / (double) RAND_MAX;
    double diff = b - a;
    double r = random * diff;
    return a + r;
}

// Generates a simple trajectory with random waypoints to move the testbench
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.clear();
    goal.trajectory.points.clear();
    goal.trajectory.joint_names.push_back("testenv_joint_1");
    goal.trajectory.joint_names.push_back("testenv_joint_2");
    goal.trajectory.joint_names.push_back("testenv_joint_3");
    goal.trajectory.joint_names.push_back("testenv_joint_4");
    goal.trajectory.joint_names.push_back("testenv_joint_5");
    goal.trajectory.joint_names.push_back("testenv_joint_6");

    double gen_fs = 20; //Hz

    double noOfSec = 0.5;
    double time = 0;

    double confident_factor = 0.6;
    double confident_factor2 = 0.8;
    double joint_limits_min[JOINT_NO] = {0, 0, 0, -2.5, -1, -2.5};
    double joint_limits_max[JOINT_NO] = {3, 3, 3, +2.5, +1, +2.5};
    double maxFreq = 2;

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.resize(JOINT_NO);
    p.velocities.resize(JOINT_NO);

    unsigned int r3 = randomDouble(0.1,4.9);
    for (unsigned int j=0; j<5; j++)
    {
        double speedFactor = randomDouble(1.0,5.0);

        double r1 = randomDouble(0.1,0.9);
        double r2 = randomDouble(0.1,0.9);

        if((j==r3))
        {
            r1 = 2;
            r2 = 2;
        }

        for (unsigned int pwmCntr=0; pwmCntr<((unsigned int) (noOfSec*gen_fs)); pwmCntr++)
        {
            if((pwmCntr==((unsigned int) (noOfSec*gen_fs*r1))) || (pwmCntr==((unsigned int) (noOfSec*gen_fs*r1))))
            {
                if(extAccelFlag == 1)
                {
                    p.positions[0] = randomDouble(joint_limits_min[0], joint_limits_max[0])*confident_factor2;
                    p.positions[1] = randomDouble(joint_limits_min[1], joint_limits_max[1])*confident_factor2;
                    p.positions[2] = randomDouble(joint_limits_min[2], joint_limits_max[2])*confident_factor2;
                }

                speedFactor = randomDouble(1.0,5.0);
            }

            double trigArg = ((double) pwmCntr)*2.0*3.1415926535897932384626433832795/gen_fs;

            p.positions[3] = randomDouble(joint_limits_min[3]*confident_factor, joint_limits_max[3]*confident_factor) *
                    sin(randomDouble(0,maxFreq)*trigArg);

            p.positions[4] = randomDouble(joint_limits_min[4]*confident_factor, joint_limits_max[4]*confident_factor) *
                    sin(randomDouble(0,maxFreq)*trigArg);

            p.positions[5] = randomDouble(joint_limits_min[5]*confident_factor, joint_limits_max[5]*confident_factor) *
                    sin(randomDouble(0,maxFreq)*trigArg);

            unsigned int r1 = (unsigned int) randomDouble(0,2);
            unsigned int r2 = (unsigned int) randomDouble(0,2);
            unsigned int r3 = (unsigned int) randomDouble(0,2);

            if(extAccelFlag == 1)
            {
                if(r1 == 0)
                    p.positions[0] = p.positions[0]+ randomDouble(-0.3,0.3);

                if(r2 == 0)
                    p.positions[1] = p.positions[1]+ randomDouble(-0.3,0.3);

                if(r3 == 0)
                    p.positions[2] = p.positions[2]+ randomDouble(-0.3,0.3);
            }

            time += (1/gen_fs)*speedFactor*1.2;
            p.time_from_start = ros::Duration(time);
            goal.trajectory.points.push_back(p);
        }
    }
}

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "exec_traj_control");

    ROS_INFO("Starting exec_traj_control node ...");

    // Precondition: Valid clock
    ros::NodeHandle nh;
    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
    {
        ROS_FATAL("Timed-out waiting for valid time.");
        return EXIT_FAILURE;
    }

    arm_control_client_Ptr ArmClient;
    createArmClient(ArmClient);

    srand (time(NULL));

    // Sends the command to start the given trajectory 1s from now
    ROS_INFO("Sends the command to start the given trajectory 1s from now ...");

    while(ros::ok())
    {
        ROS_INFO("press s or w ...");
        extAccelFlag = 1;
        
        int c = getch();   // call your non-blocking input function
        if (c == 's')
        {
            extAccelFlag = 0;
            ROS_INFO("   no ext accel is generated ...");
        }
        else if(c == 'w')
        {
            extAccelFlag = 1;
            ROS_INFO("   ext accel is generated ...");
        }


        control_msgs::FollowJointTrajectoryGoal arm_goal;

        waypoints_arm_goal(arm_goal);

        arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
        ArmClient->sendGoal(arm_goal);


        // Wait for trajectory execution
        while(!(ArmClient->getState().isDone()) && ros::ok())
        {
            ros::Duration(0.1).sleep(); // sleep for four seconds
        }
    }

    return EXIT_SUCCESS;
}
