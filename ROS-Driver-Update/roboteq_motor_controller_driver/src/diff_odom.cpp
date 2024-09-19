#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>


class Odometry_calc
{
public:
    Odometry_calc();
    void spin();

private:
    ros::NodeHandle n;
    ros::Subscriber volt_sub;
    ros::Subscriber fault_sub;
    ros::Subscriber l_wheel_sub;
    ros::Subscriber r_wheel_sub;
    ros::Publisher odom_pub;
    ros::Publisher cmd_vel_pub;

    tf::TransformBroadcaster odom_broadcaster;
    //Encoder related variables
    double encoder_min, encoder_max;
    double left, right; 

    double rate;
    ros::Duration t_delta;
    ros::Time t_next;
    ros::Time then;

    double enc_left, enc_right; 
    double ticks_rev, ticks_meter;  
    double base_width,wheel_dia;    

    double dx, dr;
    double x_final,y_final, theta_final;
    bool publish_tf_;

    void voltCb(const roboteq_motor_controller_driver::channel_values& volt);
    void faultCb(const roboteq_motor_controller_driver::channel_values& fault);
    void leftencoderCb(const roboteq_motor_controller_driver::channel_values& left_ticks);
    void rightencoderCb(const roboteq_motor_controller_driver::channel_values& right_ticks);    
    void init_variables();
    void update();
};

Odometry_calc::Odometry_calc()
{
    init_variables();
    ROS_INFO("Started odometry computing node");
    
    volt_sub = n.subscribe("/volts", 1000, &Odometry_calc::voltCb, this);
    fault_sub = n.subscribe("/fault_flag", 1000, &Odometry_calc::faultCb, this);
    l_wheel_sub = n.subscribe("/encoder_count", 1, &Odometry_calc::leftencoderCb, this);
    r_wheel_sub = n.subscribe("/encoder_count", 1, &Odometry_calc::rightencoderCb, this);

    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);   
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    //Retrieving parameters of this node
    ros::NodeHandle nhPriv("~");
    nhPriv.getParam("publish_tf", Odometry_calc::publish_tf_);    
    ROS_INFO_STREAM("odom diff publish_tf: " << publish_tf_);
}

void Odometry_calc::init_variables()
{
    left = 0;
    right = 0;

    wheel_dia = 0.15;  //0.15
    base_width = 0.565; //0.56

    // max rpm = 3000
    encoder_min =  -(4096 * 4 * 9 * 3000 / 60);
    encoder_max =  4096 * 4 * 9 * 3000 / 60;

    rate = 5; //5

    ticks_rev = 4096 * 4 * 9;
    ticks_meter = ticks_rev / (wheel_dia * M_PI);

    t_delta = ros::Duration(1.0 / rate);
    t_next = ros::Time::now() + t_delta;    
    then = ros::Time::now();

    enc_left = 0;
    enc_right = 0;

    dx = 0;
    dr = 0;
 
    x_final = 0; y_final=0; theta_final=0;
}


//Spin function
void Odometry_calc::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        update();
        loop_rate.sleep();
    }
}


//Update function
void Odometry_calc::update()
{
    ros::Time now = ros::Time::now();
    double elapsed;
    double d_left, d_right, d, th,x,y;

    if ( now > t_next) 
    {
        elapsed = now.toSec() - then.toSec(); 

        if ((left - enc_left) > encoder_max || (left - enc_left) < encoder_min || 
            (right - enc_right) > encoder_max || (right - enc_right) < encoder_min) 
        {
                // counter overflow... ignor once
            d_left = 0;
            d_right = 0;
        }
        else
        {
            d_left = (left - enc_left) / ( ticks_meter);
            d_right = (right - enc_right) / ( ticks_meter);
        }
        
        enc_left = left;
        enc_right = right;

        d = (d_left + d_right ) / 2.0;
        // ROS_INFO_STREAM(d_left << " : " << d_right);
        th = ( d_right - d_left ) / base_width;     
        dx = d /elapsed;
        dr = th / elapsed;
    
        if ( d != 0)
        {
            x = cos( th ) * d;
            //ROS_INFO_STREAM(x);
            y = -sin( th ) * d;
            // calculate the final position of the robot
            x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y );
            y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );
        }

        if( th != 0)
            theta_final = theta_final + th;

        geometry_msgs::Quaternion odom_quat ;

        odom_quat.x = 0.0;
        odom_quat.y = 0.0;
        odom_quat.z = 0.0;

        odom_quat.z = sin( theta_final / 2 );   
        odom_quat.w = cos( theta_final / 2 );

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = now;
        if (publish_tf_)
        {
            odom_trans.header.frame_id = "odom";
        }
        else
        {
            odom_trans.header.frame_id = "odom"; //odom_diff
        }
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x_final;
        odom_trans.transform.translation.y = y_final;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        if (publish_tf_)
        {
            odom_broadcaster.sendTransform(odom_trans);
        }

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = now;

        if (publish_tf_)
        {
            odom.header.frame_id = "odom";
        }
        else
        {
            odom.header.frame_id = "odom"; //odom_diff
        }

        //set the position
        odom.pose.pose.position.x = x_final;
        odom.pose.pose.position.y = y_final;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = dx;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = dr;

        //publish the message
        odom_pub.publish(odom);

        then = now;
        ros::spinOnce();
    }
//      ROS_INFO_STREAM("Not in loop");
}


void Odometry_calc::voltCb(const roboteq_motor_controller_driver::channel_values& volt)
{
    if (volt.value[2] < 4500)
    {
        ROS_INFO_STREAM("Low voltage");
    }
}


void Odometry_calc::faultCb(const roboteq_motor_controller_driver::channel_values& fault)
{
    if (fault.value[0] != 0)
    {
        ROS_INFO_STREAM("motor fault: " << fault.value[0]);
    }
    if (fault.value[0] == 0x20)
    {
        geometry_msgs::Twist cmd_vel_msg = {};
        cmd_vel_pub.publish(cmd_vel_msg);        
    }    
}


void Odometry_calc::leftencoderCb(const roboteq_motor_controller_driver::channel_values& left_ticks)
{
    left = left_ticks.value[0];
}


//Right encoder callback
void Odometry_calc::rightencoderCb(const roboteq_motor_controller_driver::channel_values& right_ticks)
{
    right = right_ticks.value[1];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv,"diff_odom");
    Odometry_calc obj;
    obj.spin();
    return 0;
}
