#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <boost/assign/list_of.hpp>

#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


//FILE *fp;

double sim_granularity=0.0;
double angular_sim_granularity=0.0;
const double sim_period=0.2;
double sim_time=0.0;

bool start=false;
double sec=0;

double lv_acc=0.0; 
double av_acc=0.0;

double lv_goal=0.0;
double av_goal=0.0;

void velCallback(const geometry_msgs::Twist::ConstPtr& vel){
    lv_goal=vel->linear.x;
    av_goal=vel->angular.z;
    start=true;
}
double computeNewVelocity(double vg, double vi, double a_max, double dt){
    if((vg - vi) >= 0) {
        return std::min(vg, vi + a_max * dt);
    }
    return std::max(vg, vi - a_max * dt);
}
double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
    return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
}
double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
    return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
} 
double computeNewThetaPosition(double thetai, double vth, double dt){
    return thetai + vth * dt;
}
int main(int argc, char* argv[]){
    ros::init(argc,argv, "nav_stack_test");
    /*
    std::string file=std::string("/home/aloe/test.csv");;
    if(!ros::param::get("~file",file)){
        ros::param::set("~file",file);
    }
    
    fp=fopen(file.c_str(),"w");
    if(fp==NULL){
        ROS_ERROR("Failed to open %s",file.c_str());
        return 0;
    }
    printf(file.c_str());
    fprintf(fp,"time,ref l.V,l.V,ref a.v,a.v,x,y,th\n");
    */
    
    ros::NodeHandle n;
    ros::NodeHandle pn;
    n.getParam("/move_base/MyTrajectoryPlannerROS/acc_lim_theta",av_acc);
    n.getParam("/move_base/MyTrajectoryPlannerROS/acc_lim_x",lv_acc);
    n.getParam("/move_base/MyTrajectoryPlannerROS/sim_time",sim_time);
    n.getParam("/move_base/MyTrajectoryPlannerROS/angular_sim_granularity",angular_sim_granularity);
    n.getParam("/move_base/MyTrajectoryPlannerROS/sim_granularity",sim_granularity);
    printf("%f,%f,%f,%f,%f\n",av_acc,lv_acc,sim_time,sim_granularity,angular_sim_granularity);
    ros::Subscriber sub = n.subscribe("cmd_vel",6000,velCallback);
    ros::Publisher odom_publisher = pn.advertise<nav_msgs::Odometry>("odom", 6000);  
    tf::TransformBroadcaster tf_broadcaster;
    
    ros::Rate loop_rate(5);
    

    //yaw to quarternion
    double th=0.0;
    geometry_msgs::Quaternion odom_quat;
    
    //tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    
    
    //nav_msgs
    nav_msgs::Odometry odom;        
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    ros::Time current_time;
    while(ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
        if(start){
            int num_steps;
            num_steps = int(
                std::max(
                fabs(((lv_goal+odom.twist.twist.linear.x)/2)*(fabs(lv_goal-odom.twist.twist.linear.x)/lv_acc)
                    +lv_goal*(sim_time-fabs(lv_goal-odom.twist.twist.linear.x)/lv_acc))/sim_granularity,
                fabs(((av_goal+odom.twist.twist.angular.z)/2)*(fabs(av_goal-odom.twist.twist.angular.z)/av_acc)
                    +av_goal*(sim_time-fabs(av_goal-odom.twist.twist.angular.z)/av_acc))/angular_sim_granularity)
                +0.5
            );
            if (num_steps==0) num_steps=1;
            double dt = sim_time / num_steps;
            double time=0.0;
            for(int i = 0; i < num_steps; ++i){
                odom.twist.twist.linear.x = computeNewVelocity(lv_goal, odom.twist.twist.linear.x, lv_acc, dt);
                odom.twist.twist.angular.z = computeNewVelocity(av_goal, odom.twist.twist.angular.z, av_acc, dt);

                //calculate positions
                odom.pose.pose.position.x = computeNewXPosition(odom.pose.pose.position.x, odom.twist.twist.linear.x, 0, th, dt);
                odom.pose.pose.position.y = computeNewYPosition(odom.pose.pose.position.y, odom.twist.twist.linear.x, 0, th, dt);
                th = computeNewThetaPosition(th, odom.twist.twist.angular.z, dt);
                time+=dt;
                if(time>sim_period) break;
            }
            odom_quat = tf::createQuaternionMsgFromYaw(th);
            current_time = ros::Time::now();
            //tf
            odom_trans.header.stamp = current_time;
            odom_trans.transform.translation.x = odom.pose.pose.position.x;
            odom_trans.transform.translation.y = odom.pose.pose.position.y;
            odom_trans.transform.rotation = odom_quat;

            //navmsg
            odom.header.stamp = current_time;
            odom.pose.pose.orientation= odom_quat;    
            
            
            tf_broadcaster.sendTransform(odom_trans);
            odom_publisher.publish(odom);
            start=false;
        }
        else{
            lv_goal=0;
            av_goal=0;
            int num_steps;
            num_steps = int(
                std::max(
                fabs((((lv_goal+odom.twist.twist.linear.x)/2)*(fabs(lv_goal-odom.twist.twist.linear.x)/lv_acc)
                    +lv_goal*(sim_time-fabs(lv_goal-odom.twist.twist.linear.x)/lv_acc))/sim_granularity),
                fabs(((av_goal+odom.twist.twist.angular.z)/2)*(fabs(av_goal-odom.twist.twist.angular.z)/av_acc)
                    +av_goal*(sim_time-fabs(av_goal-odom.twist.twist.angular.z)/av_acc))/angular_sim_granularity)
                +0.5
            );
            if (num_steps==0) num_steps=1;
            double dt = sim_time / num_steps;
            double time=0.0;
            for(int i = 0; i < num_steps; ++i){
                odom.twist.twist.linear.x = computeNewVelocity(lv_goal, odom.twist.twist.linear.x, lv_acc, dt);
                odom.twist.twist.angular.z = computeNewVelocity(av_goal, odom.twist.twist.angular.z, av_acc, dt);

                //calculate positions
                odom.pose.pose.position.x = computeNewXPosition(odom.pose.pose.position.x, odom.twist.twist.linear.x, 0, th, dt);
                odom.pose.pose.position.y = computeNewYPosition(odom.pose.pose.position.y, odom.twist.twist.linear.x, 0, th, dt);
                th = computeNewThetaPosition(th, odom.twist.twist.angular.z, dt);
                time+=dt;
                if(time>sim_period) break;
            }
            odom_quat = tf::createQuaternionMsgFromYaw(th);
            current_time = ros::Time::now();
            //tf
            odom_trans.header.stamp = current_time;
            odom_trans.transform.translation.x = odom.pose.pose.position.x;
            odom_trans.transform.translation.y = odom.pose.pose.position.y;
            odom_trans.transform.rotation = odom_quat;

            //navmsg
            odom.header.stamp = current_time;
            odom.pose.pose.orientation= odom_quat;    
            
            
            tf_broadcaster.sendTransform(odom_trans);
            odom_publisher.publish(odom);            
        }
        
    
    }
    
    //fclose(fp);
    
    return 0;
}
