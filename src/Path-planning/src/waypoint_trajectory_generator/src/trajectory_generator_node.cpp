#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include "trajectory_generator_waypoint.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace Eigen;

// Param from launch file
    double _vis_traj_width;
    double _Vel, _Acc;
    int    _dev_order, _min_order;
    Vector3d  Start_point;
    Vector3d  End_point;
    BezierTrajOptimizer Beziertraj(7);     
    Vector3d _last_vel(0,0,0);
    Vector3d _last_acc(0,0,0);
    Vector3d _last_vel_front(0,0,0);
    Vector3d _last_acc_front(0,0,0);
// Set the obstacle map
    double _resolution, _inv_resolution;
    double _x_size, _y_size, _z_size;
    Vector3d _map_lower, _map_upper;
    int _max_x_id, _max_y_id, _max_z_id;
//
    TrajectoryGeneratorWaypoint * _trajGene      = new TrajectoryGeneratorWaypoint();
    FlightCorridor* _corridor                    = new FlightCorridor();
    FlightCorridor* _corridor2                   = new FlightCorridor();

// ros related
    ros::Subscriber _way_pts_sub,_way_pts_sub2,_way_pts_sub3,_map_sub,_back_drone_v_a_sub,_front_drone_v_a_sub;
    ros::Publisher  _wp_traj_vis_pub,_wp_traj_vis_pub2,_wp_traj_vis_pub3,_wp_traj_besier_vis_pub,
    _wp_traj_besier_vis_pub2, _wp_path_vis_pub,  _vel_pub,_vel_pub2,_jerk_pub,_acc_pub,_front_pos_pub,
    _corridor_pub,_corridor_pub2,_points_pub;

// for planning
    int _poly_num1D;
    MatrixXd _polyCoeff;
    VectorXd _polyTime;
    Vector3d _startPos = Vector3d::Zero();
    Vector3d _startVel = Vector3d::Zero();

// declare
    void visWayPointTraj( MatrixXd polyCoeff, VectorXd time,int flag);
    void visWayPointTraj_besier( VectorXd time,int flag);
    void visWayPointPath(MatrixXd path);
    Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t );
	Vector3d getVelocity(MatrixXd polyCoeff, int k, double t);
    Vector3d getAcc(MatrixXd polyCoeff, int k, double t);
    VectorXd timeAllocation( MatrixXd Path);
    void trajGeneration(Eigen::MatrixXd path,int flag);
    void rcvWaypointsCallBack(const nav_msgs::Path & wp);
    void rcvWaypointsCallBack2(const nav_msgs::Path & wp);
    void rcvMinSnapCallBack(const nav_msgs::Path & wp);
    nav_msgs::Path vector3d_to_waypoints(vector<Vector3d> path);

    void visCorridor(FlightCorridor* cor,int flag);


void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;
    _trajGene->resetObs();
    _corridor->resetObs();
    _corridor2->resetObs();
    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];
        // set obstalces into grid map for path planning
        _trajGene->setObs(pt.x, pt.y, pt.z);
        _corridor->setObs(pt.x, pt.y, pt.z);
        _corridor2->setObs(pt.x, pt.y, pt.z);
        // ROS_INFO("setting %f",pt.x);
    }

}



VectorXd corridor_time_generator(FlightCorridor* cor)
{ 
    VectorXd time(cor->cubes.size());

    // The time allocation is many relative timelines but not one common timeline
    for(int i = 0; i < time.rows(); i++)
    {
        double distance = (cor->cubes[i].start_node->coord-cor->cubes[i].end_node->coord).norm();
        // double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
        double x1 = _Vel * _Vel / (2 * _Acc); 
        double x2 = distance - 2 * x1;
        if(x2<=0){
            time(i) = 2*sqrt(distance/_Acc);
        }
        else{
            double t1 = _Vel / _Acc;
            double t2 = x2 / _Vel;
            time(i) = 2 * t1 + t2;;
        }
        // time(i)=1;
        //time(i) = distance/_Vel;
        // ROS_INFO("time i =%f",time(i));
    }
    return time;
}


void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }
    Start_point = wp_list[0];
    End_point = wp_list[wp_list.size()-1];
    ros::Time time_corr_start=ros::Time::now();
    int last_node_order=0;
    int check_order=0;
    int suc_flag=0;
    // while(false)
    int count=0;
    while(true)
    {   
        count++;
        if(count>100){
            ROS_INFO("died!!!");
            break;
        }
        GridNodePtr start_node=new GridNode(_corridor->coord2gridIndex(wp_list[last_node_order]),wp_list[last_node_order]);
        if(wp_list.size()==1){
            check_order=0;
            suc_flag=1;
        }
        else{
            for(check_order=last_node_order+1;check_order<wp_list.size();check_order++)
            {
                GridNodePtr end_node=new GridNode(_corridor->coord2gridIndex(wp_list[check_order]),wp_list[check_order]);
                FlightCube temp_cube(start_node,end_node);
                // ROS_INFO("current_cube safe check is %d   order=%d",_corridor->check_cube_safe(temp_cube),check_order);
                if(_corridor->check_cube_safe(temp_cube)&&check_order==wp_list.size()-1)
                {
                    suc_flag=1;
                    break;
                }
                else if(!_corridor->check_cube_safe(temp_cube))
                    break;
                else if(check_order==wp_list.size())
                {
                    ROS_WARN("no solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                }
            }
        }
        // ROS_INFO("suc_flag=%d path size=%d ,last_node_order=%d     check_order=%d",suc_flag,wp_list.size(),last_node_order,check_order);

        GridNodePtr end_node;
        if(suc_flag)
            end_node=new GridNode(_corridor->coord2gridIndex(wp_list[check_order]),wp_list[check_order]);
        else
            end_node=new GridNode(_corridor->coord2gridIndex(wp_list[check_order-1]),wp_list[check_order-1]);
        FlightCube temp_cube(start_node,end_node);
        _corridor->expand_cube(temp_cube);
        _corridor->update_attributes(temp_cube);
        // temp_cube.Display();
        _corridor->cubes.push_back(temp_cube);
        last_node_order=check_order-1;
        if(suc_flag)
            break;
    }
    VectorXd corridor_time=corridor_time_generator(_corridor);//生成时间
    ros::Time time_corr_end=ros::Time::now();
    ROS_WARN("corridor generation success! Time cost is %f  ms",(time_corr_end-time_corr_start).toSec()*1000);
    visCorridor(_corridor,1);
    //ROS_INFO("000000!");
    ROS_INFO_STREAM("Start:"<<Start_point<<"End: "<<End_point);
    Vector3d a(0,0,0);
    Vector3d v(0,0,0); 
    if (corridor_time.size()==1){//TRICK!!!
        corridor_time(0) = corridor_time(0)*2;
    }
    int bezier_flag = Beziertraj.bezierCurveGeneration(*_corridor,100000,10000000,Start_point,End_point,corridor_time,v,a);
    // if(bezier_flag==0)
    //     ROS_INFO("bezier traj generation success!!!");
    // else
    //     ROS_INFO("bezier traj generation failed!!!");    
    ros::Time time_bezier_end=ros::Time::now();
    ROS_WARN("bezier traj generation success! Time cost is %f  ms",(time_bezier_end-time_corr_end).toSec()*1000);

    visWayPointTraj_besier(corridor_time,1);
    _corridor->cubes.clear();


    


    //MatrixXd waypoints(wp_list.size() + 1, 3);
    //waypoints.row(0) = _startPos;
    //MatrixXd waypoints(wp_list.size(), 3);
    //for(int k = 0; k < (int)wp_list.size(); k++)
    //    waypoints.row(k+1) = wp_list[k];
    //for(int k = 0; k < (int)wp_list.size(); k++)
    //    waypoints.row(k) = wp_list[k];
    
    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    
    
    // trajGeneration(waypoints,0);
    //化简后的结点。
}

void rcvWaypointsCallBack2(const nav_msgs::Path & wp)
{   
    ROS_INFO("receive wp 2!!!");
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }
    Start_point = wp_list[0];
    End_point = wp_list[wp_list.size()-1];
    ros::Time time_corr_start=ros::Time::now();
    int last_node_order=0;
    int check_order=0;
    int suc_flag=0;
    // while(false)
    int count2=0;
    while(true)
    {
        count2++;
        if(count2>100)
        {
            ROS_INFO("%d",count2);
            break;
        }
        GridNodePtr start_node=new GridNode(_corridor2->coord2gridIndex(wp_list[last_node_order]),wp_list[last_node_order]);

        if(wp_list.size()==1)//解决终点起点重合的问题
        {
            check_order=0;
            suc_flag=1;
            // break;
        }
        else
        {
            for(check_order=last_node_order+1;check_order<wp_list.size();check_order++)
            {
                GridNodePtr end_node=new GridNode(_corridor2->coord2gridIndex(wp_list[check_order]),wp_list[check_order]);
                FlightCube temp_cube(start_node,end_node);
                // ROS_INFO("current_cube safe check is %d   order=%d",_corridor->check_cube_safe(temp_cube),check_order);
                int safe_flag=_corridor2->check_cube_safe(temp_cube);
                // ROS_INFO("check one cube   safe_flag=%d",safe_flag);
                if(safe_flag&&check_order==wp_list.size()-1)
                {
                    // ROS_INFO("break because of success!");
                    suc_flag=1;
                    break;
                }
                else if(!safe_flag)
                {
                    // ROS_INFO("break because of collision!");
                    break;
                }

                else if(check_order==wp_list.size())
                {
                    ROS_WARN("no solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                }
            }
        }
        

        
        ROS_INFO("suc_flag=%d path size=%d ,last_node_order=%d     check_order=%d",suc_flag,wp_list.size(),last_node_order,check_order);
        // if(last_node_order==check_order-1)
        // {
        //     ROS_WARN("dead loop!!!!!!!!!!!!!");
        //     // GridNodePtr end_node=new GridNode(_corridor2->coord2gridIndex(wp_list[check_order]),wp_list[check_order]);
        //     // FlightCube temp_cube(start_node,end_node);
        //     // _corridor2->set_safe_force(temp_cube);
        // }

        GridNodePtr end_node;
        if(suc_flag)
            end_node=new GridNode(_corridor2->coord2gridIndex(wp_list[check_order]),wp_list[check_order]);
        else
            end_node=new GridNode(_corridor2->coord2gridIndex(wp_list[check_order-1]),wp_list[check_order-1]);
        FlightCube temp_cube(start_node,end_node);
        _corridor2->expand_cube(temp_cube);
        _corridor2->update_attributes(temp_cube);
        temp_cube.Display();
        _corridor2->cubes.push_back(temp_cube);
        last_node_order=check_order-1;
        if(suc_flag)
            break;
    }
    VectorXd corridor_time=corridor_time_generator(_corridor2);//生成时间
    ros::Time time_corr_end=ros::Time::now();
    ROS_WARN("corridor2222 generation success! Time cost is %f  ms   size=%d",(time_corr_end-time_corr_start).toSec()*1000,_corridor2->cubes.size());
    visCorridor(_corridor2,2);
    // // //ROS_INFO("000000!");
    // // ROS_INFO_STREAM("Start:"<<Start_point<<"End: "<<End_point);
    Vector3d a(0,0,0);
    Vector3d v(0,0,0); 
    if (corridor_time.size()==1){//TRICK!!!
        corridor_time(0) = corridor_time(0)*2;
    }
    int bezier_flag = Beziertraj.bezierCurveGeneration(*_corridor2,10,10,Start_point,End_point,corridor_time,_last_vel,_last_acc);
    // int bezier_flag = Beziertraj.bezierCurveGeneration(*_corridor2,100,100,Start_point,End_point,corridor_time,v,a);
    ROS_INFO("start_p=%f %f   end_p=%f  %f",Start_point(0),Start_point(1),End_point(0),End_point(1));
    // if(bezier_flag==0)
    //     ROS_INFO("bezier traj generation success!!!");
    // else
    //     ROS_INFO("bezier traj generation failed!!!");    
    ros::Time time_bezier_end=ros::Time::now();
    ROS_WARN("bezier traj2 generation success! Time cost is %f  ms",(time_bezier_end-time_corr_end).toSec()*1000);

    visWayPointTraj_besier(corridor_time,2);
    _corridor2->cubes.clear();
}





void rcvMinSnapCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }

    //MatrixXd waypoints(wp_list.size() + 1, 3);
    //waypoints.row(0) = _startPos;
    MatrixXd waypoints(wp_list.size(), 3);
    //for(int k = 0; k < (int)wp_list.size(); k++)
    //    waypoints.row(k+1) = wp_list[k];
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k) = wp_list[k];
    
    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    
    
    trajGeneration(waypoints,0);
    //化简后的结点。
}



void trajGeneration(Eigen::MatrixXd path,int flag=0)
{   
    
    MatrixXd vel = MatrixXd::Zero(2, 3); 
    MatrixXd acc = MatrixXd::Zero(2, 3);

    vel.row(0) = _startVel;

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    _polyTime  = timeAllocation(path);

    // generate a minimum-snap piecewise monomial polynomial-based trajectory
    _polyCoeff = _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
    visWayPointPath(path);//可视化结点的连线
    visWayPointTraj( _polyCoeff, _polyTime,1);//画出不迭代时候的轨迹
    int unsafe_segment;
    if(!flag){
        //开始迭代
        MatrixXd repath = path;
        int count = 0;
        unsafe_segment = _trajGene->safeCheck(_polyCoeff, _polyTime);
        while(unsafe_segment != -1){
        //cout << "reoptimize!"<< endl;
            MatrixXd repath_a,repath_b;
            VectorXd mid(3);
            int rows = repath.rows();
            mid(0) = (repath(unsafe_segment,0) + repath(unsafe_segment+1,0))/2;
            mid(1) = (repath(unsafe_segment,1) + repath(unsafe_segment+1,1))/2;
            mid(2) = (repath(unsafe_segment,2) + repath(unsafe_segment+1,2))/2;
            repath_a = repath.block(0,0,unsafe_segment+1,3);
            repath_b = repath.block(unsafe_segment+1,0,rows-unsafe_segment-1,3);
            repath.resize(rows + 1,3);
            repath.block(0,0,unsafe_segment+1,3) = repath_a;
            repath(unsafe_segment+1,0) = mid(0);
            repath(unsafe_segment+1,1) = mid(1);
            repath(unsafe_segment+1,2) = mid(2);
            repath.block(unsafe_segment+2,0,rows-unsafe_segment-1,3) = repath_b;
            _polyTime  = timeAllocation(repath);
            _polyCoeff = _trajGene->PolyQPGeneration(_dev_order, repath, vel, acc, _polyTime);
            unsafe_segment = _trajGene->safeCheck(_polyCoeff, _polyTime);
            count++;
            if(count > 10)
            {
                unsafe_segment = -1;
                ROS_INFO("over replan!!!");
            }
            // visWayPointTraj( _polyCoeff, _polyTime,flag);
            visWayPointTraj( _polyCoeff, _polyTime,2);
            //为了迭代更加清楚，这里每次画图停止1s，工程实际使用要去掉哦
            ros::Rate rate(1);
            // rate.sleep();
        }
    ROS_INFO("iterated %d times",count);
    // if(count<=10)
    visWayPointTraj( _polyCoeff, _polyTime,flag);
    }

}


void rcvBackDroneVACallback(const nav_msgs::Path & wp)
{
    if (wp.poses.size()!=2)
    {
        ROS_WARN("Invalid v a!!!");
    }
    else
    {
        _last_vel<<wp.poses[0].pose.position.x,wp.poses[0].pose.position.y,wp.poses[0].pose.position.z;
        _last_acc<<wp.poses[1].pose.position.x,wp.poses[1].pose.position.y,wp.poses[1].pose.position.z;
    }    
}
void rcvFrontDroneVACallback(const nav_msgs::Path& wp){
    if (wp.poses.size()!=3)
    {
        ROS_WARN("Invalid v a j!!!");
    }
    else
    {
        _last_vel_front<<wp.poses[0].pose.position.x,wp.poses[0].pose.position.y,wp.poses[0].pose.position.z;
        _last_acc_front<<wp.poses[1].pose.position.x,wp.poses[1].pose.position.y,wp.poses[1].pose.position.z;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel",   _Vel,   1.0 );
    nh.param("planning/acc",   _Acc,   1.0 );//平均时速
    nh.param("planning/dev_order", _dev_order,  4 );
    nh.param("planning/min_order", _min_order,  3 );
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
    nh.param("map/resolution",    _resolution,   0.1);
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );

    //_poly_numID is the maximum order of polynomial
    _poly_num1D = 2 * _dev_order;

    //state of start point  
    _startVel(0)  = 0;
    _startVel(1)  = 0;
    _startVel(2)  = 0;
    
    _map_sub  = nh.subscribe( "/random_complex/global_map", 1, rcvPointCloudCallBack );

    _way_pts_sub     = nh.subscribe( "/demo_node/grid_path", 1, rcvWaypointsCallBack );//给贝塞尔曲线用，一号飞机
    //_way_pts_sub2     = nh.subscribe( "/demo_node/simplified_waypoints", 1, rcvMinSnapCallBack );//minisnap
    _way_pts_sub3     = nh.subscribe( "/demo_node/grid_path2", 1, rcvWaypointsCallBack2 );//给贝塞尔曲线用，二号飞机
 
    _back_drone_v_a_sub = nh.subscribe( "/drone_node/back_drone_v_a", 1, rcvBackDroneVACallback );
    _front_drone_v_a_sub = nh.subscribe("/drone_node/front_drone_v_a",1, rcvFrontDroneVACallback );
    // _way_pts_sub2    = nh.subscribe( "/demo_node/simplified_waypoints2", 1, rcvWaypointsCallBack2 );
    // _way_pts_sub3    = nh.subscribe( "/demo_node/simplified_waypoints3", 1, rcvWaypointsCallBack3 );//迭代的最后输出，matlab看速度用

    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);//s=d=迭代后
    _wp_traj_vis_pub2 = nh.advertise<visualization_msgs::Marker>("vis_trajectory2", 1);//迭代前
    _wp_traj_besier_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory_besier", 1);//一号飞机
    _wp_traj_besier_vis_pub2 = nh.advertise<visualization_msgs::Marker>("vis_trajectory_besier2", 1);//二号飞机
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);
    _vel_pub =         nh.advertise<nav_msgs::Path>("vel",1);
    _vel_pub2 =         nh.advertise<nav_msgs::Path>("vel2",1);
    _front_pos_pub = nh.advertise<nav_msgs::Path>("front_pos",1);
    _acc_pub =         nh.advertise<nav_msgs::Path>("acc",1);
    _jerk_pub =         nh.advertise<nav_msgs::Path>("jerk",1);
    _corridor_pub =    nh.advertise<visualization_msgs::Marker>("vis_corridor",1);
    _corridor_pub2 =    nh.advertise<visualization_msgs::Marker>("vis_corridor2",1);
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;  
    _inv_resolution = 1.0 / _resolution;
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);
    _trajGene-> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    _corridor-> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    _corridor2-> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();  
        status = ros::ok();  
        rate.sleep();
    }
    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time,int flag)
{        
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "world";
    //waypoint_trajectory_generator::Trajectoy _traj;
    //_traj.traj_points.clear();
    _traj_vis.ns = "traj_node/trajectory_waypoints";
    // else if(flag==1)
        // _traj_vis.ns = "traj_node/trajectory_waypoints2";

    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    if(flag==1)
    {
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;
    }

    else if(flag==0)
    {
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 1.0;
    }

    else if(flag==2)
    {
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 1.0;
    _traj_vis.color.b = 0.0;
    }

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;

    vector<Vector3d> vel_pub;
    vector<Vector3d> acc_pub;


    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
            if(flag==0||flag==1){
                Vector3d vel = getVelocity(polyCoeff, i, t);
                Vector3d acc = getAcc(polyCoeff,i,t);
                vel_pub.push_back(vel);
                acc_pub.push_back(acc);
                std::cout<<"acc="<<acc<<std::endl;
            }
            pos = getPosPoly(polyCoeff, i, t);
            cur(0) = pt.x = pos(0);
            cur(1) = pt.y = pos(1);
            cur(2) = pt.z = pos(2);
            _traj_vis.points.push_back(pt);
            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }
    ROS_INFO_STREAM("optimizer traj success, the length is "<<traj_len);
    if(flag==0)
    {
        //迭代后的轨迹
        _wp_traj_vis_pub.publish(_traj_vis);
        // _vel_pub.publish(vector3d_to_waypoints(vel_pub));
        // _acc_pub.publish(vector3d_to_waypoints(acc_pub));
    }
    else if(flag==1)//未迭代的轨迹
        _wp_traj_vis_pub2.publish(_traj_vis);
    else if(flag==2){
        //迭代后的轨迹
        _wp_traj_vis_pub.publish(_traj_vis); 
    }
    // _points_pub.publish(_traj);
}


void visWayPointTraj_besier( VectorXd time,int flag)
{        
    //可视化贝塞尔曲线
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "world";
    //waypoint_trajectory_generator::Trajectoy _traj;
    //_traj.traj_points.clear();
    _traj_vis.ns = "traj_node/trajectory_waypoints";
    // else if(flag==1)
        // _traj_vis.ns = "traj_node/trajectory_waypoints2";

    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;


    if(flag==1)
    {
        _traj_vis.color.a = 1.0;
        _traj_vis.color.r = 1.0;
        _traj_vis.color.g = 0.0;
        _traj_vis.color.b = 0.0;

    }

    else if(flag==2)
    {
        _traj_vis.color.a = 1.0;
        _traj_vis.color.r = 0.0;
        _traj_vis.color.g = 0.0;
        _traj_vis.color.b = 1.0;
    }
    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;

    vector<Vector3d> vel_pub;
    vector<Vector3d> acc_pub;
    vector<Vector3d> jerk_pub;
    vector<Vector3d> pos_pub;

    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
            // if(flag==0||flag==1){
            //     Vector3d vel = getVelocity(polyCoeff, i, t);
            //     Vector3d acc = getAcc(polyCoeff,i,t);
            //     vel_pub.push_back(vel);
            //     acc_pub.push_back(acc);
            // }
            Vector3d vel = Beziertraj.getVelFromBezier(t,i);
            Vector3d acc = Beziertraj.getAccFromBezier(t,i);
            Vector3d jerk = Beziertraj.getJerkFromBezier(t,i);
            // if(flag==2&&count==17)
            // {
            //     _last_acc=acc;
            //     _last_vel=vel;
            // }
                
        
            vel_pub.push_back(vel);
            acc_pub.push_back(acc);
            jerk_pub.push_back(jerk);


            pos = Beziertraj.getPosFromBezier(t,i);
            pos_pub.push_back(pos);
            cur(0) = pt.x = pos(0);
            cur(1) = pt.y = pos(1);
            cur(2) = pt.z = pos(2);
            _traj_vis.points.push_back(pt);
            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }
    ROS_INFO_STREAM("optimizer traj success, the length is "<<traj_len);

    if(flag==1)
    {
        
        _wp_traj_besier_vis_pub.publish(_traj_vis);
        _vel_pub.publish(vector3d_to_waypoints(vel_pub));
        _front_pos_pub.publish(vector3d_to_waypoints(pos_pub));
        _acc_pub.publish(vector3d_to_waypoints(acc_pub));
        _jerk_pub.publish(vector3d_to_waypoints(jerk_pub));
    }
    else if(flag==2)
    {
        _wp_traj_besier_vis_pub2.publish(_traj_vis);
        _vel_pub2.publish(vector3d_to_waypoints(vel_pub));
    }

}



void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "world";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = _vis_traj_width;
    line_list.scale.y = _vis_traj_width;
    line_list.scale.z = _vis_traj_width;
    line_list.color.a = 1.0;

    
    line_list.color.r = 1.0;
    line_list.color.g = 0.5;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}
Vector3d getVelocity(MatrixXd polyCoeff, int k, double t)
{
	Vector3d vel;
	for (int dim = 0; dim < 3; dim++) {
		VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
		VectorXd time = VectorXd::Zero(_poly_num1D);
		for (int j = 0; j < _poly_num1D; j++)
			if (j == 0)
				time(j) = 0;
			else if (j == 1)
				time(j) = 1;
			else
				time(j) = pow(t, j-1)*j;

		vel(dim) = coeff.dot(time);
	}
	return vel;
}
Vector3d getAcc(MatrixXd polyCoeff, int k, double t){
    Vector3d Acc;
	for (int dim = 0; dim < 3; dim++) {
		VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
		VectorXd time = VectorXd::Zero(_poly_num1D);
		for (int j = 0; j < _poly_num1D; j++)
			if (j == 0 || j==1)
				time(j) = 0;
			else if (j == 2)
				time(j) = 2;
			else
				time(j) = pow(t, j-2)*j*(j-1);

		Acc(dim) = coeff.dot(time);
	}
	return Acc;
}

VectorXd timeAllocation( MatrixXd Path)
{ 

    VectorXd time(Path.rows() - 1);

    // The time allocation is many relative timelines but not one common timeline
    for(int i = 0; i < time.rows(); i++)
    {
        double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
        double x1 = _Vel * _Vel / (2 * _Acc); 
        double x2 = distance - 2 * x1;
        if(x2<=0){
            time(i) = 2*sqrt(distance/_Acc);
        }
        else{
            double t1 = _Vel / _Acc;
            double t2 = x2 / _Vel;
            time(i) = 2 * t1 + t2;;
        }
        //time(i) = distance/_Vel;
    }
    return time;
}


nav_msgs::Path vector3d_to_waypoints(vector<Vector3d> path)
{
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;

    for (auto ptr: path)
    {
        pt.pose.position.y =  ptr(1);
        pt.pose.position.x =  ptr(0);
        pt.pose.position.z =  ptr(2);
        waypoints.poses.push_back(pt);//维护waypoints
    }
    return waypoints;
}


void visCorridor(FlightCorridor* cor,int flag)
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    

    node_vis.color.a = 0.2;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    
    // if(is_use_jps)
    //     node_vis.ns = "demo_node/jps_path";
    // else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    

    // if(is_use_jps){
    //     node_vis.color.a = 1.0;
    //     node_vis.color.r = 1.0;
    //     node_vis.color.g = 0.0;
    //     node_vis.color.b = 0.0;
    // }
    // else{
    //     node_vis.color.a = 1.0;
    //     node_vis.color.r = 1.0;
    //     node_vis.color.g = 1.0;
    //     node_vis.color.b = 1.0;
    // }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;
    geometry_msgs::Point pt;

    ROS_INFO("_corridor  size=%d ",cor->cubes.size());
    for(int i = 0; i < int(cor->cubes.size()); i++)
    {
        FlightCube cube=cor->cubes[i];
        for (int i=cube.start_node->index[0]-cube.x_neg_int;i<=cube.start_node->index[0]+cube.x_pos_int;i++)
            for (int j=cube.start_node->index[1]-cube.y_neg_int;j<=cube.start_node->index[1]+cube.y_pos_int;j++)
                for (int k=cube.start_node->index[2]-cube.z_neg_int;k<=cube.start_node->index[2]+cube.z_pos_int;k++)
                {
                    // ROS_INFO("i=%d  j=%d   k=%d ",i,j,k);
                    Vector3i index(i,j,k);
                    Vector3d coord = cor->gridIndex2coord(index);
                    pt.x = coord(0);
                    pt.y = coord(1);
                    pt.z = coord(2);
                    node_vis.points.push_back(pt);
                }
    }

    if (flag==1)
        _corridor_pub.publish(node_vis);
    else if (flag==2)
        _corridor_pub2.publish(node_vis);
}





