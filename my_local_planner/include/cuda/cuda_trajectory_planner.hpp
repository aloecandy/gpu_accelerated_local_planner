#ifndef CUDA_TRAJECTORY_PLANNER_HPP_
#define CUDA_TRAJECTORY_PLANNER_HPP_
#include <cuda_runtime.h>

#include "cuda/cuda_costmap_2d.hpp"
#include "cuda/cuda_map_grid.hpp"
#include "cuda/cuda_point.hpp"
#include "cuda/cuda_trajectory.hpp"
//generateTrajectory 호출마다 바뀌는 인수와 안 바뀌는 인수 분리해야함

class CuTrajectoryPlanner{
  public:
    CuTrajectoryPlanner(){
      path_map_=new CuMapGrid();
      goal_map_=new CuMapGrid();
      costmap_=new CuCostmap2D();
      d_p=NULL;
    }
    CuTrajectoryPlanner(double x, double y, double theta, double vx, double vy, double vtheta, 
        double impossible_cost,
        CuPoint* global_plan_, int global_plan_size,
        CuMapGrid* path_map_, CuMapGrid* goal_map_, CuCostmap2D* costmap_){
      this->x = x; //volatile
      this->y = y; //volatile
      this->theta = theta; //volatile
      this->vx = vx; //volatile
      this->vy = vy; //volatile
      this->vtheta = vtheta; //static
      this->path_map_=path_map_; //volatile, except size
      this->goal_map_=goal_map_; //volatile, except size
      this->global_plan_=global_plan_; //volatile
      this->global_plan_size=global_plan_size; //volatile
      
      this->impossible_cost = impossible_cost;//static
      this->costmap_=costmap_; //static
    }
    void replace(double x, double y, double theta, double vx, double vy, double vtheta, 
        double impossible_cost,
        CuPoint* global_plan_, int global_plan_size){
      this->x = x; //volatile
      this->y = y; //volatile
      this->theta = theta; //volatile
      this->vx = vx; //volatile
      this->vy = vy; //volatile
      this->vtheta = vtheta; //static
      this->global_plan_=global_plan_; //volatile
      this->global_plan_size=global_plan_size; //volatile
      
      this->impossible_cost = impossible_cost;//static
    }
    double x, y, theta, vx, vy, vtheta, impossible_cost;
    
    CuPoint* global_plan_; //contains only position
    int global_plan_size;
    
    CuMapGrid* path_map_;
    CuMapGrid* goal_map_;

    CuCostmap2D* costmap_;

    //for device
    CuTrajectoryPlanner* d_p;
    CuPoint* d_global_plan_; //contains only position
    


    void cuInit(){
      //backup addresses
      CuPoint* tmp_global_plan_=global_plan_;
      CuMapGrid* tmp_path_map_=path_map_;
      CuMapGrid* tmp_goal_map_=goal_map_;
      CuCostmap2D* tmp_costmap_=costmap_;
      
      //host-->device///////////////////////////////
      cudaMalloc((void**)&d_global_plan_,global_plan_size*sizeof(CuPoint));
      cudaMemcpy(d_global_plan_,global_plan_,global_plan_size*sizeof(CuPoint),cudaMemcpyHostToDevice);
      path_map_->cuInit();
      goal_map_->cuInit();
      costmap_->cuInit();

      global_plan_=d_global_plan_;
      path_map_=path_map_->d_p;
      goal_map_=goal_map_->d_p;
      costmap_=costmap_->d_p;

      cudaMalloc((void**)&d_p,sizeof(CuTrajectoryPlanner));
      cudaMemcpy(d_p,this,sizeof(CuTrajectoryPlanner),cudaMemcpyHostToDevice);
      /////////////////////////////host-->device////


      //restore addresses
      global_plan_=tmp_global_plan_;
      path_map_=tmp_path_map_;
      goal_map_=tmp_goal_map_;
      costmap_=tmp_costmap_;
    }
    void cuCopy(){
      //backup addresses
      CuPoint* tmp_global_plan_=global_plan_;
      CuMapGrid* tmp_path_map_=path_map_;
      CuMapGrid* tmp_goal_map_=goal_map_;
      CuCostmap2D* tmp_costmap_=costmap_;
      
      cudaFree(d_global_plan_);
      cudaMalloc((void**)&d_global_plan_,global_plan_size*sizeof(CuPoint));
      cudaMemcpy(d_global_plan_,global_plan_,global_plan_size*sizeof(CuPoint),cudaMemcpyHostToDevice);
      path_map_->cuCopy();
      goal_map_->cuCopy();
      costmap_->cuCopy();

      global_plan_=d_global_plan_;
      path_map_=path_map_->d_p;
      goal_map_=goal_map_->d_p;
      costmap_=costmap_->d_p;

      cudaMemcpy(d_p,this,sizeof(CuTrajectoryPlanner),cudaMemcpyHostToDevice);
      
      //restore addresses
      global_plan_=tmp_global_plan_;
      path_map_=tmp_path_map_;
      goal_map_=tmp_goal_map_;
      costmap_=tmp_costmap_;
    }
    void cuFree(){
      cudaFree(d_global_plan_);
      path_map_->cuFree();
      goal_map_->cuFree();
      costmap_->cuFree();
    }
    ~CuTrajectoryPlanner(){
      cuFree();
      delete path_map_;
      delete goal_map_;
      delete costmap_;
    }
};
class CuTrajectoryPlannerS{
  public:
    CuTrajectoryPlannerS(){}
    void replace(double acc_x, double acc_y, double acc_theta,
        double sim_time_, double sim_granularity_, double angular_sim_granularity_, 
        double pdist_scale_, double gdist_scale_, double occdist_scale_, 
        double heading_scoring_timestep_,
        bool heading_scoring_, bool simple_attractor_,
        double inscribed_radius_, double circumscribed_radius_,
        CuPoint* footprint_spec_,int footprint_size){
      this->acc_x = acc_x; //static
      this->acc_y = acc_y; //static
      this->acc_theta = acc_theta; //static
      
      //this->impossible_cost = impossible_cost;//static

      this->sim_time_ = sim_time_; //static
      this->sim_granularity_ = sim_granularity_;//static 
      this->angular_sim_granularity_ = angular_sim_granularity_;//static
      
      this->pdist_scale_ = pdist_scale_; //static
      this->gdist_scale_ = gdist_scale_; //static
      this->occdist_scale_ = occdist_scale_; //static
      
      this->heading_scoring_timestep_ = heading_scoring_timestep_;//static
      this->heading_scoring_=heading_scoring_; //static
      this->simple_attractor_=simple_attractor_; //static
      
      this->inscribed_radius_=inscribed_radius_; //static
      this->circumscribed_radius_=circumscribed_radius_; //static
      
      this->footprint_spec_=footprint_spec_; //static
      this->footprint_size=footprint_size; //static
      //this->costmap_=costmap_; //static
    }
    void cuInit(){
      //backup address
      CuPoint* tmp=footprint_spec_;

      cudaMalloc((void**)&d_footprint_spec_,footprint_size*sizeof(CuPoint));
      cudaMemcpy(d_footprint_spec_,footprint_spec_,footprint_size*sizeof(CuPoint),cudaMemcpyHostToDevice);
      footprint_spec_=d_footprint_spec_;

      cudaMalloc((void**)&d_p, sizeof(CuTrajectoryPlannerS));
      cudaMemcpy(d_p,this,sizeof(CuTrajectoryPlannerS),cudaMemcpyHostToDevice);

      //restore address
      footprint_spec_=tmp;
    }
    double acc_x, acc_y, acc_theta, impossible_cost;
    double sim_time_, sim_granularity_, angular_sim_granularity_;
    double pdist_scale_, gdist_scale_, occdist_scale_, heading_scoring_timestep_;
    bool heading_scoring_, simple_attractor_;
    double inscribed_radius_, circumscribed_radius_;

    CuPoint* footprint_spec_;
    int footprint_size;

    //for device
    CuTrajectoryPlannerS* d_p;
    CuPoint* d_footprint_spec_;    

    //CuCostmap2D* costmap_;
};

//for host function    
class H_CuTrajectoryPlanner{
  public:
    H_CuTrajectoryPlanner(){
      best_traj=new CuTrajectory();
    }
    H_CuTrajectoryPlanner(int vx_samples_,int vtheta_samples_,
        double min_vel_x,double min_vel_theta,double min_in_place_vel_th_,
        double dvx, double dvtheta,double heading_lookahead_,
        bool escaping_,bool stuck_left, bool stuck_right){
      this->vx_samples_=vx_samples_;
      this->vtheta_samples_=vtheta_samples_;
      this->min_vel_x=min_vel_x;
      this->min_vel_theta=min_vel_theta;
      this->min_in_place_vel_th_=min_in_place_vel_th_;
      this->dvx=dvx;
      this->dvtheta=dvtheta;
      this->heading_lookahead_=heading_lookahead_;
      this->escaping_=escaping_;
      this->stuck_left=stuck_left; 
      this->stuck_right=stuck_right;
    }
    void replace(int vx_samples_,int vtheta_samples_,
        double min_vel_x,double min_vel_theta,double min_in_place_vel_th_,
        double dvx, double dvtheta,double heading_lookahead_,
        bool escaping_,bool stuck_left, bool stuck_right){
      this->vx_samples_=vx_samples_;
      this->vtheta_samples_=vtheta_samples_;
      this->min_vel_x=min_vel_x;
      this->min_vel_theta=min_vel_theta;
      this->min_in_place_vel_th_=min_in_place_vel_th_;
      this->dvx=dvx;
      this->dvtheta=dvtheta;
      this->heading_lookahead_=heading_lookahead_;
      this->escaping_=escaping_;
      this->stuck_left=stuck_left; 
      this->stuck_right=stuck_right;
    }
    //ros->cuda_host
    int vx_samples_; 
    int vtheta_samples_;
    double min_vel_x, min_vel_theta,min_in_place_vel_th_;
    double dvx, dvtheta;
    double heading_lookahead_;
    bool escaping_;
    bool stuck_left, stuck_right;
    CuTrajectory* best_traj;
};

class CuParams{
  public:
    double vx_samp;
    double vy_samp;
    double vtheta_samp;
};
void cuCreateTrajectories(CuTrajectoryPlanner *ctp,CuTrajectoryPlannerS *ctps,H_CuTrajectoryPlanner *h_ctp);


CuTrajectoryPlannerS* ctps;
CuTrajectoryPlanner* ctp;

H_CuTrajectoryPlanner *h_ctp;


#endif