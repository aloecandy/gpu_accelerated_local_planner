#include <cuda_runtime.h>

#include "cuda/cuda_trajectory_planner.hpp"
#include <stdio.h>
//#include <string>
#include "cuda/cuda_line_iterator.hpp"
//#include <algorithm>


void catchErr(cudaError_t err,char* act){
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to %s (error code %s)!\n", act, cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
}


__device__ double cuFootprintCost(double x_i,double y_i, double theta_i ,CuTrajectoryPlanner *ctp,CuTrajectoryPlannerS *ctps){
    double footprint_cost= 0.0;
    double cos_th = cos(theta_i);
    double sin_th = sin(theta_i);


    CuPoint* oriented_footprint = new CuPoint[ctps->footprint_size];
    for(int i=0;i<ctps->footprint_size;i++){
        oriented_footprint[i].x =x_i + (ctps->footprint_spec_[i].x * cos_th - ctps->footprint_spec_[i].y * sin_th);
        oriented_footprint[i].y =y_i + (ctps->footprint_spec_[i].x * sin_th + ctps->footprint_spec_[i].y * cos_th);
    }
    
    //used to put things into grid coordinates
    unsigned int cell_x, cell_y;
    //get the cell coord of the center point of the robot
    if(!ctp->costmap_->worldToMap(x_i,y_i,cell_x,cell_y)) return -1.0;

    //if number of points in the footprint is less than 3, we'll just assume a circular robot
    if(ctps->footprint_size < 3){
        unsigned char cost = ctp->costmap_->getCost(cell_x, cell_y);
        //if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
        if(cost == 254 || cost == 253 || cost == 255)
            return -1.0;
        return cost;
    }
  
    //now we really have to lay down the footprint in the costmap grid
    unsigned int x0,x1,y0,y1;
    double line_cost = 0.0;
    footprint_cost = 0.0;
    double point_cost;
    //we need to rasterize each line in the footprint
    for(unsigned int i=0;i<ctps->footprint_size-1;i++){
        //get the cell coord of the first point
        if(!ctp->costmap_->worldToMap(oriented_footprint[i].x,oriented_footprint[i].y,x0,y0)){
            delete[] oriented_footprint;
            return -1.0;
        }
        //get the cell coord of the second point
        if(!ctp->costmap_->worldToMap(oriented_footprint[i+1].x,oriented_footprint[i+1].y,x1,y1)){
            delete[] oriented_footprint;
            return -1.0;
        }

        //line_cost = lineCost(x0, x1, y0, y1);
        point_cost= -1.0;
        for(LineIterator line(x0,y0,x1,y1);line.isValid();line.advance()){
            unsigned char cost = ctp->costmap_->getCost(line.getX(),line.getY());
            if(cost==254||cost==255)    point_cost=-1;
            else point_cost = (double)cost;
            
            if(point_cost<0){
                line_cost = -1;
                break;
            }
            if(line_cost<point_cost){
                line_cost=point_cost;
            }
        }//line_cost = lineCost(x0, x1, y0, y1);////////
        
        footprint_cost = max(line_cost,footprint_cost);
        
        //if there is an obstacle that hits the line... we know that we can return false right away 
        if(line_cost<0){
            delete[] oriented_footprint;
            return -1.0;
        }
    }
    //we also need to connect the first point in the footprint to the last point
    //get the cell coord of the last point
    if(!ctp->costmap_->worldToMap(oriented_footprint[ctps->footprint_size-1].x,oriented_footprint[ctps->footprint_size-1].y,x0,y0)){
        delete[] oriented_footprint;
        return -1.0;
    }
    //get the cell coord of the first point
    if(!ctp->costmap_->worldToMap(oriented_footprint[0].x,oriented_footprint[0].y,x1,y1)){
        delete[] oriented_footprint;
        return -1.0;    
    }
        
    //line_cost = lineCost(x0, x1, y0, y1);
    line_cost = 0.0;
    point_cost= -1.0;
    for(LineIterator line(x0,y0,x1,y1);line.isValid();line.advance()){
        unsigned char cost = ctp->costmap_->getCost(line.getX(),line.getY());
        if(cost==254||cost==255)    point_cost=-1;
        else point_cost = (double)cost;
        
        if(point_cost<0){
            line_cost = -1;
            break;
        }
        if(line_cost<point_cost){
            line_cost=point_cost;
        }
    }
    footprint_cost = max(line_cost,footprint_cost);
    if(line_cost<0){
        delete[] oriented_footprint;
        return -1.0;
    }
    delete[] oriented_footprint;
    return footprint_cost;
}
__device__ double cuHeadingDiff(int cell_x,int cell_y,double x,double y,double heading,CuTrajectoryPlanner *ctp,CuTrajectoryPlannerS *ctps){
    unsigned int goal_cell_x, goal_cell_y;
    
    for(int i= ctp->global_plan_size-1;i>=0;i--){
        if(ctp->costmap_->worldToMap(ctp->global_plan_[i].x,ctp->global_plan_[i].y,goal_cell_x,goal_cell_y)){
            
            //line_cost = lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y);
            double line_cost = 0.0;
            double point_cost= -1.0;
            for(LineIterator line(cell_x, goal_cell_x, cell_y, goal_cell_y);line.isValid();line.advance()){
                unsigned char cost = ctp->costmap_->getCost(line.getX(),line.getY());
                if(cost==254||cost==255)    point_cost=-1;
                else point_cost = (double)cost;
                
                if(point_cost<0){
                    line_cost = -1;
                    break;
                }
                if(line_cost<point_cost){
                    line_cost=point_cost;
                }
            }//line_cost = lineCost(x0, x1, y0, y1);////////

            if(line_cost>=0){
                double gx,gy;
                ctp->costmap_->mapToWorld(goal_cell_x,goal_cell_y,gx,gy);
                
                double angle = atan2(gy-y,gx-x)-heading;

                double a = fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
                if (a> M_PI)
                    a-=2.0*M_PI;
                
                return fabs(a);

            }
        }
    }
    return double(1.7976931348623157e+308);//DBL_MAX
}
__device__ double cuComputeNewVelocity(double vg, double vi, double a_max, double dt){
    if((vg - vi) >= 0) {
        return min(vg, vi + a_max * dt);
    }
    return max(vg, vi - a_max * dt);
}
__device__ double cuComputeNewXPosition(double xi, double vx, double vy, double theta, double dt){
    return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
}
__device__ double cuComputeNewYPosition(double yi, double vx, double vy, double theta, double dt){
    return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
}
__device__ double cuComputeNewThetaPosition(double thetai, double vth, double dt){
    return thetai + vth * dt;
}

__global__ void cuGenerateTrajectory(CuTrajectoryPlanner *ctp,CuTrajectoryPlannerS *ctps, CuParams *p,CuTrajIn *tIn, CuTrajOut *tOut, CuTrajPts* pts,int size){
    int ii = threadIdx.x + blockIdx.x * blockDim.x;
    if(ii<size){    
        double x_i = ctp->x;
        double y_i = ctp->y;
        double theta_i = ctp->theta;

        double vx_i, vy_i, vtheta_i;

        vx_i = ctp->vx;
        vy_i = ctp->vy;
        vtheta_i = ctp->vtheta;
        
        double dt= ctps->sim_time_/tIn[ii].num_steps;
        double time=0.0;

        tOut[ii].xv_=p[ii].vx_samp;
        tOut[ii].yv_=p[ii].vy_samp;
        tOut[ii].thetav_=p[ii].vtheta_samp;
        tOut[ii].cost_=-1.0;
        double path_dist =0.0;
        double goal_dist =0.0;
        double occ_cost =0.0;
        double heading_diff =0.0;
        for(int i=0;i<tIn[ii].num_steps;i++){
        
            unsigned int cell_x, cell_y;
            if(!ctp->costmap_->worldToMap(x_i,y_i,cell_x,cell_y)){
                tOut[ii].cost_=-1.0;
                return;   
            }
            
        
            double footprint_cost = cuFootprintCost(x_i,y_i,theta_i,ctp,ctps);
            if(footprint_cost<0){
                tOut[ii].cost_=-1.0;
                return;
            }
            occ_cost = max(max(occ_cost,footprint_cost),double(ctp->costmap_->getCost(cell_x,cell_y)));
            if(ctps->simple_attractor_){
                goal_dist = (x_i-ctp->global_plan_[ctp->global_plan_size-1].x)*
                    (x_i-ctp->global_plan_[ctp->global_plan_size-1].x)+
                    (y_i-ctp->global_plan_[ctp->global_plan_size-1].y)*
                    (y_i-ctp->global_plan_[ctp->global_plan_size-1].y);
            }else{
                bool update_path_and_goal_distances = true;
                if(ctps->heading_scoring_){
                    if(time >= ctps->heading_scoring_timestep_ && time < ctps->heading_scoring_timestep_ + dt){
                        heading_diff = cuHeadingDiff(cell_x,cell_y,x_i,y_i,theta_i,ctp,ctps);
                    }else{
                        update_path_and_goal_distances = false;
                    }
                }
                if(update_path_and_goal_distances){
                    path_dist = ctp->path_map_->map_[ctp->path_map_->size_x_ * cell_y + cell_x].target_dist;
                    goal_dist = ctp->goal_map_->map_[ctp->goal_map_->size_x_ * cell_y + cell_x].target_dist;
                    if(ctp->impossible_cost <= goal_dist || ctp->impossible_cost <= path_dist ){
                        tOut[ii].cost_= -2.0;
                        return;
                    }
                }
            }
            int pp=tIn[ii].index+i;
            pts[pp].x_pt=x_i;
            pts[pp].y_pt=y_i;
            pts[pp].th_pt=theta_i;
            
            vx_i = cuComputeNewVelocity(p[ii].vx_samp,vx_i,ctps->acc_x,dt);
            vy_i= cuComputeNewVelocity(p[ii].vy_samp,vy_i,ctps->acc_y,dt);
            vtheta_i= cuComputeNewVelocity(p[ii].vtheta_samp,vtheta_i,ctps->acc_theta,dt);
            
            x_i = cuComputeNewXPosition(x_i,vx_i,vy_i,theta_i,dt);
            y_i = cuComputeNewYPosition(y_i,vx_i,vy_i,theta_i,dt);
            theta_i = cuComputeNewThetaPosition(theta_i,vtheta_i,dt);

            time += dt;
        }
        
        double cost = -1.0;
        if(!ctps->heading_scoring_){
            cost=ctps->pdist_scale_*path_dist+goal_dist*ctps->gdist_scale_+ctps->occdist_scale_*occ_cost;
        }else{
            cost=ctps->occdist_scale_*occ_cost+ctps->pdist_scale_*path_dist+0.3*heading_diff+goal_dist*ctps->gdist_scale_;
        }
        tOut[ii].cost_=cost;    
    }
}


void cuCreateTrajectories(CuTrajectoryPlanner *ctp,CuTrajectoryPlannerS *ctps,H_CuTrajectoryPlanner *h_ctp){
    int cSize;
    double bestCost=-1;
    int bestIndex=-1;

    CuParams* h_p;
    CuTrajIn* h_trajIn;
    CuTrajOut* h_trajOut;
    CuTrajPts* h_pts;
    CuParams* h_p_;
    CuTrajIn* h_trajIn_;
    CuTrajOut* h_trajOut_;
    CuTrajPts* h_pts_;
    if(!h_ctp->escaping_){
        cSize=h_ctp->vx_samples_*h_ctp->vtheta_samples_;     
        h_p = new CuParams[cSize];
        //호스트 -> 디바이스
        h_trajIn =new CuTrajIn[cSize];
        
        //결과 평가. 디바이스->호스트
        h_trajOut = new CuTrajOut[cSize];
        
        unsigned long index=0;
        for(int i=0; i< h_ctp->vx_samples_;i++){
            int j=0;
            int ii=i*h_ctp->vtheta_samples_+j;
            
            h_p[ii].vx_samp=h_ctp->min_vel_x+h_ctp->dvx*i;
            h_p[ii].vy_samp=0.0;
            h_p[ii].vtheta_samp=0;
            
            double vmag = hypot(h_p[ii].vx_samp, h_p[ii].vy_samp);
            if(!ctps->heading_scoring_){
                h_trajIn[ii].num_steps =int(
                max(
                 fabs(((vmag+ctp->vx)/2)*(fabs(vmag-ctp->vx)/ctps->acc_x)
                     +vmag*(ctps->sim_time_-fabs(vmag-ctp->vx)/ctps->acc_x))/ctps->sim_granularity_,
                  fabs(((h_p[ii].vtheta_samp+ctp->vtheta)/2)*(fabs(h_p[ii].vtheta_samp-ctp->vtheta)/ctps->acc_theta)
                    +h_p[ii].vtheta_samp*(ctps->sim_time_-fabs(h_p[ii].vtheta_samp-ctp->vtheta)/ctps->acc_theta))/ctps->angular_sim_granularity_)
                  +0.5
                  );
            }
            else{
                h_trajIn[ii].num_steps = int(ctps->sim_time_/ctps->sim_granularity_+0.5);
            }
            
            if(h_trajIn[ii].num_steps==0){
                h_trajIn[ii].num_steps=1;
            }
            h_trajIn[ii].index=index;
            index+=h_trajIn[ii].num_steps;
            for(j=1; j<h_ctp->vtheta_samples_;j++){
                ii=i*h_ctp->vtheta_samples_+j;
                h_p[ii].vx_samp=h_ctp->min_vel_x+h_ctp->dvx*i;
                h_p[ii].vy_samp=0.0;
                h_p[ii].vtheta_samp=h_ctp->min_vel_theta+h_ctp->dvtheta*(j-1);
                double vmag = hypot(h_p[ii].vx_samp, h_p[ii].vy_samp);
                if(!ctps->heading_scoring_){
                    h_trajIn[ii].num_steps =int(
                    max(
                      fabs(((vmag+ctp->vx)/2)*(fabs(vmag-ctp->vx)/ctps->acc_x)
                         +vmag*(ctps->sim_time_-fabs(vmag-ctp->vx)/ctps->acc_x))/ctps->sim_granularity_,
                      fabs(((h_p[ii].vtheta_samp+ctp->vtheta)/2)*(fabs(h_p[ii].vtheta_samp-ctp->vtheta)/ctps->acc_theta)
                        +h_p[ii].vtheta_samp*(ctps->sim_time_-fabs(h_p[ii].vtheta_samp-ctp->vtheta)/ctps->acc_theta))/ctps->angular_sim_granularity_)
                      +0.5
                      );
                }
                else{
                    h_trajIn[ii].num_steps = int(ctps->sim_time_/ctps->sim_granularity_+0.5);
                }
                if(h_trajIn[ii].num_steps==0){
                    h_trajIn[ii].num_steps=1;
                }
                h_trajIn[ii].index=index;
                index+=h_trajIn[ii].num_steps;
            }
        }
        
        h_pts=new CuTrajPts[index];
        //start
        CuParams* p;
        CuTrajIn* tIn;
        CuTrajOut* tOut;
        CuTrajPts* pts;

        
        

        cudaMalloc((void**)&p,sizeof(CuParams)*cSize);
        cudaMemcpy(p,h_p,sizeof(CuParams)*cSize,cudaMemcpyHostToDevice);
        cudaMalloc((void**)&tIn,sizeof(CuTrajIn)*cSize);
        cudaMemcpy(tIn,h_trajIn,sizeof(CuTrajIn)*cSize,cudaMemcpyHostToDevice);
        cudaMalloc((void**)&tOut,sizeof(CuTrajOut)*cSize);
        
        cudaMalloc((void**)&pts,sizeof(CuTrajPts)*index);
        int threadsPerBlock = 256;
        int blocksPerGrid =(cSize + threadsPerBlock - 1) / threadsPerBlock;
        
        cuGenerateTrajectory<<<blocksPerGrid,threadsPerBlock>>>(ctp->d_p,ctps->d_p,p,tIn,tOut,pts,cSize);
        cudaDeviceSynchronize();
        catchErr(cudaGetLastError(),"launch kernel function");
        
        catchErr(cudaMemcpy(h_trajOut,tOut,sizeof(CuTrajOut)*cSize,cudaMemcpyDeviceToHost),"tOut");
        catchErr(cudaMemcpy(h_pts,pts,sizeof(CuTrajPts)*index,cudaMemcpyDeviceToHost),"pts");
        
        //end
        
        for(int i=0;i<cSize;i++){
            //printf("[%d] %f\n",i,h_trajOut[i].cost_);
            if(h_trajOut[i].cost_ >=0 &&(h_trajOut[i].cost_<bestCost||bestCost<0)){
                bestCost=h_trajOut[i].cost_;
                bestIndex=i;
            }
        }
        /*
        if(bestIndex!=-1){
            h_ctp->best_traj->newPts(h_trajIn[bestIndex].num_steps);
            //printf("%d,%d\n",h_trajIn[bestIndex].num_steps,h_ctp->best_traj->pts_size);
            h_ctp->best_traj->replace(h_trajOut[bestIndex].xv_,h_trajOut[bestIndex].yv_,h_trajOut[bestIndex].thetav_,h_trajOut[bestIndex].cost_,h_trajOut[bestIndex].time_delta_);
            for(int i=0;i<h_ctp->best_traj->pts_size;i++){
                //printf("[%d] %f %f %f\n", i, pts[h_trajIn[bestIndex].index+i].x_pt,pts[h_trajIn[bestIndex].index+i].y_pt,pts[h_trajIn[bestIndex].index+i].th_pt);
                h_ctp->best_traj->pts[i].x_pt=h_pts[h_trajIn[bestIndex].index+i].x_pt;
                h_ctp->best_traj->pts[i].y_pt=h_pts[h_trajIn[bestIndex].index+i].y_pt;
                h_ctp->best_traj->pts[i].th_pt=h_pts[h_trajIn[bestIndex].index+i].th_pt;
            }
        }*/
        
        
        /*delete[] h_pts;
        delete[] h_p;
        delete[] h_trajIn;
        delete[] h_trajOut;*/
        cudaFree(p);
        cudaFree(tIn);
        cudaFree(tOut);
        cudaFree(pts);

        
    }
    //next we want to generate trajectories for rotating in place
    cSize=h_ctp->vtheta_samples_;
    h_p_ = new CuParams[cSize];
    //호스트 -> 디바이스
    h_trajIn_ =new CuTrajIn[cSize];
    
    //결과 평가. 디바이스->호스트
    h_trajOut_ = new CuTrajOut[cSize];
    
    double vtheta_samp= h_ctp->min_vel_theta;

    unsigned long index=0;
    for(int ii =0; ii<h_ctp->vtheta_samples_;ii++){
        double vtheta_samp_limited = vtheta_samp>0 ? max(vtheta_samp,h_ctp->min_in_place_vel_th_):min(vtheta_samp,-1.0*h_ctp->min_in_place_vel_th_);

        h_p_[ii].vx_samp=0.0;
        h_p_[ii].vy_samp=0.0;
        h_p_[ii].vtheta_samp=vtheta_samp_limited;
        double vmag = hypot(h_p_[ii].vx_samp, h_p_[ii].vy_samp);
        
        if(!ctps->heading_scoring_){
            h_trajIn_[ii].num_steps =int(
            max(
              fabs(((vmag+ctp->vx)/2)*(fabs(vmag-ctp->vx)/ctps->acc_x)
                 +vmag*(ctps->sim_time_-fabs(vmag-ctp->vx)/ctps->acc_x))/ctps->sim_granularity_,
              fabs(((h_p_[ii].vtheta_samp+ctp->vtheta)/2)*(fabs(h_p_[ii].vtheta_samp-ctp->vtheta)/ctps->acc_theta)
                +h_p_[ii].vtheta_samp*(ctps->sim_time_-fabs(h_p_[ii].vtheta_samp-ctp->vtheta)/ctps->acc_theta))/ctps->angular_sim_granularity_)
              +0.5
              );
        }
        else{
            h_trajIn_[ii].num_steps = int(ctps->sim_time_/ctps->sim_granularity_+0.5);
        }
        
        if(h_trajIn_[ii].num_steps==0){
            h_trajIn_[ii].num_steps=1;
        }
        h_trajIn_[ii].index=index;
        index+=h_trajIn_[ii].num_steps;
        
        vtheta_samp+=h_ctp->dvtheta;
    }
    
    h_pts_=new CuTrajPts[index];

    //start
    CuParams* p;
    CuTrajIn* tIn;
    CuTrajOut* tOut;
    CuTrajPts* pts;

    
    cudaMalloc((void**)&p,sizeof(CuParams)*cSize);
    cudaMemcpy(p,h_p_,sizeof(CuParams)*cSize,cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tIn,sizeof(CuTrajIn)*cSize);
    cudaMemcpy(tIn,h_trajIn_,sizeof(CuTrajIn)*cSize,cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tOut,sizeof(CuTrajOut)*cSize);
    
    cudaMalloc((void**)&pts,sizeof(CuTrajPts)*index);
    int threadsPerBlock = 256;
    int blocksPerGrid =(cSize + threadsPerBlock - 1) / threadsPerBlock;
    
    cuGenerateTrajectory<<<blocksPerGrid,threadsPerBlock>>>(ctp->d_p,ctps->d_p,p,tIn,tOut,pts,cSize);
    cudaDeviceSynchronize();
    catchErr(cudaGetLastError(),"launch kernel function");
    
    catchErr(cudaMemcpy(h_trajOut_,tOut,sizeof(CuTrajOut)*cSize,cudaMemcpyDeviceToHost),"tOut");
    catchErr(cudaMemcpy(h_pts_,pts,sizeof(CuTrajPts)*index,cudaMemcpyDeviceToHost),"pts");
    
    //end
        
    
    int ii=-1;//bestIndex
    CuTrajectory* tmpTraj=h_ctp->best_traj;
    //let's try to rotate toward open space
    double heading_dist=(double)1.7976931348623157e+308;//DBL_MAX
    bool changed=false;
    vtheta_samp= h_ctp->min_vel_theta;
    for(int i=0;i<cSize;i++){
        //printf("[%d] %f\n",i,h_trajOut_[i].cost_);
        //if the new trajectory is better... let's take it...
        //note if we can legally rotate in place we prefer to do that rather than move with y velocity
        if(h_trajOut_[i].cost_ >=0 &&
          (h_trajOut_[i].cost_<= bestCost || bestCost <0)&&
          (vtheta_samp > h_ctp->dvtheta || vtheta_samp < -1*h_ctp->dvtheta)){
            double x_r, y_r, th_r;
            if(!changed){
                if(bestIndex==-1){
                    x_r = tmpTraj->pts[tmpTraj->pts_size-1].x_pt;
                    y_r = tmpTraj->pts[tmpTraj->pts_size-1].y_pt;
                    th_r = tmpTraj->pts[tmpTraj->pts_size-1].th_pt;
                }
                else{
                    int ep = h_trajIn[bestIndex].index+h_trajIn[bestIndex].num_steps -1;
                    x_r = h_pts[ep].x_pt;
                    y_r = h_pts[ep].y_pt;
                    th_r = h_pts[ep].th_pt;
                }
            }
            else{
                int ep= h_trajIn_[ii].index+ h_trajIn_[ii].num_steps -1;
                x_r = h_pts_[ep].x_pt;
                y_r = h_pts_[ep].y_pt;
                th_r = h_pts_[ep].th_pt;
            }
            x_r += h_ctp->heading_lookahead_ * cos(th_r);
            y_r += h_ctp->heading_lookahead_ * sin(th_r);

            unsigned int cell_x, cell_y;

            if(ctp->costmap_->worldToMap(x_r,y_r,cell_x,cell_y)){
                double ahead_gdist = ctp->goal_map_->map_[ctp->goal_map_->size_x_ * cell_y + cell_x].target_dist;
                if( ahead_gdist < heading_dist){
                    if(vtheta_samp<0 && !h_ctp->stuck_left){                                
                        changed=true;
                        bestCost=h_trajOut_[i].cost_;
                        ii=i;
                        heading_dist=ahead_gdist;
                    }
                    else if(vtheta_samp>0 && !h_ctp->stuck_right){
                        changed=true;
                        bestCost=h_trajOut_[i].cost_;
                        ii=i;
                        heading_dist=ahead_gdist;
                    }
                }
            }
        }
        vtheta_samp+=h_ctp->dvtheta;
    }
    if(ii!=-1){
        //printf("rotating\n");
        h_ctp->best_traj->newPts(h_trajIn_[ii].num_steps);
        //printf("%d,%d\n",h_trajIn_[ii].num_steps,h_ctp->best_traj->pts_size);
        h_ctp->best_traj->replace(h_trajOut_[ii].xv_,h_trajOut_[ii].yv_,h_trajOut_[ii].thetav_,h_trajOut_[ii].cost_,h_trajOut_[ii].time_delta_);
        for(int i=0;i<h_ctp->best_traj->pts_size;i++){
            //printf("[%d] %f %f %f\n", i, pts[h_trajIn_[ii].index+i].x_pt,pts[h_trajIn_[ii].index+i].y_pt,pts[h_trajIn_[ii].index+i].th_pt);
            h_ctp->best_traj->pts[i].x_pt=h_pts_[h_trajIn_[ii].index+i].x_pt;
            h_ctp->best_traj->pts[i].y_pt=h_pts_[h_trajIn_[ii].index+i].y_pt;
            h_ctp->best_traj->pts[i].th_pt=h_pts_[h_trajIn_[ii].index+i].th_pt;
        }
    }
    else{
        if(bestIndex!=-1){
            //printf("forward\n");
            h_ctp->best_traj->newPts(h_trajIn[bestIndex].num_steps);
            //printf("%d,%d\n",h_trajIn[bestIndex].num_steps,h_ctp->best_traj->pts_size);
            h_ctp->best_traj->replace(h_trajOut[bestIndex].xv_,h_trajOut[bestIndex].yv_,h_trajOut[bestIndex].thetav_,h_trajOut[bestIndex].cost_,h_trajOut[bestIndex].time_delta_);
            for(int i=0;i<h_ctp->best_traj->pts_size;i++){
                //printf("[%d] %f %f %f\n", i, pts[h_trajIn[bestIndex].index+i].x_pt,pts[h_trajIn[bestIndex].index+i].y_pt,pts[h_trajIn[bestIndex].index+i].th_pt);
                h_ctp->best_traj->pts[i].x_pt=h_pts[h_trajIn[bestIndex].index+i].x_pt;
                h_ctp->best_traj->pts[i].y_pt=h_pts[h_trajIn[bestIndex].index+i].y_pt;
                h_ctp->best_traj->pts[i].th_pt=h_pts[h_trajIn[bestIndex].index+i].th_pt;
            }
        }
        else{
            //printf("not found\n");
            h_ctp->best_traj->newPts(h_trajIn_[cSize-1].num_steps);
            //printf("%d,%d\n",h_trajIn_[cSize-1].num_steps,h_ctp->best_traj->pts_size);
            h_ctp->best_traj->replace(h_trajOut_[cSize-1].xv_,h_trajOut_[cSize-1].yv_,h_trajOut_[cSize-1].thetav_,h_trajOut_[cSize-1].cost_,h_trajOut_[cSize-1].time_delta_);
            for(int i=0;i<h_ctp->best_traj->pts_size;i++){
                //printf("[%d] %f %f %f\n", i, pts[h_trajIn_[cSize-1].index+i].x_pt,pts[h_trajIn_[cSize-1].index+i].y_pt,pts[h_trajIn_[cSize-1].index+i].th_pt);
                h_ctp->best_traj->pts[i].x_pt=h_pts_[h_trajIn_[cSize-1].index+i].x_pt;
                h_ctp->best_traj->pts[i].y_pt=h_pts_[h_trajIn_[cSize-1].index+i].y_pt;
                h_ctp->best_traj->pts[i].th_pt=h_pts_[h_trajIn_[cSize-1].index+i].th_pt;
            }
        }
    }
    delete[] h_pts;
    delete[] h_p;
    delete[] h_trajIn;
    delete[] h_trajOut;

    delete[] h_pts_;
    delete[] h_p_;
    delete[] h_trajIn_;
    delete[] h_trajOut_;

    cudaFree(p);
    cudaFree(tIn);
    cudaFree(tOut);
    cudaFree(pts);
}
