#ifndef CUDA_TRAJECTORY_HPP_
#define CUDA_TRAJECTORY_HPP_

//for the array of trajectory points
//number of CuTrajPtsArray = ?
//fix array size, hope the usage is less than that
class CuTrajPts{
  public:
    double x_pt;
    double y_pt;
    double th_pt;
};
//for the best trajectory
class CuTrajectory{
  public:
    CuTrajectory(){
      cost_=-1;
      pts=NULL;
    }
    CuTrajectory(double xv_, double yv_, double thetav_,
     double cost_, double time_delta_, int pts_size){
       this->xv_=xv_;
       this->yv_=yv_;
       this->thetav_=thetav_;
       this->cost_=cost_;
       this->time_delta_=time_delta_;
       this->pts_size=pts_size;

       pts= new CuTrajPts[pts_size];
    }
    ~CuTrajectory(){
      delete[] pts;
      pts=NULL;
    }
    void replace(double xv_, double yv_, double thetav_,
     double cost_, double time_delta_){
       this->xv_=xv_;
       this->yv_=yv_;
       this->thetav_=thetav_;
       this->cost_=cost_;
       this->time_delta_=time_delta_;
    }
    void newPts(int pts_size){
      this->pts_size=pts_size;
      delete [] pts;
      pts= new CuTrajPts[pts_size];
    }
    double xv_, yv_, thetav_; ///< @brief The x, y, and theta velocities of the trajectory
    double cost_; ///< Value to compare. The coy, vthetast/score of the trajectory
    double time_delta_; ///< @brief The time gap between points
    
    CuTrajPts* pts;
    int pts_size;
};

//number of CuTrajIn = number of samples
//need host->device
//don't need device->host
class CuTrajIn{
  public:
    unsigned long index; //address in PtsArray
    int num_steps; //size of PtsArray
};


//number of CuTrajOut = number of samples
//don't need host->device
//need device->host
class CuTrajOut{
  public:
    double xv_, yv_, thetav_; ///< @brief The x, y, and theta velocities of the trajectory
    double cost_; ///< Value to compare. The coy, vthetast/score of the trajectory
    double time_delta_; ///< @brief The time gap between points
};


#endif