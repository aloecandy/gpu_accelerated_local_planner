#ifndef CUDA_COSTMAP_2D_HPP_
#define CUDA_COSTMAP_2D_HPP_
#include <cuda_runtime.h>

#include  <string.h>
//from costmap_2d

class CuCostmap2D{
    public:
        unsigned int size_x_;
        unsigned int size_y_;
        double resolution_;
        double origin_x_;
        double origin_y_;
        unsigned char* costmap_;
        unsigned char default_value_;
        
        CuCostmap2D* d_p;
        unsigned char* d_costmap_;
        
        CuCostmap2D(){
            d_p=NULL;
            d_costmap_=NULL;
        }
        CuCostmap2D(void* ros_costmap){
            ros_costmap = (void *)ros_costmap + 16;
            memcpy(this,ros_costmap,48);
            d_p=NULL;
            d_costmap_=NULL;
        }
        void replace(void* ros_costmap){
            ros_costmap = (void *)ros_costmap + 16;
            memcpy(this,ros_costmap,48);
        }
        __device__ __host__ void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy){
            wx = origin_x_ + (mx + 0.5) * resolution_;
            wy = origin_y_ + (my + 0.5) * resolution_;
        }
        __device__ __host__ bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my){
            if( wx < origin_x_ || wy < origin_y_ )  return false;
            mx = (int)((wx - origin_x_)/resolution_);
            my = (int)((wy - origin_y_)/resolution_);
            if (mx < size_x_ && my < size_y_)   return true;
            return false;
        }
        __device__ __host__ unsigned int getIndex(unsigned int mx, unsigned int my) const{
            return my * size_x_ + mx;
        }
        __device__ __host__ unsigned char getCost(unsigned int mx, unsigned int my) const{
            return costmap_[getIndex(mx,my)];
        }
        void cuInit(){ //malloc and memcpy
            //backup address
            unsigned char* tmp = costmap_;
            
            
            cudaMalloc((void**)&d_costmap_,size_x_*size_y_*sizeof(unsigned char));
            cudaMemcpy(d_costmap_,costmap_,size_x_*size_y_*sizeof(unsigned char),cudaMemcpyHostToDevice);
            costmap_=d_costmap_;
            
            cudaMalloc((void**)&d_p,sizeof(CuCostmap2D));
            cudaMemcpy(d_p,this,sizeof(CuCostmap2D),cudaMemcpyHostToDevice);

            //restore address
            costmap_=tmp;
        }
        void cuCopy(){
            unsigned char* tmp = costmap_;
            cudaMemcpy(d_costmap_,costmap_,size_x_*size_y_*sizeof(unsigned char),cudaMemcpyHostToDevice);            
            costmap_=d_costmap_;
            cudaMemcpy(d_p,this,sizeof(CuCostmap2D),cudaMemcpyHostToDevice);
            costmap_=tmp;
        }
        void cuFree(){
            cudaFree(d_costmap_);
            cudaFree(d_p);
            d_p=NULL;
            d_costmap_=NULL;
        }
        ~CuCostmap2D(){
            delete[] costmap_;  
            cuFree();
        }

};
#endif
// tested except default_value_
/* //for test

#include "cuda/cuda_costmap_2d.hpp"

CuCostmap2D h_costmap((void *)&costmap_);
printf("size_x_ %u, size_y_ %u\n",h_costmap.size_x_,h_costmap.size_y_);
printf("%f %f %f\n",h_costmap.resolution_,h_costmap.origin_x_,h_costmap.origin_y_);

for(unsigned int i=0; i<h_costmap.size_x_;i++) {
    for(unsigned int j=0;j<h_costmap.size_y_;j++){
        if(h_costmap.getCost(i,j) != costmap_.getCost(i,j)) printf("not matched %u,%u \n", i,j);
    }
}
*/