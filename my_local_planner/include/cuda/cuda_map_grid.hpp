#ifndef CUDA_MAP_GRID_HPP_
#define CUDA_MAP_GRID_HPP_
#include <cuda_runtime.h>
#include "cuda/cuda_map_cell.hpp"

class CuMapGrid{
    public:
        double goal_x_, goal_y_; /**< @brief The goal distance was last computed from */
        unsigned int size_x_, size_y_; ///< @brief The dimensions of the grid
        CuMapCell* map_;

        CuMapGrid* d_p;
        CuMapCell* d_map_;

        CuMapGrid(){
            d_p=NULL;
            d_map_=NULL;
        }
        CuMapGrid(double goal_x_, double goal_y_, unsigned int size_x_, unsigned int size_y_){
            this->goal_x_=goal_x_;
            this->goal_y_=goal_y_;
            this->size_x_=size_x_;
            this->size_y_=size_y_;
            map_=new CuMapCell[size_x_*size_y_];
            d_p=NULL;
            d_map_=NULL;
        }
        void replace(double goal_x_, double goal_y_, unsigned int size_x_, unsigned int size_y_){
            if((this->size_x_==size_x_)&&(this->size_y_==size_y_)){
                this->goal_x_=goal_x_;
                this->goal_y_=goal_y_;
            }
            else{
                this->goal_x_=goal_x_;
                this->goal_y_=goal_y_;
                this->size_x_=size_x_;
                this->size_y_=size_y_;
                map_=new CuMapCell[size_x_*size_y_];
            }
        }
        void cuInit(){
            unsigned int size = size_x_*size_y_;
            
            CuMapCell* tmp=map_;
            
            cudaMalloc((void**)&d_map_,size*sizeof(CuMapCell));
            cudaMemcpy(d_map_,map_,size*sizeof(CuMapCell),cudaMemcpyHostToDevice);
            map_=d_map_;

            cudaMalloc((void**)&d_p,sizeof(CuMapGrid));
            cudaMemcpy(d_p,this,sizeof(CuMapGrid),cudaMemcpyHostToDevice);

            map_=tmp;
        }
        void cuCopy(){
            CuMapCell* tmp=map_;
            cudaMemcpy(d_map_,map_,size_x_*size_y_*sizeof(CuMapCell),cudaMemcpyHostToDevice);
            map_=d_map_;
            cudaMemcpy(d_p,this,sizeof(CuMapGrid),cudaMemcpyHostToDevice);
            map_=tmp;
        }
        void cuFree(){
            cudaFree(d_map_);
            cudaFree(d_p);
            d_p=NULL;
            d_map_=NULL;
        }
        ~CuMapGrid(){
            delete[] map_;
            cuFree();
        }
};

#endif
/*
remove private: in map_grid.h
    std::vector<MapCell> map_;
*/

/* //test
cuda_mapgrid h_path_map_(path_map_.goal_x_,path_map_.goal_y_,path_map_.size_x_,path_map_.size_y_);
cuda_mapgrid h_goal_map_(goal_map_.goal_x_,goal_map_.goal_y_,goal_map_.size_x_,goal_map_.size_y_);

for(int i=0;i<path_map_.size_x_*path_map_.size_y_;i++){
memcpy(&h_path_map_.map_[i],&path_map_.map_[i],24);
}        
for(int i=0;i<goal_map_.size_x_*goal_map_.size_y_;i++){
memcpy(&h_goal_map_.map_[i],&goal_map_.map_[i],24);
}
*/