#ifndef CUDA_MAP_CELL_HPP_
#define CUDA_MAP_CELL_HPP_

class CuMapCell{
    public:
        unsigned int cx, cy; ///< @brief Cell index in the grid map
        double target_dist; ///< @brief Distance to planner's path
        bool target_mark; ///< @brief Marks for computing path/goal distances
        bool within_robot; ///< @brief Mark for cells within the robot footprint
};

#endif