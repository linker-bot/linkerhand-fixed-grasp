#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include <Eigen/Dense>
#include <Eigen/Core>

namespace algorthm
{
    namespace pathplanning
    {
        template <std::size_t SIZE>
        std::array<Eigen::Matrix<double_t, 4, 4>, SIZE> FindLongestPath(const std::array<Eigen::Matrix<double_t, 4, 4>, SIZE> &original_path)
        {
            std::array<Eigen::Matrix<double, 4, 4>, SIZE> result;
            std::array<Eigen::Vector3d, SIZE> coordinates;
            std::array<bool, SIZE> visited = {false};
            std::array<uint8_t, SIZE> index;
            for (int i = 0; i < SIZE; i++)
            {
                coordinates[i] = original_path[i].col(3).head(3);
            }
            // start with index 0 in the original path
            int start_idx = 0;
            int prev_idx = start_idx;
            index.at(0) = startidx;
            visited[startidx] = true;
            for(int i = 1; i < SIZE; i++)
            {
                 // find the furthest point
                double_t max_distance = -1;
                uint8_t max_idx = -1;
                for (int j = 0; j < SIZE; j++)
                {    
                    if (visited[j])
                    {
                        continue;
                    }
                    else
                    {
                        double_t cur_distance = (coordinates[prev_idx] - coordinates[j]).lpnorm<2>();
                        if (cur_distance > max_distance)
                        {
                            max_idx = j;
                            max_distance = cur_distance;
                        }
                    }
                }
                if(max_idx == -1)
                {
                    break;
                }
                else
                {
                    // update next idx
                    prev_idx = max_idx;
                    visited[prev_idx] = true; 
                    index.at(i) = max_idx;
                }
            }
            for(int i =0; i < SIZE; i++)
            {
                result.at(i) = original_path.at(index[i]);
            }
            return result;
        }
    }
}
#endif