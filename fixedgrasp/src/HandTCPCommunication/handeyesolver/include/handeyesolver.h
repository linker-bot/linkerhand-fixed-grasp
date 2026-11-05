#ifndef HANDEYE_SOLVER_H
#define HANDEYE_SOLVER_H
#include <thread>
#include <sys/wait.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "pipecomm.h"
#include <vector>
#include "GeneralFunction.h"

typedef Eigen::Matrix<double, 4, 4> DoublePoseMatrix;

namespace algorithm
{
    namespace calibration
    {
        enum HandEyeMethod
        {
            DualQuaternion      = 1,
            KroneckerProduct    = 2,
            Iterative_eyeinhand = 3,  // MinCovirance     = 3,
            Iterative           = 4,
        };
        class HandEyeSolver
        {
        public:
            HandEyeSolver();
            std::vector<std::pair<DoublePoseMatrix, DoublePoseMatrix>> CalculateX(const std::vector<DoublePoseMatrix>& hand_data, const std::vector<DoublePoseMatrix>& eye_data,
                                                                                  const std::vector<HandEyeMethod>& methods);
            ~HandEyeSolver();

        private:
            ipc::comm::PipeComm pipecomm_;
            pid_t               pid_;
            uint8_t             write_buffer_[10240];
            uint8_t             read_buffer_[2096];
        };
    }  // namespace calibration
}  // namespace algorithm

#endif