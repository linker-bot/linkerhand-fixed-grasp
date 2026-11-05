#include "handeyesolver.h"
#include <sys/types.h>
#include "GeneralFunction.h"

char* const       handeyerawdata    = "handeyerawdata";
char* const       handeyeresult     = "handeyeresult";
std::string       pythonFilePath    = ros::package::getPath("aimo_control") + "/handeyesolver/python/main.py";
const char* const PythonPath        = pythonFilePath.c_str();
char* const       PythonIntrepreter = "/usr/bin/python3.8";
const char* const PythonArgs[]      = { PythonIntrepreter, PythonPath, NULL };

namespace algorithm
{
    namespace calibration
    {
        HandEyeSolver::HandEyeSolver() : pipecomm_(handeyerawdata, handeyeresult), pid_(-1), write_buffer_(), read_buffer_() {}
        std::vector<std::pair<DoublePoseMatrix, DoublePoseMatrix>> HandEyeSolver::CalculateX(const std::vector<DoublePoseMatrix>& hand_data, const std::vector<DoublePoseMatrix>& eye_data,
                                                                                             const std::vector<HandEyeMethod>& methods)
        {
            pid_                                                                   = fork();
            size_t                                                     sz          = hand_data.size();
            size_t                                                     method_size = methods.size();
            std::vector<std::pair<DoublePoseMatrix, DoublePoseMatrix>> result;
            if(sz * 32 + 1 <= 10240)
            {
                if(pid_ > 0)
                {
                    // parent process
                    // copy the method data into buffer
                    int count = 0;
                    memcpy(&write_buffer_[0], &method_size, sizeof(uint8_t));
                    count++;
                    for(int i = 0; i < methods.size(); i++)
                    {
                        memcpy(&write_buffer_[1 + i * sizeof(uint8_t)], &methods[i], sizeof(uint8_t));
                        count++;
                    }
                    // copy hand/eye data into buffer
                    memcpy(&write_buffer_[1 + method_size * sizeof(uint8_t)], &sz, sizeof(uint8_t));
                    count++;
                    for(int i = 0; i < hand_data.size(); i++)
                    {
                        memcpy(&write_buffer_[2 + method_size * sizeof(uint8_t) + i * 16 * 8], hand_data[i].data(), 128);
                        count += 16 * 8;
                    }

                    for(int i = 0; i < eye_data.size(); i++)
                    {
                        memcpy(&write_buffer_[2 + method_size * sizeof(uint8_t) + i * 16 * 8 + sz * 16 * 8], eye_data[i].data(), 128);
                        count += 16 * 8;
                    }

                    std::cout << "Parent start" << std::endl;

                    pipecomm_.Start(handeyerawdata, handeyeresult);
                    pipecomm_.Write(reinterpret_cast<void*>(write_buffer_), count);
                    pipecomm_.Read(reinterpret_cast<void*>(read_buffer_), 16 * sizeof(double_t) * methods.size() * 2);

                    for(int i = 0; i < methods.size(); i++)
                    {
                        Eigen::Map<DoublePoseMatrix, Eigen::Unaligned> base_to_eye(reinterpret_cast<double_t*>(read_buffer_) + i * 32, 4, 4);
                        Eigen::Map<DoublePoseMatrix, Eigen::Unaligned> target_to_hand(reinterpret_cast<double_t*>(read_buffer_) + i * 32 + 16, 4, 4);
                        result.emplace_back(std::make_pair(base_to_eye.transpose(), target_to_hand.transpose()));
                    }
                    wait(NULL);
                    return result;
                }
                else if(pid_ == 0)
                {
                    // child process
                    std::cout << "Child start" << std::endl;
                    int result = execvp((char* const)PythonIntrepreter, (char* const*)PythonArgs);
                }
                else
                {
                    // cannot fork process
                }
            }
            else
            {
                std::cout << "Size exceed buffer size " << std::endl;
            }
            return result;
        }
        HandEyeSolver::~HandEyeSolver()
        {
            if(pid_ >= 0)
            {
                kill(pid_, SIGKILL);
                exit(0);
            }
        }
    }  // namespace calibration
}  // namespace algorithm
