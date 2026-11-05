#include "pipecomm.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> /* Definition of AT_* constants */

const int PipeSizeLimit = 10240;
namespace ipc
{
    namespace comm
    {
        PipeComm::PipeComm(char *reader_pipe_path, char *writer_pipe_path)
            : reader_pipe_fd_(-1), writer_pipe_fd_(-1)
        {
            if (access(reader_pipe_path, F_OK) == -1)
            {
                int ret = mkfifo(reader_pipe_path, 0664);
                if (ret < 0)
                {
                    perror("reader pipe creation");
                    exit(ret);
                }
            }
            if (access(writer_pipe_path, F_OK) == -1)
            {
                int ret = mkfifo(writer_pipe_path, 0664);
                if (ret < 0)
                {
                    perror("write pipe creation");
                    exit(ret);
                }
            }
        }

        int PipeComm::Start(char *writer_pipe_path, char *reader_pipe_path)
        {
            // on c++ end, first open writer then open reader, 
            // on python side, first open reader then open writer
            // as c++ reader = python writer, c++ writer = python reader
            writer_pipe_fd_ = open(writer_pipe_path, O_WRONLY, 0);
            reader_pipe_fd_ = open(reader_pipe_path, O_RDONLY);
            
            return F_OK;
        }

        int PipeComm::Write(void *writebuffer, const uint16_t sz)
        {
            if (sz < PipeSizeLimit)
            {
                int ret = write(writer_pipe_fd_, writebuffer, sz);
                std::cout << "write " << ret << std::endl;
                if (ret == -1)
                {
                    perror("write");
                    exit(-1);
                }
                return ret;
            }
            else
            {
                std::cout << "ignore write request, exceeds pipe size" << std::endl;
                return -1;
            }
        }

        int PipeComm::Read(void *read_buffer, const uint16_t sz)
        {
            if (sz < PipeSizeLimit && sz > 0)
            {
                int ret = read(reader_pipe_fd_, read_buffer, sz);
                if (ret == -1)
                {
                    perror("read");
                    exit(0);
                }
                return F_OK;
            }
            else
            {
                std::cout << "ignore read request, illegal size request" << std::endl;
                return -1;
            }
        }

        PipeComm::~PipeComm()
        {
            if (reader_pipe_fd_ != -1)
            {
                close(reader_pipe_fd_);
            }
            if (writer_pipe_fd_ != -1)
            {
                close(writer_pipe_fd_);
            }
        }

    }
}
