#ifndef PIPECOMM_H
#define PIPECOMM_H
#include <iostream>
#include <sys/types.h>
#include <unistd.h>

namespace ipc
{
    namespace comm
    {

        class PipeComm
        {
        public:
            explicit PipeComm(char *reader_pipe_path, char *writer_pipe_path);
            int Start(char *reader_pipe_path, char *writer_pipe_path);
            int Read(void *read_buffer, const uint16_t sz);
            int Write(void* writebuffer, const uint16_t sz);
            ~PipeComm();

        private:
            PipeComm();
            int reader_pipe_fd_;
            int writer_pipe_fd_;
        };
    }
}

#endif
