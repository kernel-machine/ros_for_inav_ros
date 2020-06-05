#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <thread>
#include <atomic>
#include "Buffer.h"
#include <iostream>

#define RX_BUFF_SIZE 512
#define PACKET_HEADER 'i'

class Serial {

    private:
        int fd;
        std::thread * serialReadThread;
        Buffer * b;
        std::atomic<Buffer*> * atomic_buffer;
        static void serialReadPolling(int fd, std::atomic<Buffer*> * atomic_buffer);
    public:
        Serial(const char* name);
        ~Serial();
        int setInterfaceAttribs(int speed, int parity);
        void setBlocking(bool should_block);
        bool isOpen();

        void readData(uint8_t * buf, size_t size);
        void spin();
        ssize_t dataAvaiable();

        void writeData(uint8_t * buff, size_t size);
};
