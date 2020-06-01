#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <iostream>

#define RX_BUFFER_SIZE 512
#define CIRCULAR_INDEX(i) (i%RX_BUFFER_SIZE)

class Serial {

    private:
        int fd;
        uint8_t rxBuff[RX_BUFFER_SIZE];
        uint8_t rxBufferHead = 0;
        uint8_t rxBufferTail = 0;
    public:
        Serial(const char* name);
        ~Serial();
        int setInterfaceAttribs(int speed, int parity);
        void setBlocking(bool should_block);

        void readData(uint8_t * buf, size_t size);
        void spin();
        ssize_t dataAvaiable();


        void writeData(uint8_t * buff, size_t size);
        bool isOpen();
};
