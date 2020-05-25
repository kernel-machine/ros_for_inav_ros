#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>


class Serial { 
    private:
        int fd;
    public:
        Serial(const char* name);
        ~Serial();
        int serialSetInterfaceAttribs(int speed, int parity);
        void serialSetBlocking(bool should_block);
        void serialRead(uint8_t * buf, size_t size);
        bool isOpen();
};
