#include <sys/types.h>
#include <sys/socket.h>
//"in" per "sockaddr_in"
#include <netinet/in.h>
//"fcntl" per la funzione "fcntl"
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <iostream>
#include <string.h>

#define BUFF_SIZE 1024

class Socket {
    private:
        int fd;
        struct sockaddr_in address; 
        void (*disconnectedCallback)();

        //void (*callback)(uint8_t *, size_t);
    public: 
        Socket(unsigned short port);
        ~Socket();
        void setCallback(void (*callback)(const uint8_t*,size_t));
        void setClientDisconnectedCallback(void (*callback)());
        bool isOpen();
};