#include "socket.h"

Socket::Socket(unsigned short port)
{
    this->fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(int));

    if (bind(this->fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        std::cerr << "ERRORE BIND "<<errno<< std::endl;
    }

    if (listen(this->fd, 10) < 0)
    {
        std::cerr << "ERRORE LISTEN "<<errno<< std::endl;
    }
}

Socket::~Socket()
{
    close(this->fd);
    std::cout << "SOCKET CHIUSO" << std::endl;
}

void Socket::setCallback(void (*callback)(const uint8_t *, size_t))
{
    //std::thread t(&Socket::listener, callback, this->fd, &this->address);
    //listener(callback, this->fd, &this->address);
    socklen_t socklen = sizeof(struct sockaddr);

    while(true) {
        std::cout << "WAIT FOR CLIENT CONNECTION" << std::endl;
        int new_socket = accept(this->fd, (struct sockaddr *)&(this->address), &socklen);

        std::cout << "CLIENT CONNECTED" << std::endl;

        uint8_t buff[BUFF_SIZE]={0};
        const char * exit_string = "exit";
        size_t len;
        do
        {
            len = read(new_socket, buff, BUFF_SIZE);
            callback(buff, len);
            memset(buff, 0, BUFF_SIZE);
        } while (memcmp(buff, exit_string, strlen(exit_string)) && len>0);
        close(new_socket);
        std::cout << "CLIENT DISCONNECTED" << std::endl;
        this->disconnectedCallback();
    }
    
}

bool Socket::isOpen()
{
    return this->fd > 0;
}

void Socket::setClientDisconnectedCallback(void (*callback)()){
    this->disconnectedCallback=callback;
}
