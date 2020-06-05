#include "serial.h"

Serial::Serial(const char* name) {
  int fd = open (name, O_RDWR | O_NOCTTY | O_SYNC );
  if (fd < 0) {
    fprintf (stderr,"error %d opening serial, fd %d\n", errno, fd);
  }
  this->fd = fd;
  this->b = new Buffer(RX_BUFF_SIZE);
  this->atomic_buffer = new std::atomic<Buffer*>(this->b);
}

Serial::~Serial(){
  close(this->fd);
  delete serialReadThread;
  delete b;
}

int Serial::setInterfaceAttribs(int speed, int parity) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    return -1;
  }
  switch (speed){
  case 9600:
    speed=B9600;
  break;
  case 19200:
    speed=B19200;
    break;
  case 57600:
    speed=B57600;
    break;
  case 115200:
    speed=B115200;
    break;
  default:
    printf("cannot sed baudrate %d\n", speed);
    return -1;
  }
  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);
  cfmakeraw(&tty);
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);               // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;      // 8-bit chars

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    fprintf (stderr,"error %d from tcsetattr\n", errno);
    return -1;
  }
  return 0;
}

void Serial::setBlocking(bool should_block) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
      fprintf (stderr,"error %d from tggetattr\n", errno);
      return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    fprintf (stderr,"error %d setting term attributes\n", errno);
}

void Serial::writeData(uint8_t * buff, size_t size){
  ssize_t writtenBytes = 0;
  ssize_t tmp = 0;
  while(writtenBytes < size){
    tmp = write(this->fd, buff+writtenBytes, size-writtenBytes);
    if(tmp < 0){
      perror("Write error");
      exit(1);
      break;
    }
    else  writtenBytes += tmp;
  }
}

bool Serial::isOpen(){
  return this->fd>0;
}


void Serial::spin(){
  this->serialReadThread 
    = new std::thread(&(Serial::serialReadPolling), this->fd, this->atomic_buffer);
}


ssize_t Serial::dataAvaiable(){
  return atomic_buffer->load()->getBytesCount();
}

void Serial::readData(uint8_t * buf, size_t size){
  atomic_buffer->load()->popBytes(buf,size);
}

void Serial::serialReadPolling(int fd,std::atomic<Buffer*> * atomic_buffer){
  /*
    This must be executed on a second thread that poll on the serial
  */
  uint8_t data;
  int readRis = 0;
  while (true)
  {
    readRis = read(fd,&data,1);
    if(readRis>0){
      //ATOMIC SECTION
      //append localBuff to mainBuff
      atomic_buffer->load()->pushBytes(&data,1);
    }
    else if(readRis==-1 || readRis==0)break;
  }
  
}

