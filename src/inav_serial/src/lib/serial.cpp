#include "serial.h"

Serial::Serial(const char* name) {
  int fd = open (name, O_RDWR | O_NOCTTY | O_SYNC );
  if (fd < 0) {
    fprintf (stderr,"error %d opening serial, fd %d\n", errno, fd);
  }
  this->fd = fd;
}

Serial::~Serial(){
  close(this->fd);
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

    tmp = write(this->fd,buff,size);
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
  uint8_t b;
  int i;
  for(i = 0; i<RX_BUFFER_SIZE; i++){

    int status = read(this->fd, &b, 1);
    if( status == 0 )
      break; //NO MORE DATA TO BE READED
    else if( status < 0)
      break; //ERROR
    else { //READED 1 BYTE
      rxBuff[rxBufferTail]=b;
      rxBufferTail++;
      if(rxBufferTail==RX_BUFFER_SIZE)
        rxBufferTail=0;
    }
  }
  std::cout<<i<<" bytes received"<<std::endl;
}

ssize_t Serial::dataAvaiable(){
  std::cout<<"HEAD "<<(int)this->rxBufferHead<<" TAIL "<<(int)this->rxBufferTail<<std::endl;
  if(this->rxBufferTail >= this->rxBufferHead)
    return this->rxBufferTail - this->rxBufferHead;
  else {
    return RX_BUFFER_SIZE - this->rxBufferHead + this->rxBufferTail;
  }
}

void Serial::readData(uint8_t * buf, size_t size){
  /*
  if(size > this->dataAvaiable())
    return;

  for(int i = 0;i<size;i++){
    *(buf+i)=rxBuff[(rxBufferHead+i)%RX_BUFFER_SIZE];
  }

  this->rxBufferHead+=size;
  this->rxBufferHead%=RX_BUFFER_SIZE;
  */
 
 for(int i = 0;i<size;i++)
  read(this->fd,buf+i,1);
 
}
