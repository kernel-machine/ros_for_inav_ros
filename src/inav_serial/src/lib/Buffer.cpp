#include "Buffer.h"

Buffer::Buffer(const size_t size){
    this->buff = new uint8_t[size];
    this->buffSize=size;
}

Buffer::~Buffer(){
    delete this->buff;
}

size_t Buffer::getBytesCount(){
    return this->buffPosition;
}

size_t Buffer::getAvaiableSpace(){
    return this->buffSize-this->buffPosition;
}

int Buffer::pushBytes(uint8_t * bytes, size_t size){
    if( this->getAvaiableSpace() < size){
        return -1;
    }
    int i;
    for(i = 0;i<size;i++){
        this->buff[this->buffPosition]=*(bytes+i);
        this->buffPosition++;
        if(this->buffPosition==this->buffSize)
            break;
    }
    return i;
}

int Buffer::popBytes(uint8_t * bytes, size_t size){
    if(size>this->getBytesCount()){
        return -1;
    }
    int i;
    for(i=0;i<size;i++){
        *(bytes+i)=this->buff[this->buffPosition-size+i];
    }
    this->buffPosition-=i;
    return i;
}
