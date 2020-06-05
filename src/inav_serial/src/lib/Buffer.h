#include <stdint.h>
#include <cstddef>

class Buffer {
    private:
        uint8_t buffPosition = 0;
        uint8_t * buff;
        size_t buffSize;
    public:
        Buffer(const size_t size);
        ~Buffer();
        size_t getBytesCount();
        size_t getAvaiableSpace();
        int pushBytes(uint8_t * bytes, size_t size);
        int popBytes(uint8_t * bytes, size_t size);
        
};