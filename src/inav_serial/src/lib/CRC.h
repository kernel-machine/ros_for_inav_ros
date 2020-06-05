#include <cstdint>

class CRC{
    private:
        static uint8_t crc8(uint8_t crc, uint8_t a);
        static uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length);
    public:
        static uint8_t compute(const void *data, uint32_t length);
};