#include "CRC.h"

uint8_t CRC::crc8(uint8_t crc, uint8_t a)
{
    uint8_t crc_u = a;
    crc_u ^= crc;

    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }

    return crc_u;
}

uint8_t CRC::crc8_update(uint8_t crc, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    const uint8_t *pend = p + length;

    for (; p != pend; p++) {
        crc = crc8(crc, *p);
    }
    return crc;
}

uint8_t CRC::compute(const void *data, uint32_t length){
    return CRC::crc8_update(0,data,length);
}