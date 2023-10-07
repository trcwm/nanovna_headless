#include <iostream>
#include <cstring>
#include <cstdio>
#include "cobs.h"

int main()
{
    const uint8_t data[] = 
    {
        0x05,0x03,0x20,0x1D,0x9A,0x01,0x00
    };

    uint8_t decoded[255];

    auto len = strlen((const char*)data);
    len = cobsDecode(data, len, decoded);

    for(size_t i=0; i<len; i++)
    {
        printf(" %02X", decoded[i]);
    }

    std::cout << "\n";
    return EXIT_SUCCESS;
}
