#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <iomanip>
#include <algorithm>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h> 

#include <cmath>

#include "cobs.h"

#pragma pack(1)
struct Accumulators
{
    int32_t m_refI;
    int32_t m_refQ;
    int32_t m_chI;
    int32_t m_chQ;
};
#pragma pack()

void handleRX(int serial_port)
{
    static_assert(sizeof(Accumulators) == 16);

    const Accumulators *accus;
    uint8_t rxBuffer[255];
    uint8_t cobsBuffer[sizeof(rxBuffer)+2];

    float angleRef;
    float angleCh;
    float magRef;
    float magCh;

    float refI;
    float refQ;
    float chI;
    float chQ;

    int samples = 0;
    const int16_t *samplePtr;
    int totalBytes = 0;
    int bytes = 0;
    do
    {
        bytes = read(serial_port, &(rxBuffer[totalBytes]), 1);
        if ((bytes == 1) && (rxBuffer[totalBytes] == 0))
        {
            size_t len = cobsDecode(rxBuffer, totalBytes, cobsBuffer);
            std::cout << " RX " << len << "bytes\n";
            if (len > 0)
            {
                uint8_t cmd = cobsBuffer[0];
                switch(cmd)
                {
                case 0x00:  // sample counter
                    std::cout << "  counter = " << *reinterpret_cast<const uint32_t*>(cobsBuffer+1) << "\n";
                    break;
                case 0x01:
                    samples = (len-1)/2;
                    samplePtr = reinterpret_cast<const int16_t*>(cobsBuffer+1);
                    for(size_t i=0; i<samples; i++)
                    {
                        std::cout << "  " << samplePtr[i] << ",\n";
                    }
                    std::cout << "\n";
                    break;
                case 0x02:
                    accus = reinterpret_cast<const Accumulators*>(cobsBuffer+1);
                    std::cout << "  ref I: " << accus->m_refI << "\n";
                    std::cout << "  ref Q: " << accus->m_refQ << "\n";
                    std::cout << "  ch  I: " << accus->m_chI << "\n";
                    std::cout << "  ch  Q: " << accus->m_chQ << "\n";
                    refI = accus->m_refI / (30000.0f * 32768.0f);
                    refQ = accus->m_refQ / (30000.0f * 32768.0f);
                    chI = accus->m_chI / (30000.0f * 32768.0f);
                    chQ = accus->m_chQ / (30000.0f * 32768.0f);                    
                    magRef = 20.0f*log10(sqrt(refI*refI + refQ*refQ));
                    magCh  = 20.0f*log10(sqrt(chI*chI + chQ*chQ));
                    angleRef = 180.0f * atan2(refQ, refI) / 3.1415927f;
                    angleCh = 180.0f * atan2(chQ, chI) / 3.1415927f;
                    std::cout << "  ch   : " << (magCh - magRef) << " dB\n";
                    std::cout << "         " << (angleCh - angleRef) << " deg.\n";
                    break;   
                case 0x03:  // set frequency
                    std::cout << " set freq ok\n";
                    break;                    
                case 0x04:  // get frequency
                    std::cout << "  Freq = " << 
                        (*reinterpret_cast<uint32_t*>(cobsBuffer+1)/1000) << " kHz \n";
                    break;
                case 0x05:  // input select
                    std::cout << "  input select\n";
                    break;                    
                case 0xFF:  // error response
                    std::cout << "  error!\n";
                    break;
                default:
                    std::cout << "  Unknown response: ";
                    for(size_t i=0; i<len; i++)
                    {
                        printf(" %02X", cobsBuffer[i]);
                    }
                    std::cout << "\n";
                    break;                    
                }
            }
            return;
        }

        if (bytes == 0)
        {
            std::cerr << "USB time-out\n";
        }

        totalBytes += bytes;
        if (totalBytes >= sizeof(rxBuffer))
        {
            std::cerr << "RXBuffer overrun!\n";
            return;
        }
    } while (bytes > 0);
}

void sendBuffer(int serial_port, const uint8_t *buffer, size_t len)
{
    std::cout << " TX " << len << "bytes\n";
    for(size_t i=0; i<len; i++)
    {
        printf(" %02X", buffer[i]);
    }   
    std::cout << "\n"; 
    write(serial_port, buffer, len);
}

int main(int argc, const char *argv[])
{
    int serial_port = open("/dev/ttyACM0", O_RDWR);

    if (serial_port < 0) 
    {
        std::cerr << "Error " << errno << " from open: " << strerror(errno) << "\n";
        return EXIT_FAILURE;
    }

    termios tty;
    if(tcgetattr(serial_port, &tty) != 0) 
    {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << "\n";
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;

    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo 
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetspeed(&tty, B115200);

    tcsetattr(serial_port, TCSAFLUSH, &tty);

    uint8_t cmdBuffer[8];
    uint8_t cobsBuffer[sizeof(cmdBuffer)+2];
    uint32_t freq = 10000000;   

    size_t len;
    bool quit = false;
    
    std::cout << "0) get I2S buffer callback counter\n";
    std::cout << "1) dump raw int16_t measurement buffer\n";
    std::cout << "2) dump accumulators\n";
    std::cout << "f) get frequency\n";
    std::cout << "+) increase frequency by 100 kHz\n";
    std::cout << "-) decrease frequency by 100 kHz\n";
    std::cout << "q) exit\n\n";
    
    while(!quit)
    {
        char c = getchar();
        switch(c)
        {
        case '0':   // get callback count
            cmdBuffer[0] = '\x00';
            len = cobsEncode(cmdBuffer, 1, cobsBuffer);
            cobsBuffer[len++] = 0;
            sendBuffer(serial_port, cobsBuffer, len);
            handleRX(serial_port);
            break;
        case '1':   // get RAW buffer
            cmdBuffer[0] = '\x01';
            len = cobsEncode(cmdBuffer, 1, cobsBuffer);
            cobsBuffer[len++] = 0;
            sendBuffer(serial_port, cobsBuffer, len);
            handleRX(serial_port);
            break;
        case '2':   // get accumulators
            cmdBuffer[0] = '\x02';
            len = cobsEncode(cmdBuffer, 1, cobsBuffer);
            cobsBuffer[len++] = 0;
            sendBuffer(serial_port, cobsBuffer, len);
            handleRX(serial_port);
            break;
        case 'f':   // get frequency
            cmdBuffer[0] = '\x04';
            len = cobsEncode(cmdBuffer, 1, cobsBuffer);
            cobsBuffer[len++] = 0;
            sendBuffer(serial_port, cobsBuffer, len);
            handleRX(serial_port);
            break;            
        case '-':
            freq = std::clamp(freq - 100000U, 100000U, 50000000U);
            cmdBuffer[0] = '\x03';
            memcpy(cmdBuffer+1, &freq, sizeof(freq));
            len = cobsEncode(cmdBuffer, 1 + sizeof(freq), cobsBuffer);
            cobsBuffer[len++] = 0;
            sendBuffer(serial_port, cobsBuffer, len);
            handleRX(serial_port);
            std::cout << "Freq: " << static_cast<int>(freq / 1000) << " kHz\n";
            break;
        case '+':
            freq = std::clamp(freq + 100000U, 100000U, 50000000U);
            cmdBuffer[0] = '\x03';
            memcpy(cmdBuffer+1, &freq, sizeof(freq));
            len = cobsEncode(cmdBuffer, 1 + sizeof(freq), cobsBuffer);
            cobsBuffer[len++] = 0;
            sendBuffer(serial_port, cobsBuffer, len);
            handleRX(serial_port);
            std::cout << "Freq: " << static_cast<int>(freq / 1000) << " kHz\n";
            break;
        case 'r':
            cmdBuffer[0] = '\x05';
            cmdBuffer[1] = 0;
            len = cobsEncode(cmdBuffer, 2, cobsBuffer);
            cobsBuffer[len++] = 0;
            sendBuffer(serial_port, cobsBuffer, len);
            handleRX(serial_port);
            break;
        case 't':
            cmdBuffer[0] = '\x05';
            cmdBuffer[1] = 1;
            len = cobsEncode(cmdBuffer, 2, cobsBuffer);
            cobsBuffer[len++] = 0;
            sendBuffer(serial_port, cobsBuffer, len);
            handleRX(serial_port);
            break;            
        case 'q':
        case 'Q':
            quit = true;
            break;
        default:
            std::cout << "0,1,2 or q\n";
            break;
        }
    }

    close(serial_port);
    std::cout << "done\n";
    return EXIT_SUCCESS;
}
