#include <stdbool.h>
#include "strconvert.h"

// Use macro, std isdigit more big
#define _isdigit(c) (c >= '0' && c <= '9')

// Rewrite universal standart str to value functions to more compact
//
// Convert string to int32
int32_t my_atoi(const char *p)
{
    int32_t value = 0;
    uint32_t c;
    bool neg = false;

    if (*p == '-')
    {
        neg = true;
        p++;
    }
    if (*p == '+') p++;
    while ((c = *p++ - '0') < 10) value = value * 10 + c;
    return neg ? -value : value;
}

// Convert string to uint32
//  0x - for hex radix
//  0o - for oct radix
//  0b - for bin radix
//  default dec radix
uint32_t my_atoui(const char *p)
{
    uint32_t value = 0, radix = 10, c;
    if (*p == '+') p++;
    if (*p == '0')
    {
        switch (p[1])
        {
        case 'x':
            radix = 16;
            break;
        case 'o':
            radix = 8;
            break;
        case 'b':
            radix = 2;
            break;
        default:
            goto calculate;
        }
        p += 2;
    }
calculate:
    while (1)
    {
        c = *p++ - '0';
        // c = to_upper(*p) - 'A' + 10
        if (c >= 'A' - '0') c = (c & (~0x20)) - ('A' - '0') + 10;
        if (c >= radix) return value;
        value = value * radix + c;
    }
}

double my_atof(const char *p)
{
    int neg = false;
    if (*p == '-') neg = true;
    if (*p == '-' || *p == '+') p++;
    double x = my_atoi(p);
    while (_isdigit((int)*p)) p++;
    if (*p == '.')
    {
        double d = 1.0f;
        p++;
        while (_isdigit((int)*p))
        {
            d /= 10;
            x += d * (*p - '0');
            p++;
        }
    }
    if (*p == 'e' || *p == 'E')
    {
        p++;
        int exp = my_atoi(p);
        while (exp > 0)
        {
            x *= 10;
            exp--;
        }
        while (exp < 0)
        {
            x /= 10;
            exp++;
        }
    }
    if (neg) x = -x;
    return x;
}

//
// Function used for search substring v in list
// Example need search parameter "center" in "start|stop|center|span|cw" getStringIndex
// return 2 If not found return -1 Used for easy parse command arguments
int get_str_index(char *v, const char *list)
{
    int i = 0;
    while (1)
    {
        char *p = v;
        while (1)
        {
            char c = *list;
            if (c == '|') c = 0;
            if (c == *p++)
            {
                // Found, return index
                if (c == 0) return i;
                list++;  // Compare next symbol
                continue;
            }
            break;  // Not equal, break
        }
        // Set new substring ptr
        while (1)
        {
            // End of string, not found
            if (*list == 0) return -1;
            if (*list++ == '|') break;
        }
        i++;
    }
    return -1;
}