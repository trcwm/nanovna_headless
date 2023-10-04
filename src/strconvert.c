
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
    int neg = FALSE;
    if (*p == '-') neg = TRUE;
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
