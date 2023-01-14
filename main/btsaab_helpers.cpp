#include "freertos/FreeRTOS.h"
#include "btsaab.h"
#include "string.h"

void rotateStringLeft(char *str, int n)
{
    int len = strlen(str);
    n = n % len; // handle case where n > len
    char temp[n + 1];
    strncpy(temp, str, n); // copy first n characters to temp
    temp[n] = '\0';        // add null termination to temp
    for (int i = 0; i < len - n; i++)
    {
        str[i] = str[i + n]; // shift remaining characters to the left
    }
    // memcpy(&str[(len - n) - 1], temp, n); // copy temp to the end of str
    memcpy(&str[(len - n)], temp, n + 1); // copy temp to the end of str
}

void framePrint(can_frame *f)
{
    printf("%d %02X:", millis(), f->can_id);
    for (int i = 0; i < f->can_dlc; i++)
    {
        printf("%02X", f->data[i]);
    }
    printf("\n");
}

int replacechar(char *str, char orig, char rep)
{
    char *ix = str;
    int n = 0;
    while ((ix = strchr(ix, orig)) != NULL)
    {
        *ix++ = rep;
        n++;
    }
    return n;
}

void replace_utf8_multibyte(char *str, char *from, char *to)
{
    int i, j, k;
    int str_len = strlen(str);
    int from_len = strlen(from);
    int to_len = strlen(to);
    bool found;
    for (i = 0; i < str_len; i++)
    {
        if (str[i] == from[0])
        {
            found = true;
            for (j = 1; j < from_len; j++)
            {
                if (str[i + j] != from[j])
                {
                    found = false;
                    break;
                }
            }
            if (found)
            {
                for (j = 0; j < from_len; j++)
                {
                    for (k = i + from_len; k < str_len; k++)
                    {
                        str[k - from_len + to_len] = str[k];
                    }
                }
                for (j = 0; j < to_len; j++)
                {
                    str[i + j] = to[j];
                }
                str_len = str_len - from_len + to_len;
                i = i + to_len - 1;
            }
        }
    }
}