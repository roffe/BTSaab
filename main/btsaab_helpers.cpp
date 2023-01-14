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

void utf8_to_sid(char *str)
{
    int i = 0, j = 0;
    while (str[i])
    {
        if (str[i] == 0xC3)
        { // check for the first byte of a 2-byte UTF-8 character
            switch (str[i + 1])
            {
            case 0xA5: // å
                str[j] = 0x10;
                i += 2;
                break;
            case 0xA8: // è
                str[j] = 0xE8;
                i += 2;
                break;
            case 0xA9: // é
                str[j] = 0xE9;
                i += 2;
                break;
            case 0x85: // Å
                str[j] = 0xE1;
                i += 2;
                break;
            case 0x86: // Æ
                str[j] = 0xE1;
                i += 2;
                break;
            case 0xA1: // á
                str[j] = 0x0F;
                i += 2;
                break;
            case 0xA4: // ä
                str[j] = 0x11;
                i += 2;
                break;
            case 0x84: // Ä
                str[j] = 0xD1;
                i += 2;
                break;
            case 0xB6: // ö
                str[j] = 0x12;
                i += 2;
                break;
            case 0x96: // Ö
                str[j] = 0xD7;
                i += 2;
                break;
            case 0xBC: // ü
                str[j] = 0x13;
                i += 2;
                break;
            case 0x9C: // Ü
                str[j] = 0xD9;
                i += 2;
                break;
            case 0xB8: // ø
                str[j] = 0xF8;
                i += 2;
                break;
            case 0x98: // Ø
                str[j] = 0xD8;
                i += 2;
                break;
            default:
                str[j] = str[i];
                i++;
            }
        }
        else
        {
            str[j] = str[i];
            i++;
        }
        j++;
    }
    str[j] = '\0'; // add null terminator
}