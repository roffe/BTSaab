#include "btsaab.h"
#define min(a, b) ((a) < (b) ? (a) : (b))

void showWelcome()
{
    can_frame frame = {.can_id = 0x348, .can_dlc = 8, .data = {0x11, 0x02, 0x05, 0x19, 0x04, 0x00, 0x00, 0x00}};
    sendCAN(&frame);
    delay(60);
    char welcome[] = "¬® BTSaab 1.0¬®";
    setRadioText(welcome);
    delay(700);
}

void setRadioText(char *text)
{
    uint8_t charLen = strlen(text);
    uint8_t charPos = 0;

    can_frame frame = {
        .can_id = 0x328,
        .can_dlc = 8,
        .data = {
            (uint8_t)(0x3F + ceil((float)charLen / 5)), // setup sequence
            0x96,
            0x02,
        },
    };

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (frame.data[0] < 0x60)
    {
        uint8_t bytesToCopy = min(5, charLen - charPos);
        memcpy(&frame.data[3], &text[charPos], bytesToCopy);
        charPos += bytesToCopy;

        // Zero-pad the remaining bytes in the frame
        memset(&frame.data[3 + bytesToCopy], 0, 5 - bytesToCopy);

        sendCAN(&frame);
        if (frame.data[0] & 0x40)
        {
            frame.data[0] ^= 0x40;
        }
        if (frame.data[0] > 0x00)
        {
            xTaskDelayUntil(&xLastWakeTime, 8);
        }
        frame.data[0]--;
    }
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
            case 0xA6: // æ
                str[j] = 'a';
                str[j + 1] = 'e';
                j++;
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