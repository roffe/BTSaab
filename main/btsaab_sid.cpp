#include "btsaab.h"
#define min(a, b) ((a) < (b) ? (a) : (b))

/* Format of NODE_DISPLAY_RESOURCE_REQ frame:
 ID: Node ID requesting to write on SID
 [0]: Request source
 [1]: SID object to write on; 0 = entire SID; 1 = 1st row; 2 = 2nd row
 [2]: Request type: 1 = Engineering test; 2 = Emergency; 3 = Driver action; 4 = ECU action; 5 = Static text; 0xFF = We don't want to write on SID
 [3]: Request source function ID 19=IHU 12=SPA 23=ACC 14=SID
 [4-7]: Zeroed out; not in use
 */

void sendBeep(uint8_t sound)
{
    can_frame frame = {.can_id = 0x430, .can_dlc = 8, .data = {0x80, sound, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
    sendCAN(&frame);
}

// char welcome[] = "¬® BTSaab 1.0¬®";
void showWelcome()
{
    can_frame frame = {.can_id = 0x348, .can_dlc = 8, .data = {0x11, 0x02, 0x05, 0x19, 0x04, 0x00, 0x00, 0x00}};
    sendCAN(&frame);
    delay(60);
    sendBeep(146);
    delay(300);
    sendBeep(136);
    delay(5);
    setRadioText(getCompliment());
    delay(700);
}

void setRadioText(const char *text)
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
        if (str[i] == 0xE2 && str[i + 1] == 0x80 && str[i + 2] == 0x99)
        { // check for the first byte of a 3-byte UTF-8 character
            str[j] = 0x27;
            i += 3;
        }
        else if (str[i] == 0xC2)
        {
            switch (str[i + 1])
            {
            case 0xB0: // °
                str[j] = 0x1F;
                i += 2;
                break;
            default:
                str[j] = str[i];
                i++;
            }
        }
        else if (str[i] == 0xC3)
        { // check for the first byte of a 2-byte UTF-8 character
            switch (str[i + 1])
            {
            case 0xA1: // á
                str[j] = 0x0F;
                i += 2;
                break;
            case 0xA4: // ä
                str[j] = 0x11;
                i += 2;
                break;
            case 0xA5: // å
                str[j] = 0x10;
                i += 2;
                break;
            case 0xA6: // æ
                str[j++] = 'a';
                str[j] = 'e';
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
            case 0x84: // Ä
                str[j] = 0xD1;
                i += 2;
                break;
            case 0x85: // Å
                str[j] = 0xE1;
                i += 2;
                break;
            case 0x86: // Æ
                str[j++] = 'A';
                str[j] = 'e';
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