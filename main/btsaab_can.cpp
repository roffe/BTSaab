#include "btsaab.h"

void framePrint(can_frame *frame)
{
    printf("%d %02X:", millis(), frame->can_id);
    for (int i = 0; i < frame->can_dlc; i++)
    {
        printf("%02X", frame->data[i]);
    }
    printf("\n");
}
