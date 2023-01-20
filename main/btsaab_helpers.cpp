#include "freertos/FreeRTOS.h"
#include "btsaab.h"
#include "string.h"

void checkErr(MCP2515 *mcp2515)
{
    uint8_t u8ErrorFlag = mcp2515->getErrorFlags();
    if (u8ErrorFlag & MCP2515::EFLG_RX1OVR)
    {
        ESP_LOGE("CanShield", "CanShield error RX1OVR: receive buffer 1 overflow");
    }

    // if (u8ErrorFlag & MCP2515::EFLG_RX0OVR)
    //{
    //     Serial.println("CanShield error RX0OVR: receive buffer 0 overflow");
    // }

    if (u8ErrorFlag & MCP2515::EFLG_TXBO)
    {
        ESP_LOGE("CanShield", "CanShield error TXBO: bus off");
    }

    if (u8ErrorFlag & MCP2515::EFLG_TXEP)
    {
        ESP_LOGE("CanShield", "CanShield error TXEP: transmit error-passive");
    }

    if (u8ErrorFlag & MCP2515::EFLG_RXEP)
    {
        ESP_LOGE("CanShield", "CanShield error RXEP: receive error-passive");
    }

    if (u8ErrorFlag & MCP2515::EFLG_TXWAR)
    {
        ESP_LOGE("CanShield", "CanShield error TXWAR: transmit error warning");
    }

    if (u8ErrorFlag & MCP2515::EFLG_RXWAR)
    {
        ESP_LOGE("CanShield", "CanShield error RXWAR: receive error warning");
    }

    if (u8ErrorFlag & MCP2515::EFLG_EWARN)
    {
        ESP_LOGE("CanShield", "CanShield error EWARN: error warning");
    }
}

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
    memcpy(&str[(len - n)], temp, n + 1); // copy temp to the end of str
}
