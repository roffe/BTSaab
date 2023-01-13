#include <mcp2515.h>
#include <BluetoothA2DPSink.h>

void setupCAN();
void setupAudio();
void SIDControl(void *params);
void avrc_metadata_callback(uint8_t data1, const uint8_t *data2);
void audio_state_callback(esp_a2d_audio_state_t state, void *data);
void connection_state_callback(esp_a2d_connection_state_t state, void *p_param);
void checkErr(MCP2515 mcp2515);

void framePrint(can_frame *f)
{
    printf("%d %02X:", millis(), f->can_id);
    for (int i = 0; i < f->can_dlc; i++)
    {
        printf("%02X", f->data[i]);
    }
    printf("\n");
}