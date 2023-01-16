#include <mcp2515.h>
#include <BluetoothA2DPSink.h>

// void CAN_SEND_Control_Task(void *params);
void audio_state_callback(esp_a2d_audio_state_t state, void *data);
void avrc_metadata_callback(uint8_t data1, const uint8_t *data2);
void checkErr(MCP2515 mcp2515);
void connection_state_callback(esp_a2d_connection_state_t state, void *p_param);
void frameHandler(void *params);
void framePrint(can_frame *f);
void interruptHandler(void *params);
void rotateStringLeft(char *str, int n);
void sendCAN(can_frame *frame);
void setRadioText(const char *text);
void setupAudio();
void setupInterrupts();
void setupMCP2515();
void showWelcome();
void SIDControl(void *params);
void utf8_to_sid(char *str);
const char *getCompliment();
void SIDAccessControl(void *params);