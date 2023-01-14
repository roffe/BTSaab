#include <mcp2515.h>
#include "BluetoothA2DPSink.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_a2dp_api.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "btsaab.h"
#include "utf_convert.h"

gpio_num_t CAN_MOSI = GPIO_NUM_23;
gpio_num_t CAN_MISO = GPIO_NUM_19;
gpio_num_t CAN_SCLK = GPIO_NUM_18;
gpio_num_t CAN_CS = GPIO_NUM_5;
gpio_num_t CAN_INT = GPIO_NUM_34;

gpio_num_t MUTE_PIN = GPIO_NUM_32;
gpio_num_t I2S_BCK = GPIO_NUM_26;
gpio_num_t I2S_WS = GPIO_NUM_25;
gpio_num_t I2S_DOUT = GPIO_NUM_22;

uint8_t startVolume = 100; // 0 - 127

spi_device_handle_t spi;

BluetoothA2DPSink a2dp_sink;
MCP2515 mcp2515(&spi);

xQueueHandle interputQueue = xQueueCreate(5, sizeof(int));

char currentTitle[64 + 1];
char currentArtist[64 + 1];

volatile esp_a2d_audio_state_t playbackState;
volatile esp_a2d_connection_state_t connectionState;
volatile bool muted = false;

extern "C" void app_main(void)
{
    setupCAN();
    setupAudio();
    xTaskCreate(CAN_Control_Task, "CAN_Control_Task", 2048, NULL, 1, NULL);
    xTaskCreate(SIDControl, "SID Controller", 2048, NULL, 1, NULL);
}

char displayBuffer[64 + 1];
uint8_t disp_rotate = 0;
volatile bool firstDisplay = true;

/*
wchar_t *aa = L'å';
wchar_t *aA = L'Å';
wchar_t *ae = L'ä';
wchar_t *aE = L'Ä';
wchar_t *oe = L'ö';
wchar_t *oE = L'Ö';

char na = (char)16;
char nA = (char)225;
char ne = (char)17;
char nE = (char)209;
char no = (char)18;
char nO = (char)215;
*/

void CAN_Control_Task(void *params)
{
    int pinNumber = 0;
    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            if (pinNumber == 666)
            {
                if (firstDisplay)
                {
                    requestAccess();
                    firstDisplay = false;
                }

                struct can_frame frame = {
                    .can_id = 0x4A0,
                    .can_dlc = 7,
                    .data = {0x0F, 0x37, 0x00, 0x20, 0x14, 0x95, 0x00},
                };
                mcp2515.sendMessage(&frame);

                uint8_t dispLen = snprintf(displayBuffer, 64, "%s - %s  ", currentArtist, currentTitle);
                if (dispLen > 25)
                {
                    rotateStringLeft(displayBuffer, disp_rotate++);
                    if (disp_rotate > dispLen)
                    {
                        disp_rotate = 0;
                    }
                }

                char onscreen[25 + 1];
                snprintf(onscreen, 25, "%-30s", displayBuffer);

                setRadioText(onscreen);
            }
            else if (pinNumber == 34)
            {
                uint8_t irq = mcp2515.getInterrupts();
                mcp2515.clearInterrupts();
                if (irq & MCP2515::CANINTF_RX0IF)
                {
                    struct can_frame frame;
                    if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK)
                    {
                        frameHandler(&frame);
                    }
                }
                if (irq & MCP2515::CANINTF_RX1IF)
                {
                    struct can_frame frame;
                    if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK)
                    {
                        frameHandler(&frame);
                    }
                }
                if ((irq & MCP2515::CANINTF_ERRIF) || (irq & MCP2515::CANINTF_MERRF))
                {
                    checkErr(mcp2515);
                    mcp2515.clearERRIF();
                    mcp2515.clearMERR();
                }
            }
        }
    }
}

void SIDControl(void *params)
{
    while (true)
    {
        // printf("%s - %s\n", currentArtist, currentTitle);

        if (connectionState == ESP_A2D_CONNECTION_STATE_CONNECTED)
        {
            int interupt = 666;
            xQueueSendToFront(interputQueue, &interupt, 1000);
        }
        delay(600);
    }
}

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

void requestAccess()
{
    struct can_frame frame = {
        .can_id = 0x348,
        .can_dlc = 8,
        .data = {0x11, 0x02, 0x05, 0x19, 0x04, 0x00, 0x00, 0x00},
    };
    mcp2515.sendMessage(&frame);
    delay(70);
}

void setRadioText(char *text)
{
    uint8_t charLen = strlen(text);
    charLen--;
    uint8_t charPos = 0;
    uint8_t row = 2;
    uint8_t pkgs = floor(charLen / 5);
    int seq = 0x40 + pkgs;
    while (seq >= 0)
    {
        struct can_frame frame = {
            .can_id = 0x328,
            .can_dlc = 8,
            .data = {(uint8_t)seq, 0x96, (uint8_t)row},
        };
        for (size_t i = 3; i < 8; i++)
        {
            if (charPos < charLen)
            {
                frame.data[i] = text[charPos++];
            }
            else
            {
                frame.data[i] = 0x00;
            }
        }

        mcp2515.sendMessage(&frame);

        if (seq & 0x40)
        {
            seq -= 0x40;
        }
        seq--;
        if (seq > 0)
        {
            delay(10);
        }
    }
}

void frameHandler(can_frame *f)
{
    if (f->can_id == 0x410)
    {
        if (f->data[5] == 0x01) // night panel enabled
        {
            if (!muted)
            {
                printf("mute\n");
                gpio_set_level(MUTE_PIN, 0);
                muted = true;
            }
        }
        else if (f->data[5] == 0x00)
        {
            if (muted)
            {
                printf("unmute\n");
                gpio_set_level(MUTE_PIN, 1);
                muted = false;
            }
        }
    }
    // framePrint(f);
}

void setupCAN()
{
    gpio_pad_select_gpio(CAN_INT);
    gpio_set_direction(CAN_INT, GPIO_MODE_INPUT);
    gpio_pulldown_en(CAN_INT);
    gpio_pullup_dis(CAN_INT);
    gpio_set_intr_type(CAN_INT, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CAN_INT, gpio_interrupt_handler, (void *)CAN_INT);

    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = CAN_MOSI,
        .miso_io_num = CAN_MISO,
        .sclk_io_num = CAN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    // Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 128, // 50% duty cycle
        .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .clock_speed_hz = 6000000,
        .spics_io_num = CAN_CS,
        .queue_size = 3};
    esp_err_t ret;

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret == ESP_OK);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_47KBPS, MCP_8MHZ);
    mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
    mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
    mcp2515.setFilter(MCP2515::RXF0, false, 0x290); // buttons
    mcp2515.setFilter(MCP2515::RXF1, false, 0x410);
    mcp2515.setNormalMode();
}

void setupAudio()
{

    gpio_set_direction(MUTE_PIN, GPIO_MODE_OUTPUT);
    gpio_pulldown_en(MUTE_PIN);
    gpio_pullup_dis(MUTE_PIN);
    gpio_set_level(MUTE_PIN, 1);

    esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9); // +9dBm

    i2s_pin_config_t i2s_pin_config = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };
    a2dp_sink.set_pin_config(i2s_pin_config);

    static i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 48000, // updated automatically by A2DP
        .bits_per_sample = (i2s_bits_per_sample_t)32,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 16,
        .dma_buf_len = 128,
        .use_apll = true,
        .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
    };
    a2dp_sink.set_i2s_config(i2s_config);

    a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_PLAYING_TIME);
    a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);

    a2dp_sink.set_on_audio_state_changed(audio_state_callback);
    a2dp_sink.set_on_connection_state_changed(connection_state_callback);

    a2dp_sink.set_auto_reconnect(3);
    a2dp_sink.set_volume(startVolume);

    a2dp_sink.start("BTSaab 🎵🎵🎵");
}

void avrc_metadata_callback(uint8_t id, const uint8_t *value)
{
    if (id == 0x01)
    {
        snprintf(currentTitle, 64, "%s", value);
        disp_rotate = 0;
        // utf_convert(currentTitle, currentTitle, 64);
        return;
    }
    else if (id == 0x02)
    {
        snprintf(currentArtist, 64, "%s", value);
        disp_rotate = 0;
        // utf_convert(currentArtist, currentArtist, 64);
        return;
    }
    // printf("AVRC metadata rsp: attribute id 0x%x, %s\n", id, value);
}

void audio_state_callback(esp_a2d_audio_state_t state, void *data)
{
    playbackState = state;
    char stateString[8];
    switch (state)
    {
    case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
        strcpy(stateString, "SUSPEND");
        break;
    case ESP_A2D_AUDIO_STATE_STOPPED:
        strcpy(stateString, "STOPPED");
        break;
    case ESP_A2D_AUDIO_STATE_STARTED:
        strcpy(stateString, "STARTED");
        break;
    default:
        break;
    }
    printf("Audio %s\n", stateString);
}

void connection_state_callback(esp_a2d_connection_state_t state, void *p_param)
{
    connectionState = state;
    // esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(p_param);
    switch (state)
    {
    case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
        printf("A2DP disconnected\n");
        break;
    case ESP_A2D_CONNECTION_STATE_CONNECTING:
        printf("A2DP connecting\n");
        break;
    case ESP_A2D_CONNECTION_STATE_CONNECTED:
        printf("A2DP connected\n");
        a2dp_sink.set_volume(startVolume);
        firstDisplay = true; // force display update
        break;
    case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
        printf("A2DP disconnecting\n");
        break;
    }
}

void checkErr(MCP2515 mcp2515)
{
    uint8_t u8ErrorFlag = mcp2515.getErrorFlags();
    if (u8ErrorFlag & MCP2515::EFLG_RX1OVR)
    {
        printf("CanShield error RX1OVR: receive buffer 1 overflow");
    }

    // if (u8ErrorFlag & MCP2515::EFLG_RX0OVR)
    //{
    //     Serial.println("CanShield error RX0OVR: receive buffer 0 overflow");
    // }

    if (u8ErrorFlag & MCP2515::EFLG_TXBO)
    {
        printf("CanShield error TXBO: bus off\n");
    }

    if (u8ErrorFlag & MCP2515::EFLG_TXEP)
    {
        printf("CanShield error TXEP: transmit error-passive\n");
    }

    if (u8ErrorFlag & MCP2515::EFLG_RXEP)
    {
        printf("CanShield error RXEP: receive error-passive\n");
    }

    if (u8ErrorFlag & MCP2515::EFLG_TXWAR)
    {
        printf("CanShield error TXWAR: transmit error warning\n");
    }

    if (u8ErrorFlag & MCP2515::EFLG_RXWAR)
    {
        printf("CanShield error RXWAR: receive error warning\n");
    }

    if (u8ErrorFlag & MCP2515::EFLG_EWARN)
    {
        printf("CanShield error EWARN: error warning\n");
    }
}
