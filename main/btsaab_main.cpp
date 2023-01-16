#include <mcp2515.h>
#include "esp_system.h"
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
#include <pthread.h>
#include "utf_convert.h"

// the ticks between each SID update
#define SID_UPDATE_RATE 60
#define ROTATE_STEPS 2

// For old SID 12, 9-5 2003-2005 24
#define SCREEN_WIDTH 24

// SPI speed to talk to MCP2515
#define SPI_MHZ 6

// Buffer sizes
#define MAX_ARTIST_LENGTH 64
#define MAX_TITLE_LENGTH 64
#define SCREEN_BUFFER_LENGTH MAX_ARTIST_LENGTH + MAX_TITLE_LENGTH

gpio_num_t CAN_MOSI = GPIO_NUM_23;
gpio_num_t CAN_MISO = GPIO_NUM_19;
gpio_num_t CAN_SCLK = GPIO_NUM_18;
gpio_num_t CAN_CS = GPIO_NUM_5;
gpio_num_t CAN_INT = GPIO_NUM_34;

gpio_num_t ENABLE_OUTPUT_PIN = GPIO_NUM_32;
gpio_num_t I2S_BCK = GPIO_NUM_26;
gpio_num_t I2S_WS = GPIO_NUM_25;
gpio_num_t I2S_DOUT = GPIO_NUM_22;

uint8_t startVolume = 100; // 0 - 127

BluetoothA2DPSink a2dp_sink;

pthread_mutex_t canMutex;
spi_device_handle_t spi;
MCP2515 mcp2515(&spi);

xQueueHandle interruptQueue = xQueueCreate(5, sizeof(int));

char currentTitle[MAX_ARTIST_LENGTH + 1];
char currentArtist[MAX_TITLE_LENGTH + 1];

volatile esp_a2d_audio_state_t playbackState;
volatile esp_a2d_connection_state_t connectionState;
volatile bool muted = false;
volatile uint8_t disp_rotate = 0;

TaskHandle_t xHandle;

extern "C" void app_main(void)
{
    if (pthread_mutex_init(&canMutex, NULL) != 0)
    {
        ESP_LOGE("BTSAAB", "Failed to initialize the can mutex");
    }

    setupCAN();

    showWelcome();

    // xTaskCreate(CAN_SEND_Control_Task, "CAN_SEND_Control_Task", 2048, NULL, 1, NULL);
    xTaskCreate(SIDAudioTextControl, "SID audio text controller", 2048, NULL, 1, NULL);
    xTaskCreate(SIDControl, "SID audio text generator", 2048, NULL, 1, &xHandle);

    setupAudio();
}
/*
void CAN_SEND_Control_Task(void *params)
{
    while (true)
    {
        can_frame frame;
        if (xQueueReceive(sendQueue, &frame, 10000) == pdTRUE)
        {
            if (pthread_mutex_lock(&canMutex) == 0)
            {
                MCP2515::ERROR err = mcp2515.sendMessage(&frame);
                if (err != MCP2515::ERROR_OK)
                {
                    printf("Error sending frame: %d\n", err);
                }
                pthread_mutex_unlock(&canMutex);
            }
            delay(10);
        }
    }
}
*/

void CAN_RECV_Control_Task(void *params)
{
    int pinNumber = 0;
    can_frame recv_frame;
    for (;;)
    {
        if (xQueueReceive(interruptQueue, &pinNumber, 10) == pdTRUE)
        {
            if (pinNumber == 34)
            {
                if (pthread_mutex_lock(&canMutex) == 0)
                {
                    uint8_t irq = mcp2515.getInterrupts();
                    if (irq & MCP2515::CANINTF_RX0IF)
                    {
                        if (mcp2515.readMessage(MCP2515::RXB0, &recv_frame) == MCP2515::ERROR_OK)
                        {
                            frameHandler(&recv_frame);
                        }
                    }
                    if (irq & MCP2515::CANINTF_RX1IF)
                    {
                        if (mcp2515.readMessage(MCP2515::RXB1, &recv_frame) == MCP2515::ERROR_OK)
                        {
                            frameHandler(&recv_frame);
                        }
                    }
                    if ((irq & MCP2515::CANINTF_ERRIF) || (irq & MCP2515::CANINTF_MERRF))
                    {
                        checkErr(mcp2515);
                        mcp2515.clearERRIF();
                        mcp2515.clearMERR();
                    }

                    /*
                    if (irq & MCP2515::CANINTF_TX0IF)
                    {
                        printf("TX0IF\n");
                    }
                    if (irq & MCP2515::CANINTF_TX1IF)
                    {
                        printf("TX1IF\n");
                    }
                    if (irq & MCP2515::CANINTF_TX2IF)
                    {
                        printf("TX2IF\n");
                    }
                    */

                    pthread_mutex_unlock(&canMutex);
                }
            }
        }
    }
}

void SIDAudioTextControl(void *params)
{

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 93;
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {

        xTaskDelayUntil(&xLastWakeTime, xFrequency);
        can_frame frame = {
            .can_id = 0x348,
            .can_dlc = 8,
            .data = {
                0x11,
                0x02,
                (connectionState == ESP_A2D_CONNECTION_STATE_CONNECTED) ? (uint8_t)0x05 : (uint8_t)0xFF,
                0x19,
                0x04,
                0x00,
                0x00,
                0x00,
            },
        };
        sendCAN(&frame);
    }
}

void SIDControl(void *params)
{
    uint8_t ca, ct;
    char displayBuffer[SCREEN_BUFFER_LENGTH + 1];
    char onscreen[SCREEN_WIDTH + 1];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        if (connectionState != ESP_A2D_CONNECTION_STATE_CONNECTED)
        {
            vTaskSuspend(NULL);
            xLastWakeTime = xTaskGetTickCount();
            disp_rotate = 0;
            continue;
        }

        int bufferPos = 0;

        ca = strlen(currentArtist);
        memcpy(&displayBuffer[bufferPos], currentArtist, ca);
        bufferPos += ca;

        displayBuffer[bufferPos++] = ' ';
        displayBuffer[bufferPos++] = '-';
        displayBuffer[bufferPos++] = ' ';

        ct = strlen(currentTitle);
        memcpy(&displayBuffer[bufferPos], currentTitle, ct);
        bufferPos += ct;

        displayBuffer[bufferPos++] = ' ';
        displayBuffer[bufferPos++] = ' ';
        displayBuffer[bufferPos++] = '\0';

        int dispLen = ca + ct + 3;

        if (dispLen > SCREEN_WIDTH)
        {
            rotateStringLeft(displayBuffer, disp_rotate);
            disp_rotate += ROTATE_STEPS;
            if (disp_rotate >= dispLen)
            {
                disp_rotate = 0;
            }
        }
        uint8_t toCopy = (dispLen > SCREEN_WIDTH) ? SCREEN_WIDTH : dispLen;
        memcpy(onscreen, displayBuffer, toCopy);
        onscreen[toCopy] = '\0';
        setRadioText(onscreen);
        xTaskDelayUntil(&xLastWakeTime, SID_UPDATE_RATE);
    }
}

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interruptQueue, &pinNumber, NULL);
}

void sendCAN(can_frame *frame)
{
    if (pthread_mutex_lock(&canMutex) == 0)
    {
        MCP2515::ERROR err = mcp2515.sendMessage(frame);
        if (err != MCP2515::ERROR_OK)
        {
            ESP_LOGE("CanShield", "Error sending frame: %d", err);
        }
        pthread_mutex_unlock(&canMutex);
    }
}

/*
290h - Steering wheel and SID buttons
Message is sent with an interval of 1 second and if a value changes.
The most significant bit of the first byte (byte 0) is set if information has changed from the last message.
The audio button information and SID button information are duplicated to bytes 4 and 5.

ID	Byte 0	Byte 1	Byte 2	Byte 3	Byte 4	Byte 5	Byte 6	Byte 7
290h	STATE	-	AUDIO	SID	AUDIO	SID	-	-

Byte	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0
STATE	CHANGED	-	-	-	-	-	-	-
AUDIO	VOL-	VOL+	SRC	SEEK+	SEEK-	NXT	-	-
SID	CLR	SET	DOWN	UP	NPANEL	CLOCK+	CLOCK-	-

Example message:
80 00 40 00 40 00 00 00

Information has changed from the last message and the Volume Up audio button has been pressed.
*/

void frameHandler(can_frame *frame)
{
    if (frame->can_id == 0x290) // Steering wheel and SID buttons
    {
        if (frame->data[5] == 0xC0) // holding CLEAR and SET
        {
        }
        if (frame->data[5] == 0xA0) // holding CLEAR and -
        {
            esp_restart(); // reset the ESP32
        }
    }
    if (frame->can_id == 0x410)
    {
        if (frame->data[5] == 0x01) // night panel enabled
        {
            if (!muted)
            {
                printf("mute\n");
                gpio_set_level(ENABLE_OUTPUT_PIN, 0);
                muted = true;
            }
        }
        else if (frame->data[5] == 0x00)
        {
            if (muted)
            {
                printf("unmute\n");
                gpio_set_level(ENABLE_OUTPUT_PIN, 1);
                muted = false;
            }
        }
    }
    // framePrint(f);
}

void setupCAN()
{
    xTaskCreate(CAN_RECV_Control_Task, "CAN receive controller", 2048, NULL, 1, NULL);

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
        .clock_speed_hz = SPI_MHZ * 1000000,
        .spics_io_num = CAN_CS,
        .queue_size = 7};
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
    // mcp2515.setFilter(MCP2515::RXF1, false, 0x410);
    mcp2515.setNormalMode();
}

void setupAudio()
{

    gpio_set_direction(ENABLE_OUTPUT_PIN, GPIO_MODE_OUTPUT);
    gpio_pulldown_en(ENABLE_OUTPUT_PIN);
    gpio_pullup_dis(ENABLE_OUTPUT_PIN);
    gpio_set_level(ENABLE_OUTPUT_PIN, 1);

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
        .sample_rate = 44100, // updated automatically by A2DP
        .bits_per_sample = (i2s_bits_per_sample_t)32,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 32,
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
    if (id == 0x01) // title
    {
        snprintf(currentTitle, MAX_TITLE_LENGTH + 1, "%s", value);
        utf8_to_sid(currentTitle);
        utf_convert(currentTitle, currentTitle, 64);
        return;
    }
    else if (id == 0x02) // artist
    {
        snprintf(currentArtist, MAX_ARTIST_LENGTH + 1, "%s", value);
        utf8_to_sid(currentArtist);
        utf_convert(currentArtist, currentArtist, 64);
        disp_rotate = 0;
        return;
    }
    // printf("AVRC metadata rsp: attribute id 0x%x, %d\n", id, &value);
}

void audio_state_callback(esp_a2d_audio_state_t state, void *data)
{
    playbackState = state;
    char stateString[8];
    switch (state)
    {
    case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
        strcpy(stateString, "SUSPEND");
        gpio_set_level(ENABLE_OUTPUT_PIN, 0);
        break;
    case ESP_A2D_AUDIO_STATE_STOPPED:
        strcpy(stateString, "STOPPED");
        gpio_set_level(ENABLE_OUTPUT_PIN, 0);
        break;
    case ESP_A2D_AUDIO_STATE_STARTED:
        strcpy(stateString, "STARTED");
        gpio_set_level(ENABLE_OUTPUT_PIN, 1);
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
        vTaskResume(xHandle);
        break;
    case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
        printf("A2DP disconnecting\n");
        break;
    }
}
