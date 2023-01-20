#include "CANHandler.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <cstdint>
#include <cstdio>
#include <vector>
#include <pthread.h>
#include "driver/gpio.h"

CANHandler *can;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(can->interruptQueue, &pinNumber, NULL);
}

CANHandler::CANHandler()
{
}

CANHandler::CANHandler(spi_device_handle_t *s)
{
    can = this;
    gpio_pad_select_gpio(GPIO_NUM_34);
    gpio_set_direction(GPIO_NUM_34, GPIO_MODE_INPUT);
    gpio_pulldown_en(GPIO_NUM_34);
    gpio_pullup_dis(GPIO_NUM_34);
    gpio_set_intr_type(GPIO_NUM_34, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_34, gpio_interrupt_handler, (void *)GPIO_NUM_34);

    if (canQueue == NULL)
        canQueue = xQueueCreate(10, sizeof(can_frame));

    if (interruptQueue == NULL)
        interruptQueue = xQueueCreate(5, sizeof(int));

    if (pthread_mutex_init(&canMutex, NULL) != 0)
    {
        ESP_LOGE("BTSAAB", "Failed to initialize the can mutex");
    }

    xTaskCreate(interruptHandler, "CAN interrupt handler", 4096, NULL, 1, NULL);
    xTaskCreate(frameHandler, "CAN frame handler", 4096, NULL, 1, NULL);
    mcp2515 = new MCP2515(s);
}

void CANHandler::start()
{
    mcp2515->reset();
    mcp2515->setBitrate(CAN_47KBPS, MCP_8MHZ);
    mcp2515->setFilterMask(MCP2515::MASK0, false, 0x7FF);
    mcp2515->setFilterMask(MCP2515::MASK1, false, 0x7FF);
    mcp2515->setFilter(MCP2515::RXF0, false, 0x290); // buttons
    mcp2515->setFilter(MCP2515::RXF1, false, 0x069); // custom messages
    mcp2515->setFilter(MCP2515::RXF2, false, 0x368); // SID controll messages
    mcp2515->setNormalMode();
}

void CANHandler::send_frame(can_frame *frame)
{
    if (pthread_mutex_lock(&canMutex) == 0)
    {
        MCP2515::ERROR err = mcp2515->sendMessage(frame);
        if (err != MCP2515::ERROR_OK)
        {
            ESP_LOGE("CanShield", "Error sending frame: %d", err);
        }
        pthread_mutex_unlock(&canMutex);
    }
}

void CANHandler::add_callback(callback_t *callback)
{
    callbacks.push_back(*callback);
}

void CANHandler::process_frame(can_frame *frame)
{
    for (auto cb : callbacks)
    {
        if (cb.id == frame->can_id)
        {
            cb.callback(frame);
        }
    }
}

void interruptHandler(void *params)
{
    int pinNumber = 0;
    for (;;)
    {
        if (xQueueReceive(can->interruptQueue, &pinNumber, portMAX_DELAY) == pdTRUE)
        {
            if (pinNumber == 34)
            {
                if (pthread_mutex_lock(&can->canMutex) == 0)
                {
                    uint8_t irq = can->mcp2515->getInterrupts();

                    if (irq & MCP2515::CANINTF_RX0IF)
                    {
                        can_frame recv_frame;
                        if (can->mcp2515->readMessage(MCP2515::RXB0, &recv_frame) == MCP2515::ERROR_OK)
                        {
                            if (xQueueSend(can->canQueue, &recv_frame, 1) != pdTRUE)
                            {
                                ESP_LOGE("CanShield", "Failed to send frame to can queue");
                            }
                        }
                    }

                    if (irq & MCP2515::CANINTF_RX1IF)
                    {
                        can_frame recv_frame;
                        if (can->mcp2515->readMessage(MCP2515::RXB1, &recv_frame) == MCP2515::ERROR_OK)
                        {
                            if (xQueueSend(can->canQueue, &recv_frame, 1) != pdTRUE)
                            {
                                ESP_LOGE("CanShield", "Failed to send frame to can queue");
                            }
                        }
                    }

                    if ((irq & MCP2515::CANINTF_ERRIF) || (irq & MCP2515::CANINTF_MERRF))
                    {
                        // checkErr(mcp2515);
                        can->mcp2515->clearERRIF();
                        can->mcp2515->clearMERR();
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
                    pthread_mutex_unlock(&can->canMutex);
                }
            }
        }
    }
}

void frameHandler(void *params)
{
    can_frame frame;
    for (;;)
    {
        if (xQueueReceive(can->canQueue, &frame, portMAX_DELAY) == pdTRUE)
        {
            can->process_frame(&frame);
        }
    }
    free(&frame);
}