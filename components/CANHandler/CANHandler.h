#include "mcp2515.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include <cstdint>
#include <vector>

void interruptHandler(void *params);
void frameHandler(void *params);

typedef void (*function_ptr)();
typedef void (*CB)(can_frame *frame);

typedef struct
{
    uint32_t id;
    CB callback;
} callback_t;

class CANHandler
{
public:
    spi_device_handle_t *spi;
    CANHandler();
    CANHandler(spi_device_handle_t *s);
    void start();
    void add_callback(callback_t *callback);
    void process_frame(can_frame *frame);
    void send_frame(can_frame *frame);
    pthread_mutex_t canMutex;
    xQueueHandle interruptQueue = nullptr;
    xQueueHandle canQueue = nullptr;
    MCP2515 *mcp2515;

private:
    std::vector<callback_t> callbacks;

protected:
};
