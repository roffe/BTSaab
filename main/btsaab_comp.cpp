#include "esp_random.h"

const char *getCompliment()
{
    const char *compliments[30] = {
        "You are a shining star",
        "You light up my day",
        "You are a great friend",
        "You are a fantastic listener",
        "You are an inspiration",
        "You are a true leader",
        "You are a valuable asset",
        "You are a rare gem",
        "You have a unique talent",
        "You are a blessing",
        "You are a joy to be around",
        "You have a kind heart",
        "You are a great role model",
        "You are a master in your craft",
        "You have a wonderful attitude",
        "You are a natural at what you do",
        "You are a creative genius",
        "You are a brilliant mind",
        "You are a visionary",
        "You are a valuable member",
        "You are a treasure",
    };

    int index = esp_random() % 30;
    return compliments[index];
}