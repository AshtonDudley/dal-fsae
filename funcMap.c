#include <stdio.h>
#include <string.h>

#define uint16_t unsigned int
#define MXN 20

void TIM_ChangeThrottleMap();

float map[11]= {0.0f, 409.6f, 819.2f, 1228.8f, 1638.4f, 2048.0f, 2457.6f, 2867.2f, 3276.8f, 3686.4f, 4096.0f};

void TIM_ChangeThrottleMap() {

    static uint16_t currentMap = 0;

    float mapsArr[MXN][11] = {
            {0.0f, 409.6f, 819.2f, 1228.8f, 1638.4f, 2048.0f, 2457.6f, 2867.2f, 3276.8f, 3686.4f, 4096.0f},
            {1228.8f, 1515.52f, 1802.24f, 2088.96f, 2375.68f, 2662.4f, 2949.12f, 3235.84f, 3522.56f, 3809.28f, 4096.0f},
            {614.4f, 819.2f, 1024.0f, 1228.8f, 1638.4f, 2048.0f, 2457.6f, 2867.2f, 3276.8f, 3686.4f, 4096.0f},
            {0.0f, 	102.4f, 307.2f, 512.0f, 819.2f, 1228.8f, 1638.4f, 2048.0f, 2457.6f, 3072.0f, 4096.0f}
        };

    if (currentMap < 4) {
        currentMap++;
    } else {
        currentMap = 0;
    }
    memcpy(map, mapsArr[currentMap], sizeof(mapsArr[currentMap]));
}

// demonstates the change in map
int main(void) {
    for (int i = 0; i < 12; i++) {
        printf("%.2f ", map[i]);
    }
    TIM_ChangeThrottleMap();
    printf("\n");
    for (int i = 0; i < 12; i++) {
        printf("%.2f ", map[i]);
    }
}
