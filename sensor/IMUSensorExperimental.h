#pragma once
#include <POP32.h>
#include "../draw/draw.hpp"

DMA_HandleTypeDef hdma_usart1_tx;

void IMU_DMA_Init() {
    UART_HandleTypeDef *huart = Serial1.getHandle();

    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart1_tx.Instance                 = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma_usart1_tx);

    __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

}

extern "C" void DMA1_Channel4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

struct IMUSensor {
    void write(uint8_t data) {
        // while (Serial1.getHandle()->gState != HAL_UART_STATE_READY);

        // HAL_UART_Transmit_DMA(
        //     Serial1.getHandle(),
        //     &data,
        //     1
        // );
    }

    void read() {
        HAL_UART_Receive_DMA(
            Serial1.getHandle(),
            rxBuf,
            8
        );
    }

    double_t cYaw=0,cPitch=0,cRoll=0;
    bool status = false;

    uint8_t rxBuf[8];

    void Reset(){
        ZeroYaw();
        ToggleAutoMode();
    }

    void Init(){
        Serial1.begin(115200);
        IMU_DMA_Init();
        Reset();
    }
    
    void ToggleAutoMode(){
        write(0XA5);write(0X52);
        delay(60);
    }

    void ToggleQueryMode(){
        write(0XA5);write(0X51);
        delay(60);
    }

    // Set Pitch, Roll and Yaw to zero
    // WARNING: delay upto 1950ms
    inline void ZeroYaw() {
        //Zero Pitch
        write(0XA5);write(0X54); 
        delay(60);
        //Zero Yaw
        write(0XA5);write(0X55);
        delay(60);
    }

    // Repeated yaw-zero until near zero
    //
    // 'precision' : the precision to stop zeroing, default is 0.02f
    // 'time_out' : the maximum time to wait for zeroing, default is 10'000ms
    void AutoZero(float_t precision = 0.02f,int32_t time_out = 10000) {
        Reset();
        unsigned long start = HAL_GetTick();
        unsigned long lastReset = start;
        while (true) {
            unsigned long time = HAL_GetTick();
            if (fetchIMU()) {
                if (time - lastReset > 1000) {
                    if (fabs(cYaw) <= precision && fabs(cYaw) != 0.0) break;                    
                    Reset();
                    lastReset = time;
                }
            }
        }
    }

    // Fetch data from IMU sensor, return true if successful
    // pvYaw will be updated with the latest yaw value
    inline bool fetchIMU() {
        static int8_t idx = 0;
        while (Serial1.available()){
            uint8_t byte = Serial1.read();
            if (idx == 0 && byte != 0xAA) continue;
                rxBuf[idx++] = byte;
                if (idx == 8) {
                    idx = 0;
                    if (rxBuf[7] == 0x55) {
                        cYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.0f;
                        cPitch = (int16_t)(rxBuf[3] << 8 | rxBuf[4]) / 100.0f;
                        cRoll = (int16_t)(rxBuf[5] << 8 | rxBuf[6]) / 100.0f;
                        status = true;
                        return true;
                    }
            }
        }
        status = false;
        return false;
    }

    // Get the current yaw value
    //
    // 'do_fetch' : true - fetching new data, flase - using previous value
    inline float_t getYaw(bool do_fetch = true){
        if (do_fetch) {
            if (!fetchIMU()) return cYaw; // If fetch failed, return previous value
        }
        return cYaw;
    }
};