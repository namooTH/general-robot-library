#pragma once
#include <POP32.h>
#include "../draw/draw.hpp"

/* Some Hacking */
ACCESS_PRIVATE_FIELD(HardwareSerial, serial_t, _serial);
auto &IMUserial = access_private::_serial(Serial1);

/* The callback which is called on data write */
int txCallback(serial_t *) {
    return 0;
}

static uint8_t rxBuf[8];
bool packetReady = false;

/* The callback which is called on data read */
void rxCallback(serial_t *) {
    packetReady = true;
}

DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

void IMU_DMA_Init()
{
    IMUserial.tx_callback = &txCallback;
    IMUserial.rx_callback = &rxCallback;

    UART_HandleTypeDef *huart = &IMUserial.handle;

    __HAL_RCC_DMA1_CLK_ENABLE();

    // TX: USART1_TX -> DMA1_Channel4
    hdma_usart1_tx.Instance                 = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart1_tx);

    // RX: USART1_RX -> DMA1_Channel5
    hdma_usart1_rx.Instance                 = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart1_rx);

    __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);
    __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

extern "C" void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

extern "C" void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

struct IMUSensor {
    double_t cYaw=0,cPitch=0,cRoll=0;
    bool status = false;

    /**
     * Reset the gyro yaw, and enable auto mode (what's that?).
     * 
     * ARGS:
     *     None.
     * 
     * RETURNS:
     *     Nothing.
     */
    void Reset(){
        ZeroYaw();
        ToggleAutoMode();
    }

    /**
     * Init Serial1, init Direct Memory Access, and reset IMU sensors.
     * 
     * ARGS:
     *     None.
     * 
     * RETURNS:
     *     Nothing.
     */
    void Init(){
        Serial1.begin(115200);
        IMU_DMA_Init();
        Reset();
    }
    
    /**
     * WTF is this function?
     * 
     * ARGS:
     *     None.
     * 
     * RETURNS:
     *     Nothing.
     */
    void ToggleAutoMode(){
        write(0XA5);write(0X52);
        delay(60);
    }

    /**
     * WTF is that function?
     * 
     * ARGS:
     *     None.
     * 
     * RETURNS:
     *     Nothing.
     */
    void ToggleQueryMode(){
        write(0XA5);write(0X51);
        delay(60);
    }

    /**
     * Reset the pitch/yaw of the gyroscope.
     * 
     * ARGS:
     *     None.
     * 
     * RETURNS:
     *     Nothing.
     */
    inline void ZeroYaw() {
        //Zero Pitch
        write(0XA5);write(0X54); 
        delay(60);
        //Zero Yaw
        write(0XA5);write(0X55);
        delay(60);
    }

    /**
     * Yaw-zero repeatly until near zero to some precision.
     * 
     * ARGS:
     *     float precision [DEFAULT=0.02] : The precision to stop zero-ing.
     *     int32_t timeout [DEFAULT=10000] : The maximum time in milliseconds to wait for zero-ing before stoping.
     * 
     * RETURNS:
     *     Nothing.
     */
    void AutoZero(float precision = 0.02f, int32_t timeout = 10000) {
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

            if (lastReset - time >= timeout) break;
        }
    }

    /**
     * Fetch data from the IMU sensor, and update angle values.
     * 
     * ARGS:
     *     None.
     * 
     * RETURNS:
     *     bool : Whether fetching is successful.
     */
    inline bool fetchIMU() {
        if (packetReady)
        {
            packetReady = false;

            if (rxBuf[0] == 0xAA && rxBuf[7] == 0x55)
            {
                cYaw   = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.0f;
                cPitch = (int16_t)(rxBuf[3] << 8 | rxBuf[4]) / 100.0f;
                cRoll  = (int16_t)(rxBuf[5] << 8 | rxBuf[6]) / 100.0f;
            }
            read();
            
            status = true;
            return true;
        }
        
        read();

        status = false;
        return false;
    }

    /**
     * Get the current yaw value
     * 
     * ARGS:
     *     bool do_fetch [DEFAULT=true] : Whether fetch new data (true) or use previous value (false).
     *
     * RETURNS:
     *     Nothing.
     */
    inline float_t getYaw(bool do_fetch = true){
        if (do_fetch) {
            if (!fetchIMU()) return cYaw; // If fetch failed, return previous value
        }
        return cYaw;
    }

    private:
        /**
         * Wait until the IMU is ready for read/write.
         * 
         * ARGS:
         *     None.
         * 
         * RETURNS:
         *     Nothing.
         */
        void waitTillReady() {
            while (IMUserial.handle.gState != HAL_UART_StateTypeDef::HAL_UART_STATE_READY);
        }

        /**
         * Write a byte of data to IMU using DMA.
         * 
         * ARGS:
         *     uint8_t data : The data.
         * 
         * RETURNS:
         *     Nothing.
         */
        void write(uint8_t data) {
            waitTillReady();

            HAL_UART_Transmit_DMA(
                &IMUserial.handle,
                &data,
                1
            );
        }

        /**
         * Read data from IMU using DMA, and store it inside rxBuf.
         * 
         * ARGS:
         *     None.
         * 
         * RETURNS:
         *     Nothing.
         */
        void read() {
            waitTillReady();
            
            HAL_UART_Receive_DMA(
                &IMUserial.handle,
                rxBuf,
                8
            );
        }
};