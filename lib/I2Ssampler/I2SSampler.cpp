
#include <Arduino.h>
#include "I2SSampler.h"
#include "driver/i2s.h"

void I2SSampler::addSample(int32_t sample)
{
    // add the sample to the current audio buffer
    m_currentAudioBuffer[m_audioBufferPos] = sample;
    m_audioBufferPos++;
    // have we filled the buffer with data?
    if (m_audioBufferPos == m_bufferSizeInSamples)
    {
        // swap to the other buffer
        std::swap(m_currentAudioBuffer, m_capturedAudioBuffer);
        // reset the buffer position
        m_audioBufferPos = 0;
        // tell the writer task to save the data
        xTaskNotify(m_writerTaskHandle, 1, eIncrement);
    }
}

void i2sReaderTask(void *param)
{
    I2SSampler *sampler = (I2SSampler *)param;
    while (true)
    {
        // wait for some data to arrive on the queue
        i2s_event_t evt;
        if (xQueueReceive(sampler->m_i2sQueue, &evt, portMAX_DELAY) == pdPASS)
        {
            if (evt.type == I2S_EVENT_RX_DONE)
            {
                // Read only the bytes associated with this event so we return
                // to queue processing promptly and avoid RX queue overflow.
                size_t remaining = evt.size;
                if (remaining == 0)
                {
                    remaining = 1024;
                }

                while (remaining > 0)
                {
                    uint8_t i2sData[1024];
                    size_t request = (remaining > sizeof(i2sData)) ? sizeof(i2sData) : remaining;
                    size_t bytesRead = 0;
                    i2s_read(sampler->getI2SPort(), i2sData, request, &bytesRead, portMAX_DELAY);
                    if (bytesRead == 0)
                    {
                        break;
                    }

                    sampler->processI2SData(i2sData, bytesRead);

                    if (bytesRead >= remaining)
                    {
                        break;
                    }
                    remaining -= bytesRead;
                }
            }
            else if (evt.type == I2S_EVENT_RX_Q_OVF)
            {
                // Drain a small chunk to help the peripheral recover.
                uint8_t discard[256];
                size_t bytesRead = 0;
                i2s_read(sampler->getI2SPort(), discard, sizeof(discard), &bytesRead, 0);
            }
        }
    }
}

void I2SSampler::start(i2s_port_t i2sPort, i2s_config_t &i2sConfig, int32_t bufferSizeInBytes, TaskHandle_t writerTaskHandle)
{
    m_i2sPort = i2sPort;
    m_writerTaskHandle = writerTaskHandle;
    m_bufferSizeInSamples = bufferSizeInBytes / sizeof(int32_t);
    m_bufferSizeInBytes = bufferSizeInBytes;
    m_audioBuffer1 = (int32_t *)malloc(bufferSizeInBytes);
    m_audioBuffer2 = (int32_t *)malloc(bufferSizeInBytes);

    m_currentAudioBuffer = m_audioBuffer1;
    m_capturedAudioBuffer = m_audioBuffer2;

    m_writerTaskHandle = writerTaskHandle;
    //install and start i2s driver
    i2s_driver_install(m_i2sPort, &i2sConfig, 16, &m_i2sQueue);
    // set up the I2S configuration from the subclass
    configureI2S();
    // start a task to read samples from the ADC
    TaskHandle_t readerTaskHandle;
    xTaskCreatePinnedToCore(i2sReaderTask, "i2s Reader Task", 4096, this, 1, &readerTaskHandle, 0);
}
