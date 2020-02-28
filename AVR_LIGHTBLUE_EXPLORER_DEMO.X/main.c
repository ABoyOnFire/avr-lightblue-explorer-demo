/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/
#include <string.h>
#include "mcc_generated_files/mcc.h"
#include "application/LIGHTBLUE_service.h"
#include "rn4870-1-ble-module/rn487x_interface.h"
#include "rn4870-1-ble-module/rn487x.h"
#include "drivers/uart.h"

#include "application/VR_Service.h"

/** MACRO used to reference Periodic Timer overflow flag Set. 
 *  This is used by the application to have a semi-accurate 
 *  periodic task execution rate. 
 *  Strict adherence to time interval is not required.
 */
#define TIMER_FLAG_SET()                (TCA0_IsOverflowInterruptEnabled())
/** MACRO used to reset the Periodic Timer overflow flag.
 *  This is used by the application to reload the semi-accurate
 *  periodic task execution.
 *  The rate allows for a (100%) drift prior to error
 *  Is susceptible to effect by extended BLE communication. 
 */
#define RESET_TIMER_INTERRUPT_FLAG      (TCA0.SINGLE.INTFLAGS = 1)
/** MACRO used to configure the application used buffer sizes.
 *  This is used by the application for communication buffers.
 */
#define MAX_BUFFER_SIZE                 (80)

static char statusBuffer[MAX_BUFFER_SIZE];      /**< Status Buffer instance passed to RN487X drive used for Asynchronous Message Handling (see *asyncBuffer in rn487x.c) */
static char lightBlueSerial[MAX_BUFFER_SIZE];   /**< Message Buffer used for CDC Serial communication when connected. Terminated by \r, \n, MAX character Passes messages to BLE for transmisison. */
static uint8_t serialIndex;                     /**< Local index value for serial communication buffer. */

static bool bleTask = false;
static bool vrTask = false;
static uint8_t bleCounter = 0;
static uint8_t vrCounter = 0;
static bool triggerPrint = false;
static char vrSerial[80];
/*
    Main application
*/
int main(void)
{
    volatile char readByte;
    volatile bool connected = false;
    /* Initializes MCU, drivers and middleware */
    SYSTEM_Initialize();
    RN487X_SetAsyncMessageHandler(statusBuffer, sizeof(statusBuffer));
    
    ENABLE_INTERRUPTS();
    
    RN487X_Init();

    while (1)
    {
        if (TIMER_FLAG_SET() == true)
        {
            RESET_TIMER_INTERRUPT_FLAG;
            bleCounter++;
            vrCounter++;
        }
        connected = RN487X_IsConnected();
        if (connected == true)
        {
            if (bleCounter > 10)
            {
                bleTask = true;
            }
            if (bleTask == true)
            {
                LIGHTBLUE_TemperatureSensor();
                LIGHTBLUE_AccelSensor();
                LIGHTBLUE_PushButton();
                LIGHTBLUE_LedState();
                LIGHTBLUE_SendProtocolVersion();
                
                bleCounter = 0;
                bleTask = false;
            }
            else 
            {
                while (RN487X_DataReady())
                {
                    LIGHTBLUE_ParseIncomingPacket(RN487X_Read());
                }
                while (uart[UART_CDC].DataReady())
                {
                    lightBlueSerial[serialIndex] = uart[UART_CDC].Read();
                    if ((lightBlueSerial[serialIndex] == '\r')
                        || (lightBlueSerial[serialIndex] == '\n')
                        || (serialIndex == (sizeof(lightBlueSerial) - 1)))
                    {
                        lightBlueSerial[serialIndex] = '\0';
                        strcpy(vrSerial, lightBlueSerial);
                        LIGHTBLUE_SendSerialData(lightBlueSerial);
                        VR_SerialData(vrSerial);
                        strcpy(vrSerial, lightBlueSerial);
                        serialIndex = 0;
                    }
                    else
                    {
                        serialIndex++;
                    }
                }
            }
        }
        else
        {
            while(RN487X_DataReady())
            {
                uart[UART_CDC].Write(RN487X_Read());
            }
            while (uart[UART_CDC].DataReady())
            {
                RN487X.Write(uart[UART_CDC].Read());
            }
        }
        if (connected == false)
        {
            if (vrCounter > 1)
            {
                vrTask = true;
            }
        }
        else
        {
            if (vrCounter > 5)
            {
                vrTask = true;
            }
        }
        if (vrTask == true)
        {
            triggerPrint = true;
        }
        if (triggerPrint == true)
        {
            VR_AllData(connected);
            vrTask = false;
            triggerPrint = false;
            vrCounter = 0;
        }
        if (uart[UART_VR].DataReady())
        {
            readByte = uart[UART_VR].Read();

            if (readByte == '0')
            {
                DATA_LED_SetHigh();
            }
            if (readByte == '1')
            {
                DATA_LED_SetLow();
            }
            if (readByte == '3')
            {
                ERROR_LED_SetHigh();
            }
            if (readByte == '4')
            {
                ERROR_LED_SetLow();
            }
        }
    }
}
/**
    End of File
*/