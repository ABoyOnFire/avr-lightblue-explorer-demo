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

#include "../include/usart1.h"


/* Static Variables holding the ringbuffer used in IRQ mode */
static uint8_t          USART1_rxbuf[USART1_RX_BUFFER_SIZE];
static volatile uint8_t USART1_rx_head;
static volatile uint8_t USART1_rx_tail;
static volatile uint8_t USART1_rx_elements;
static uint8_t          USART1_txbuf[USART1_TX_BUFFER_SIZE];
static volatile uint8_t USART1_tx_head;
static volatile uint8_t USART1_tx_tail;
static volatile uint8_t USART1_tx_elements;

void (*USART1_rx_isr_cb)(void) = &USART1_DefaultRxIsrCb;

void (*USART1_tx_isr_cb)(void) = &USART1_DefaultTxIsrCb;

void USART1_DefaultRxIsrCb(void)
{
    uint8_t data;
    uint8_t tmphead;

    /* Read the received data */
    data = USART1.RXDATAL;
    /* Calculate buffer index */
    tmphead = (USART1_rx_head + 1) & USART1_RX_BUFFER_MASK;
        
    if (tmphead == USART1_rx_tail) {
            /* ERROR! Receive buffer overflow */
    }else {
    /*Store new index*/
    USART1_rx_head = tmphead;
    
    /* Store received data in buffer */
    USART1_rxbuf[tmphead] = data;
    USART1_rx_elements++;
    }
}

void USART1_DefaultTxIsrCb(void)
{
    uint8_t tmptail;

    /* Check if all data is transmitted */
    if (USART1_tx_elements != 0) {
        /* Calculate buffer index */
        tmptail = (USART1_tx_tail + 1) & USART1_TX_BUFFER_MASK;
        /* Store new index */
        USART1_tx_tail = tmptail;
        /* Start transmission */
        USART1.TXDATAL = USART1_txbuf[tmptail];
        
        USART1_tx_elements--;
    }

    if (USART1_tx_elements == 0) {
            /* Disable Tx interrupt */
            USART1.CTRLA &= ~(1 << USART_DREIE_bp);
    }
}

void USART1_SetISRCb(usart_callback cb, usart1_cb_t type)
{
    switch (type) {
        case USART1_RX_CB:
                USART1_rx_isr_cb = cb;
                break;
        case USART1_TX_CB:
                USART1_tx_isr_cb = cb;
                break;
        default:
                // do nothing
                break;
    }
}

void USART1_SetRXISRCb(usart_callback cb)
{
    USART1_SetISRCb(cb,USART1_RX_CB);
}

void USART1_SetTXISRCb(usart_callback cb)
{
    USART1_SetISRCb(cb,USART1_TX_CB);
}

/* Interrupt service routine for RX complete */
ISR(USART1_RXC_vect)
{
    if (USART1_rx_isr_cb != NULL)
    {
        (*USART1_rx_isr_cb)();
    }
}

/* Interrupt service routine for Data Register Empty */
ISR(USART1_DRE_vect)
{
    if (USART1_tx_isr_cb != NULL)
    {
        (*USART1_tx_isr_cb)();
    }
}

ISR(USART1_TXC_vect)
{
    USART1.STATUS |= USART_TXCIF_bm;
}

bool USART1_IsTxReady()
{
    return (USART1_tx_elements != USART1_TX_BUFFER_SIZE);
}

bool USART1_IsRxReady()
{
    return (USART1_rx_elements != 0);
}

bool USART1_IsTxBusy()
{
    return (!(USART1.STATUS & USART_TXCIF_bm));
}

bool USART1_IsTxDone()
{
    return (USART1.STATUS & USART_TXCIF_bm);
}

uint8_t USART1_Read(void)
{
    uint8_t tmptail;

    /* Wait for incoming data */
    while (USART1_rx_elements == 0)
            ;
    /* Calculate buffer index */
    tmptail = (USART1_rx_tail + 1) & USART1_RX_BUFFER_MASK;
    /* Store new index */
    USART1_rx_tail = tmptail;
    ENTER_CRITICAL(R);
    USART1_rx_elements--;
    EXIT_CRITICAL(R);

    /* Return data */
    return USART1_rxbuf[tmptail];
}

void USART1_Write(const uint8_t data)
{
    uint8_t tmphead;

    /* Calculate buffer index */
    tmphead = (USART1_tx_head + 1) & USART1_TX_BUFFER_MASK;
    /* Wait for free space in buffer */
    while (USART1_tx_elements == USART1_TX_BUFFER_SIZE)
            ;
    /* Store data in buffer */
    USART1_txbuf[tmphead] = data;
    /* Store new index */
    USART1_tx_head = tmphead;
    ENTER_CRITICAL(W);
    USART1_tx_elements++;
    EXIT_CRITICAL(W);
    /* Enable Tx interrupt */
    USART1.CTRLA |= (1 << USART_DREIE_bp);
}

void USART1_Initialize()
{
    //set baud rate register
    USART1.BAUD = (uint16_t)USART1_BAUD_RATE(115200);
	
    //RXCIE enabled; TXCIE enabled; DREIE disabled; RXSIE enabled; LBME disabled; ABEIE disabled; RS485 OFF; 
    USART1.CTRLA = 0xD0;
	
    //RXEN enabled; TXEN enabled; SFDEN disabled; ODME disabled; RXMODE NORMAL; MPCM disabled; 
    USART1.CTRLB = 0xC0;
	
    //CMODE ASYNCHRONOUS; PMODE DISABLED; SBMODE 1BIT; CHSIZE 8BIT; UDORD disabled; UCPHA disabled; 
    USART1.CTRLC = 0x03;
	
    //DBGCTRL_DBGRUN
    USART1.DBGCTRL = 0x00;
	
    //EVCTRL_IREI
    USART1.EVCTRL = 0x00;
	
    //RXPLCTRL_RXPL
    USART1.RXPLCTRL = 0x00;
	
    //TXPLCTRL_TXPL
    USART1.TXPLCTRL = 0x00;
	

    uint8_t x;

    /* Initialize ringbuffers */
    x = 0;

    USART1_rx_tail     = x;
    USART1_rx_head     = x;
    USART1_rx_elements = x;
    USART1_tx_tail     = x;
    USART1_tx_head     = x;
    USART1_tx_elements = x;

}

void USART1_Enable()
{
    USART1.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

void USART1_EnableRx()
{
    USART1.CTRLB |= USART_RXEN_bm;
}

void USART1_EnableTx()
{
    USART1.CTRLB |= USART_TXEN_bm;
}

void USART1_Disable()
{
    USART1.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

uint8_t USART1_GetData()
{
    return USART1.RXDATAL;
}