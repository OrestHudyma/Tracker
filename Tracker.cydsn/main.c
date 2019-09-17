/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"

#define NMEA_MAX_SIZE       82

char NMEA_buffer[NMEA_MAX_SIZE];
char NMEA_GPRMC[NMEA_MAX_SIZE];

uint8 NMEA_pointer;


void NMEA_handle_packet();
void wake_up_handler();

CY_ISR(GPS_receive)
{
    NMEA_buffer[NMEA_pointer] = UART_GPS_GetByte();
    switch(NMEA_buffer[NMEA_pointer])
    {
        case '$':
        NMEA_pointer = 0;
        break;
        
        case 0x0A:
        NMEA_handle_packet();
        break;
    }
}
       

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    isr_GPS_received_StartEx(GPS_receive);

    CySysWdtSetInterruptCallback(CY_SYS_WDT_COUNTER2, (cyWdtCallback) wake_up_handler);
    

    for(;;)
    {
        CySysPmDeepSleep();
        
    }
}

void wake_up_handler()
{
}

void NMEA_handle_packet()
{
    uint8 i;
    uint8 error = 0;
    
    // Check for receive errors
    for(i = 0; i <= NMEA_MAX_SIZE; i++)
    {
        if (NMEA_buffer[i] == 0) 
        {
            error++;
            break;
        }
    }
    
    // Copy buffer to NMEA packet if no errors found
    if (!error)
    {
        for(i = 0; i <= NMEA_MAX_SIZE; i++)
        {
            NMEA_GPRMC[i] = NMEA_buffer[i];
        }
    }
}

/* [] END OF FILE */
