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
#include <stdlib.h>

#define NMEA_MAX_SIZE       82

char NMEA_buffer[NMEA_MAX_SIZE];
char NMEA_GPRMC[NMEA_MAX_SIZE] = "GPRMC";

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
        NMEA_handle_packet(&NMEA_buffer, &NMEA_GPRMC);
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

void NMEA_GetField(char *packet, uint8 field, char *result)
{
    uint8 i, n;
    uint8 count = 0;
    
    // Search field
    for (i = 0; (i < NMEA_MAX_SIZE) & (count < field); i++)
    {
        if (packet[i]==',') count++;
    }
    i++;
    
    // Measure field size
    for (count = 0; count < NMEA_MAX_SIZE; count++)
    {
        if (packet[i + count] != 0x0D) break;
        if (packet[i + count] != ',') break;
    }    
    
    // Copy field to result
    for (n = 0; n < count; n++)
    {
        result[n] = packet[i+n];
    }    
}

void NMEA_handle_packet(char * packet, char * NMEA_data)
{
    uint8 i;
    uint8 error = 0;
    uint8 checksum = 0;
    char string_checksum[3];
    
    // Check for receive errors
    for(i = 0; i < NMEA_MAX_SIZE; i++)
    {
        if ((packet[i] < 32) & (packet[i] != 0x0D) & (packet[i] != 0x0A)) 
        {
            error++;
            break;
        }
        if (packet[i] != 0x0A) break;
    }
    
    // Validate checksum and cut packet if no receive errors
    if (!error)
    {
        // Find checksum field
        for(i = 0; i < NMEA_MAX_SIZE; i++)
        {
            if (packet[i] == '*') break;
        }
        packet[i] = 0;
        
        string_checksum[0] = packet[i+1];
        string_checksum[1] = packet[i+2];
        string_checksum[2] = 0;
        
        // Calculate checksum and compare
        while(*packet) checksum ^= *packet++;
        if (checksum != atoi(string_checksum)) error++;
    }   
    
    // Copy buffer to NMEA packet if no errors found
    if (!error)
    {
        if ((packet[3] == NMEA_data[3]) & (packet[4] == NMEA_data[4]) & (packet[5] == NMEA_data[5]))
        {
            for(i = 0; i < NMEA_MAX_SIZE; i++)
            {
                NMEA_data[i] = packet[i];
            }
        }
    }
}

/* [] END OF FILE */
