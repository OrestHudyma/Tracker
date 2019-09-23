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

#define NMEA_MAX_SIZE             82
#define NMEA_START_DELIMITER      '$'
#define NMEA_END_DELIMITER        0x0A
#define NMEA_CHECKSUM_DELIMITER   '*'
#define NMEA_FIELD_DELIMITER      ','

#define NMEA_GPRMC_LATITUDE         3
#define NMEA_GPRMC_LONGITUDE        5
#define NMEA_GPRMC_UTC              1
#define NMEA_GPRMC_SPEED            7

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
        case NMEA_START_DELIMITER:
        NMEA_pointer = 0;
        Pin_GPS_RxLED_Write(1);
        break;
        
        case NMEA_END_DELIMITER:
        if (NMEA_buffer[2] == 'R')
        {
            NMEA_handle_packet(&NMEA_buffer, &NMEA_GPRMC);
        }
        Pin_GPS_RxLED_Write(0);
        break;
        
        default:
        NMEA_pointer++;
        break;
    }
}
       

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    isr_GPS_received_StartEx(GPS_receive);
    UART_GPS_Start();

    CySysWdtSetInterruptCallback(CY_SYS_WDT_COUNTER2, (cyWdtCallback) wake_up_handler);
    
    for(;;)
    {
       
        
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
        if (packet[i] == NMEA_FIELD_DELIMITER) count++;
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

void NMEA_handle_packet(char *packet, char *NMEA_data)
{
    uint8 i, n;
    uint8 error = 0;
    uint8 checksum = 0;
    char packet_checksum[3];
    char calculated_checksum[3];
    
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
            if (packet[i] == NMEA_CHECKSUM_DELIMITER) break;
        }
        packet[i] = 0;
        
        packet_checksum[0] = packet[i+1];
        packet_checksum[1] = packet[i+2];
        packet_checksum[2] = 0;
        
        // Calculate checksum and compare
        for (n = 0; n < i; n++)
        {
            checksum ^= packet[n];
        }
        itoa(checksum, calculated_checksum, 16);
        if (calculated_checksum[0] != packet_checksum[0]) error++;
        if (calculated_checksum[1] != packet_checksum[1]) error++;
    }   
    
    // Copy buffer to NMEA packet if no errors found
    if (!error)
    {
        if ((packet[2] == NMEA_data[2]) & (packet[3] == NMEA_data[3]) & (packet[4] == NMEA_data[4]))
        {
            for(i = 0; i < NMEA_MAX_SIZE; i++)
            {
                NMEA_data[i] = packet[i];
                Pin_GPS_RxLED_Write(1);
            }
        }
    }
}

/* [] END OF FILE */
