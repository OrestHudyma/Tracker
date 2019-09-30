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

#define GPS_FIX_TIMEOUT_SEC            500
#define GPS_FIX_IMPROVE_DELAY_MS       5000

#define POWER_ON        1
#define POWER_OFF       0

// NMEA definitions
#define NMEA_MAX_SIZE             82
#define NMEA_START_DELIMITER      '$'
#define NMEA_END_DELIMITER        0x0A
#define NMEA_CHECKSUM_DELIMITER   '*'
#define NMEA_FIELD_DELIMITER      ','

#define NMEA_GPRMC_LATITUDE         3
#define NMEA_GPRMC_LONGITUDE        5
#define NMEA_GPRMC_UTC              1
#define NMEA_GPRMC_SPEED            7
#define NMEA_GPRMC_VALIDITY         2

#define NMEA_GPRMC_VALID            'A'
#define NMEA_GPRMC_INVALID          'V'

// GSM definitions

#define GSM_BUFFER_SIZE            100


char NMEA_buffer[NMEA_MAX_SIZE];
char NMEA_GPRMC[NMEA_MAX_SIZE] = "GPRMC";
uint8 NMEA_pointer;

char GSM_buffer[GSM_BUFFER_SIZE];
uint8 GSM_pointer = 0;

void NMEA_handle_packet();
void NMEA_GetField(char *packet, uint8 field, char *result);

uint8 str_cmp(char *str1, char *str2, uint8 start, uint8 stop);
void str_append(char *base, char *add);

void wake_up_handler();

CY_ISR(GPS_receive)
{    
    NMEA_buffer[NMEA_pointer] = UART_GPS_GetByte();
    switch(NMEA_buffer[NMEA_pointer])
    {
        case NMEA_START_DELIMITER:
        NMEA_pointer = 0;
        //Pin_GPS_RxLED_Write(1);
        break;
        
        case NMEA_END_DELIMITER:
        NMEA_handle_packet(&NMEA_buffer, &NMEA_GPRMC);
        //Pin_GPS_RxLED_Write(0);
        break;
        
        default:
        NMEA_pointer++;
        break;
    }
}

CY_ISR(GSM_receive)
{  
    GSM_buffer[GSM_pointer] = UART_GSM_UartGetChar();
}

int main(void)
{
    uint16 t;
    char field_tmp[NMEA_MAX_SIZE];
        
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    isr_GPS_received_StartEx(GPS_receive);
    isr_GSM_received_StartEx(GSM_receive);
    
    UART_GPS_Start();

    CySysWdtSetInterruptCallback(CY_SYS_WDT_COUNTER2, (cyWdtCallback) wake_up_handler);
    
    for(;;)
    {
        
        /*********************** GPS *******************************/
        
        // Apply power to GPS
        Pin_GPS_power_Write(POWER_ON);
        
        // Wait for GPS fix
        for(t = 0; t < GPS_FIX_TIMEOUT_SEC; t++)
        {
            CyDelay(1000);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_VALIDITY, field_tmp);
            if (field_tmp[0] == NMEA_GPRMC_VALID) break;            
        }
        Pin_GPS_RxLED_Write(1);
        CyDelay(GPS_FIX_IMPROVE_DELAY_MS);
        
        // Turn off GPS
        Pin_GPS_power_Write(POWER_OFF);

        
        /*********************** GSM *******************************/
        

        
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
    
    // Measure field size
    for (count = 0; count < NMEA_MAX_SIZE; count++)
    {
        if (packet[i + count] == NMEA_FIELD_DELIMITER) break;
        if (packet[i + count] == 0u) break;
    }    
    
    // Copy field to result
    for (n = 0; n < count; n++)
    {
        result[n] = packet[i+n];
    }
    result[n+1] = 0;    // Add null terminator
}

void NMEA_handle_packet(char *packet, char *NMEA_data)
{
    uint8 i, n;
    uint8 error = 0;
    uint8 checksum = 0;
    char packet_checksum[3];
    char calculated_checksum[3];
        
    // Check if appropriate packet is handled
    if (!str_cmp(packet, NMEA_data, 0, 4))
    {
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
            if(str_cmp(calculated_checksum, packet_checksum, 0, 1)) error++;
        }   
        
        // Copy buffer to NMEA packet if no errors found
        if (!error)
        {
            for(i = 0; i < NMEA_MAX_SIZE; i++)
            {
                NMEA_data[i] = packet[i];
            }
        }
    }
}

uint8 str_cmp(char *str1, char *str2, uint8 start, uint8 stop)
{
    uint8 i;
    for(i = start; i <= stop; i++)
    {
        if (str1[i] != str2[i]) return 1;
    }
    return 0;
}

void str_append(char *base, char *add)
{
    uint8 i; 
    uint8 size = strlen(base);
    for(i = 0; i <= strlen(add); i++)
    {
        base[i + size] = add[i];
    }
    i++;
    base[i + size] = 0;
}

/* [] END OF FILE */
