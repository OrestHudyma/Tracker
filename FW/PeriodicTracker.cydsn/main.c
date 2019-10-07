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

#define POWER_ON        1
#define POWER_OFF       0
#define CTRL_Z          26

// NMEA definitions
#define NMEA_MAX_SIZE             82
#define NMEA_START_DELIMITER      '$'
#define NMEA_END_DELIMITER        0x0A
#define NMEA_CHECKSUM_DELIMITER   '*'
#define NMEA_FIELD_DELIMITER      ','
#define NMEA_MSG_NAME_SIZE        4

#define NMEA_GPRMC_LATITUDE         3
#define NMEA_GPRMC_LONGITUDE        5
#define NMEA_GPRMC_UTC              1
#define NMEA_GPRMC_SPEED            7
#define NMEA_GPRMC_VALIDITY         2

#define NMEA_GPRMC_VALID            'A'
#define NMEA_GPRMC_INVALID          'V'

// GSM definitions
#define GSM_BUFFER_SIZE            100
#define GSM_PWRKEY_DELAY_MS        1500

// Default Settings
#define GSM_MASTER_PHONE_NUM           "+380633584255"
#define GPS_FIX_TIMEOUT_SEC            5
#define GPS_FIX_IMPROVE_DELAY_MS       5000


char NMEA_buffer[NMEA_MAX_SIZE];
char NMEA_GPRMC[NMEA_MAX_SIZE] = "GPRMC";
uint8 NMEA_pointer;

char GSM_buffer[GSM_BUFFER_SIZE];
char GSM_command[GSM_BUFFER_SIZE];
uint8 GSM_pointer = 0;

void NMEA_handle_packet();
void NMEA_GetField(char *packet, uint8 field, char *result);

char GPS_lat[NMEA_MAX_SIZE];
char GPS_lon[NMEA_MAX_SIZE];
char GPS_spd[NMEA_MAX_SIZE];

void wake_up_handler();
void ATCommand(char* command, uint32 timeout, char* responce);

CY_ISR(GPS_receive)
{    
    if (NMEA_pointer >= NMEA_MAX_SIZE) NMEA_pointer = 0;
    NMEA_buffer[NMEA_pointer] = UART_GPS_GetChar();
    NMEA_buffer[NMEA_pointer + 1] = 0;
    switch(NMEA_buffer[NMEA_pointer])
    {
        case NMEA_START_DELIMITER:
        NMEA_pointer = 0;
        Pin_GPS_RxLED_Write(1);
        break;
        
        case NMEA_END_DELIMITER:
        NMEA_handle_packet(&NMEA_buffer, &NMEA_GPRMC);
        Pin_GPS_RxLED_Write(0);
        break;
        
        default:
        NMEA_pointer++;
        break;
    }
}

CY_ISR(GSM_receive)
{  
    if (GSM_pointer >= GSM_BUFFER_SIZE) GSM_pointer = 0;
    GSM_buffer[GSM_pointer] = UART_GSM_UartGetChar();
    GSM_buffer[GSM_pointer + 1] = 0;
    GSM_pointer++;
}

int main(void)
{
    uint16 t;
    char field_tmp[NMEA_MAX_SIZE];
        
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    isr_GPS_received_StartEx(GPS_receive);
    isr_GSM_received_StartEx(GSM_receive);
    
    UART_GPS_Start();
    UART_GSM_Start();

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
        CyDelay(GPS_FIX_IMPROVE_DELAY_MS);
        
        // Turn off GPS
        Pin_GPS_power_Write(POWER_OFF);
        
        NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_LATITUDE, GPS_lat);
        NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_LONGITUDE, GPS_lon);
        NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_SPEED, GPS_spd);
        

        
        /*********************** GSM *******************************/
        
        // Turn on GSM
        Pin_GSM_PWRKEY_Write(0);
        CyDelay(GSM_PWRKEY_DELAY_MS);
        Pin_GSM_PWRKEY_Write(1);
        
        UART_GSM_UartPutString("AT\r");   // Init communication
        CyDelay(100);
        UART_GSM_UartPutString("AT+CMGF=1\r");    // Select text mode for SMS
        CyDelay(100);
        GSM_command[0] = 0;     // Free GSM command buffer
        strlcat(GSM_command, "AT+CMGS=\"", GSM_BUFFER_SIZE);
        strlcat(GSM_command, GSM_MASTER_PHONE_NUM, GSM_BUFFER_SIZE);
        strlcat(GSM_command, "\"", GSM_BUFFER_SIZE);
        UART_GSM_UartPutString(GSM_command);
        CyDelay(100);
        GSM_command[0] = 0;     // Free GSM command buffer
        strlcat(GSM_command, "Lat:", GSM_BUFFER_SIZE);
        strlcat(GSM_command, GPS_lat, GSM_BUFFER_SIZE);
        strlcat(GSM_command, "Lon:", GSM_BUFFER_SIZE);
        strlcat(GSM_command, GPS_lon, GSM_BUFFER_SIZE);
        strlcat(GSM_command, "Spd:", GSM_BUFFER_SIZE);
        strlcat(GSM_command, GPS_spd, GSM_BUFFER_SIZE);
        strlcat(GSM_command, "Validity:", GSM_BUFFER_SIZE);
        strlcat(GSM_command, field_tmp, GSM_BUFFER_SIZE);
        UART_GSM_UartPutString(GSM_command);
        UART_GSM_UartPutChar(CTRL_Z);
        UART_GSM_UartPutString("\r");
        CyDelay(5000);
        
        // Turn off GSM
        Pin_GSM_PWRKEY_Write(0);
        CyDelay(GSM_PWRKEY_DELAY_MS);
        Pin_GSM_PWRKEY_Write(1);

        
    }
}

void wake_up_handler()
{
}

void ATCommand(char* command, uint32 timeout, char* response)
{
    GSM_pointer = 0;
    UART_GSM_UartPutString(command);
    while((timeout > 0) & (GSM_pointer == 0))
    {
        CyDelayUs(1000);
        timeout--;
    }
    if(timeout == 0) response[0] = 0;
    else 
    {
        CyDelay(200);
        strlcpy(response, GSM_buffer, GSM_BUFFER_SIZE);
    }    
}

void NMEA_GetField(char *packet, uint8 field, char *result)
{
    uint8 i;
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
    strlcpy(result, packet + i, count + 1);  // Add 1 to count for null terminator
}

void NMEA_handle_packet(char *packet, char *NMEA_data)
{
    uint8 i, n;
    uint8 error = 0;
    uint8 checksum = 0;
    char *packet_checksum;
    char calculated_checksum[3];
        
    // Check if appropriate packet is handled
    if (!strncmp(packet, NMEA_data, NMEA_MSG_NAME_SIZE))
    {
        // Check for receive errors
        for(i = 0; i < NMEA_MAX_SIZE; i++)
        {
            if ((packet[i] < 32) & (packet[i] != 0x0D) & (packet[i] != NMEA_END_DELIMITER)) 
            {
                error++;
                break;
            }
            if (packet[i] != NMEA_END_DELIMITER) break;
        }
        
        // Validate checksum and cut packet if no receive errors
        if (!error)
        {
            // Find checksum field
            packet_checksum = memchr(packet, NMEA_CHECKSUM_DELIMITER, NMEA_MAX_SIZE) + 1;
            i = (uint8)(packet_checksum - packet);
            // Cut string to NMEA_CHECKSUM_DELIMITER
            packet[i-1] = 0;
            
            // Calculate checksum and compare
            for (n = 0; n < i; n++) checksum ^= packet[n];
            itoa(checksum, calculated_checksum, 16);
            if(!strcmp(calculated_checksum, packet_checksum)) error++;
        }   
        
        // Copy buffer to NMEA packet if no errors found
        if (!error) strlcpy(NMEA_data, packet, NMEA_MAX_SIZE);
    }
}

/* [] END OF FILE */
