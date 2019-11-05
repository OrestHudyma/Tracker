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
#include <stdio.h>
#include <math.h> 

asm (".global _printf_float");  // Enable using float with printf

#define POWER_ON            0
#define POWER_OFF           1
#define ASCII_SUB           26      // ASCII SUBSTITUTE symbol (CTRL + Z)
#define SEC_DELAY_MS        1000
#define MS_DELAY_US         1000
#define MIN_TO_SEC_RATIO    60
#define KN_TO_KM_RATIO      1.852   // Knots to kilometers ratio

// NMEA definitions
#define NMEA_MAX_SIZE             82
#define NMEA_START_DELIMITER      '$'
#define NMEA_END_DELIMITER        0x0A
#define NMEA_CHECKSUM_DELIMITER   '*'
#define NMEA_FIELD_DELIMITER      ','
#define NMEA_MSG_NAME_SIZE        4

#define NMEA_GPRMC_LATITUDE         3
#define NMEA_GPRMC_EHS              4
#define NMEA_GPRMC_LONGITUDE        5
#define NMEA_GPRMC_MHS              6
#define NMEA_GPRMC_UTC              1
#define NMEA_GPRMC_SPEED            7
#define NMEA_GPRMC_VALIDITY         2

#define NMEA_GPRMC_VALID            'A'
#define NMEA_GPRMC_INVALID          'V'

// GSM definitions
#define GSM_BUFFER_SIZE            100
#define GSM_PWRKEY_DELAY_MS        1200             // 1 sec as per SIM900D DS
#define GSM_POWERUP_DELAY_MS       5000             // 2.2 sec as per SIM900D DS
#define AT_OK                      "\r\nOK\r\n"
#define AT_ERROR                   "\r\nERROR\r\n"
#define AT_TIMEOUT_MS              2000
#define AT_INTERCOMM_DELAY_MS      500

// Default Settings
#define GSM_MASTER_PHONE_NUM           "+380633584255"
#define GSM_ATTEMPTS                   5
#define GSM_NET_TIMEOUT_SEC            100
#define GPS_FIX_TIMEOUT_SEC            500
#define GPS_FIX_IMPROVE_DELAY_MS       20000

#if (AT_INTERCOMM_DELAY_MS > SEC_DELAY_MS)
    #error AT_INTERCOMM_DELAY_MS is higher than SEC_DELAY_MS
#endif

struct location_data
{
    char GPS_lat[NMEA_MAX_SIZE];
    char GPS_lon[NMEA_MAX_SIZE];
    char GPS_spd[NMEA_MAX_SIZE];
    char GPS_EHS[NMEA_MAX_SIZE];                   // Equator hemisphere
    char GPS_MHS[NMEA_MAX_SIZE];                   // Prime meridian hemisphere
};
char GPS_validity[1];

char NMEA_buffer[NMEA_MAX_SIZE];
char NMEA_GPRMC[NMEA_MAX_SIZE] = "GNRMC";
uint8 NMEA_pointer;

char GSM_buffer[GSM_BUFFER_SIZE];
char GSM_command[GSM_BUFFER_SIZE];
uint8 GSM_pointer = 0;

void NMEA_handle_packet();
void NMEA_GetField(char *packet, uint8 field, char *result);
void NMEA_native_to_formatted(struct location_data *native, struct location_data *formatted);

struct location_data loc_native = {"","","","",""};
struct location_data loc_formatted = {"","","","",""};

void wake_up_handler();
cystatus ATCommand(char* command, uint32 timeout, char* responce);
cystatus SendSMS(char* SMS_text);
cystatus GSM_Init();

CY_ISR(GPS_receive)
{    
    if (NMEA_pointer >= NMEA_MAX_SIZE) NMEA_pointer = 0;
    NMEA_buffer[NMEA_pointer] = UART_GPS_GetChar();
    NMEA_buffer[NMEA_pointer + 1] = 0;    
    switch(NMEA_buffer[NMEA_pointer])
    {
        case NMEA_START_DELIMITER:
        NMEA_pointer = 0;
        Pin_LED_status_Write(1);
        break;
        
        case NMEA_END_DELIMITER:
        NMEA_handle_packet(&NMEA_buffer, &NMEA_GPRMC);
        Pin_LED_status_Write(0);
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
    UART_GSM_ClearRxInterruptSource(UART_GSM_INTR_RX_NOT_EMPTY);    
}

int main(void)
{
    uint32 t;
            
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
            CyDelay(SEC_DELAY_MS);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_VALIDITY, GPS_validity);
            if (GPS_validity[0] == NMEA_GPRMC_VALID) 
            {                
                CyDelay(GPS_FIX_IMPROVE_DELAY_MS);
                break;
            }
        }
                
        // Turn off GPS
        Pin_GPS_power_Write(POWER_OFF);
        
        NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_VALIDITY, GPS_validity);
        if (GPS_validity[0] == NMEA_GPRMC_VALID)
        {
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_LATITUDE, loc_native.GPS_lat);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_LONGITUDE, loc_native.GPS_lon);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_EHS, loc_native.GPS_EHS);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_MHS, loc_native.GPS_MHS);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_SPEED, loc_native.GPS_spd); 
               
            NMEA_native_to_formatted(&loc_native, &loc_formatted);
        }
        CyDelay(SEC_DELAY_MS);
        Pin_LED_status_Write(0);
        
        /*********************** GSM *******************************/
        
        if (GSM_Init() == CYRET_SUCCESS)
        {
            GSM_command[0] = 0;
            if (GPS_validity[0] == NMEA_GPRMC_INVALID)
            {
                strlcat(GSM_command, "No GPS data. Using last valid.\r", GSM_BUFFER_SIZE);
            }
            
            strlcat(GSM_command, "Lat:", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_native.GPS_lat, GSM_BUFFER_SIZE);
            strlcat(GSM_command, ".", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_native.GPS_EHS, GSM_BUFFER_SIZE);
            
            strlcat(GSM_command, "\rLon:", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_native.GPS_lon, GSM_BUFFER_SIZE);
            strlcat(GSM_command, ".", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_native.GPS_MHS, GSM_BUFFER_SIZE);
            
            strlcat(GSM_command, "\rSpd:", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_formatted.GPS_spd, GSM_BUFFER_SIZE);
            
            strlcat(GSM_command, "\rValidity:", GSM_BUFFER_SIZE);
            strlcat(GSM_command, GPS_validity, GSM_BUFFER_SIZE);
            
            strlcat(GSM_command, "\rhttp://maps.google.com/?q=", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_formatted.GPS_lat, GSM_BUFFER_SIZE);
            strlcat(GSM_command, ",", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_formatted.GPS_lon, GSM_BUFFER_SIZE);
            
            SendSMS(GSM_command);
        }
        CyDelay(10000);
        // Turn off GSM
        Pin_GSM_PWRKEY_Write(0);
        CyDelay(GSM_PWRKEY_DELAY_MS);
        Pin_GSM_PWRKEY_Write(1);
                
        UART_GPS_Sleep();
        UART_GSM_Sleep();
        CySysPmDeepSleep();
        UART_GPS_Wakeup();
        UART_GSM_Wakeup();
    }
}

void wake_up_handler()
{
}

cystatus GSM_Init()
{
    uint8 error = 0;
    uint32 timeout = GSM_NET_TIMEOUT_SEC;
    char GSM_responce[GSM_BUFFER_SIZE];
    
    // Turn on GSM
    Pin_GSM_PWRKEY_Write(0);
    CyDelay(GSM_PWRKEY_DELAY_MS);
    Pin_GSM_PWRKEY_Write(1);
    CyDelay(GSM_POWERUP_DELAY_MS);
        
    ATCommand("AT\r", AT_TIMEOUT_MS, GSM_responce);   // Set comm speed
    error += ATCommand("AT\r", AT_TIMEOUT_MS, GSM_responce);   // Check comm 
    
    if(error == CYRET_SUCCESS)
    {
        // Check if SIM card PIN is disabled
        error = ATCommand("AT+CPIN?\r", AT_TIMEOUT_MS, GSM_responce);
        if(strstr(GSM_responce, "+CPIN: READY\r\n\r\nOK") == NULL) error++;
    }
    
    if(error == CYRET_SUCCESS)
    {
        // Check if SIM card PIN is disabled
        error = ATCommand("AT+CSDT?\r", AT_TIMEOUT_MS, GSM_responce);
        if(strstr(GSM_responce, "+CSDT: 0\r\n\r\nOK") == NULL) error++;
    }
    
    if(error == CYRET_SUCCESS)
    {
        // Check if registered in home network
        GSM_responce[0] =0;
        while((timeout > 0) & (strstr(GSM_responce, "1\r\n\r\nOK") == NULL))
        {
            error += ATCommand("AT+CREG?\r", AT_TIMEOUT_MS, GSM_responce);
            if(error != CYRET_SUCCESS) break;
            timeout--;
            CyDelay(SEC_DELAY_MS - AT_INTERCOMM_DELAY_MS);
        }
        if ((error == CYRET_SUCCESS) & (timeout == 0)) return CYRET_TIMEOUT;
    }
    
    if(error == CYRET_SUCCESS)
    {
        // Check if module is ready
        error = ATCommand("AT+CPAS\r", AT_TIMEOUT_MS, GSM_responce);
        if(strstr(GSM_responce, "+CPAS: 0\r\n\r\nOK") == NULL) error++;
    }
    
    // Check operator for debug
    error += ATCommand("AT+COPS?\r", AT_TIMEOUT_MS, GSM_responce);
    
    // Configure GSM module
    if(error == CYRET_SUCCESS)
    {        
        // Turn on SMS text mode
        error = ATCommand("AT+CMGF=1\r", AT_TIMEOUT_MS, GSM_responce);
    }
    
    if(error == CYRET_SUCCESS) return CYRET_SUCCESS;
    else return CYRET_UNKNOWN;
}

cystatus ATCommand(char* command, uint32 timeout, char* response)
{   
    GSM_pointer = 0;
    GSM_buffer[GSM_pointer] = 0;
    response[0] = 0;
    UART_GSM_UartPutString(command);    
    CyDelay(AT_INTERCOMM_DELAY_MS);
    while((timeout > 0) & (strstr(GSM_buffer, AT_OK) == NULL))
    {
        if (strstr(GSM_buffer, AT_ERROR) != NULL) return CYRET_UNKNOWN;
        CyDelayUs(MS_DELAY_US);
        timeout--;
    }
    if(timeout == 0) return CYRET_TIMEOUT;
    else 
    {
        strlcpy(response, GSM_buffer, GSM_BUFFER_SIZE);
        return CYRET_SUCCESS;
    }
}

cystatus GPS_Power(uint8 toggle)
{    
    uint8 attempts = GSM_ATTEMPTS;
    char GSM_responce[GSM_BUFFER_SIZE];
    
    // Turn on GSM
    Pin_GSM_PWRKEY_Write(0);
    CyDelay(GSM_PWRKEY_DELAY_MS);
    Pin_GSM_PWRKEY_Write(1);
    CyDelay(GSM_POWERUP_DELAY_MS);
    
    do
    {
        if(!attempts) break;
        attempts--;
        
        // Turn on GSM
        Pin_GSM_PWRKEY_Write(0);
        CyDelay(GSM_PWRKEY_DELAY_MS);
        Pin_GSM_PWRKEY_Write(1);
        CyDelay(GSM_POWERUP_DELAY_MS);
        ATCommand("AT\r", AT_TIMEOUT_MS, GSM_responce);   // Set comm speed
    }
    while((ATCommand("AT\r", AT_TIMEOUT_MS, GSM_responce) != CYRET_SUCCESS) & !toggle);
    if(attempts) return CYRET_SUCCESS;
    else return CYRET_TIMEOUT;
}

cystatus SendSMS(char* SMS_text)
{
    char command[GSM_BUFFER_SIZE] = "";
    char responce[GSM_BUFFER_SIZE];
    uint8 error = 0;
    
    strlcat(command, "AT+CMGS=\"", GSM_BUFFER_SIZE);
    strlcat(command, GSM_MASTER_PHONE_NUM, GSM_BUFFER_SIZE);
    strlcat(command, "\"\r", GSM_BUFFER_SIZE);
    error += ATCommand(command, 0, responce);
    command[0] = 0;     // Free GSM command buffer
    strlcat(command, SMS_text, GSM_BUFFER_SIZE);
    error += ATCommand(command, 0, responce);
    command[0] = ASCII_SUB;
    //command[0] = 27;
    command[1] = 0;
    error += ATCommand(command, 0, responce);
    if(error == 0) return CYRET_SUCCESS;
    else return CYRET_UNKNOWN;
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

void NMEA_native_to_formatted(struct location_data *native, struct location_data *formatted)
{        
    float lat;
    float lon;
    float spd;
    
    uint16 intDeg;
    
    lat = atof(native->GPS_lat);
    lon = atof(native->GPS_lon);
    spd = atof(native->GPS_spd);
    
    intDeg = trunc(lat/100);
    lat = intDeg + (lat - 100 * intDeg)/MIN_TO_SEC_RATIO;
    
    intDeg = trunc(lon/100);
    lon = intDeg + (lon - 100 * intDeg)/MIN_TO_SEC_RATIO;
    
    spd *= KN_TO_KM_RATIO;
    
    sprintf(formatted->GPS_lat, "%f", lat);
    sprintf(formatted->GPS_lon, "%f", lon);
    sprintf(formatted->GPS_spd, "%.1f", spd);
}

/* [] END OF FILE */
