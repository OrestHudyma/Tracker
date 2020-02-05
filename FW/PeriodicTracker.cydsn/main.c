/* ========================================
 *
 * Copyright Lion Security, 2020
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Lion Security.
 *
 * ========================================
*/
#include "project.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h> 
#include <stdbool.h>

asm (".global _printf_float");  // Enable using float with printf

#define POWER_ON                0
#define POWER_OFF               1
#define ASCII_SUB               26      // ASCII SUBSTITUTE symbol (CTRL + Z)
#define SEC_DELAY_MS            1000
#define MS_DELAY_US             1000
#define MIN_TO_SEC_RATIO        60
#define KN_TO_KM_RATIO          1.852   // Knots to kilometers ratio
#define DECIMAL_BASIS           10
#define WCO_FREQ                32768   // Hz
#define PHONE_NUM_MAX_LENGHT    15
#define PASSWORD_SIZE           4

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
#define NMEA_GPGGA_SATELLITES       7
#define NMEA_GPGGA_HDOP             8
#define NMEA_GPGGA_ALTITUDE         9

#define NMEA_GPRMC_VALID            'A'
#define NMEA_GPRMC_INVALID          'V'

// GSM definitions
#define GSM_BUFFER_SIZE            180
#define GSM_PWRKEY_DELAY_MS        1200             // 1 sec as per SIM900D DS
#define GSM_POWERUP_DELAY_MS       5000             // 2.2 sec as per SIM900D DS
#define AT_OK                      "\r\nOK\r\n"
#define AT_ERROR                   "\r\nERROR\r\n"
#define AT_CMGL_CONST_PATTERN      "+CMGL: "
#define AT_SMS_CMD_PATTERN         "\"\r\nCMD "
#define AT_TIMEOUT_MS              3000
#define AT_INTERCOMM_DELAY_MS      500

#define EXT_CMD_MAX_SIZE           50
#define CMD_ELEMENT_MAX_SIZE       10
#define CMD_DELIMITER              ' '

// Default user settings
#define GSM_MASTER_PHONE_NUM           "+380633584255"
#define PASSWORD                       {'0', '0', '0', '0'}
#define GSM_ATTEMPTS                   5
#define GSM_NET_TIMEOUT_SEC            500
#define GPS_FIX_TIMEOUT_SEC            500
#define GPS_FIX_IMPROVE_DELAY_MS       30000

// System settings
#define OVERALL_TIMEOUT     150     // seconds
#define POWER_STAB_DELAY    2000    // miliseconds
#define GSM_WAIT_MS         60000   // miliseconds
#define WRONG_PASS_MAX      5

// External commands
#define EXTCMD_HELP                 0
#define EXTCMD_GET_SETTINGS         1
#define EXTCMD_RESET                2
#define EXTCMD_HARD_RESET           3

#if (AT_INTERCOMM_DELAY_MS > SEC_DELAY_MS)
    #error AT_INTERCOMM_DELAY_MS is higher than SEC_DELAY_MS
#endif

struct settings {
    char GSM_master_phone_num[PHONE_NUM_MAX_LENGHT];
    char password[PASSWORD_SIZE];
    uint8 GSM_attempts;
    uint16 GSM_net_timeout_sec;
    uint16 GPS_fix_timeout_sec;
    uint32 GPS_fix_improve_delay_ms;
} user_settings;

struct cmd  {
    uint8 cmd;
    uint8 password[PASSWORD_SIZE];
    uint32 parameter;
} ext_cmd_parsed;
char ext_cmd_str[GSM_BUFFER_SIZE];
const uint8 default_password[PASSWORD_SIZE] = PASSWORD;
uint16 wrong_pass_count = 0;

/* EEPROM storage in work flash, this is defined in Em_EEPROM.c*/
#if defined (__ICCARM__)
#pragma data_alignment = CY_FLASH_SIZEOF_ROW
const uint8_t Em_EEPROM_US_Storage[Em_EEPROM_US_PHYSICAL_SIZE] = {0u};
const uint8_t Em_EEPROM_US_Empty[Em_EEPROM_US_PHYSICAL_SIZE] = {0u};
#else
const uint8_t Em_EEPROM_US_Storage[Em_EEPROM_US_PHYSICAL_SIZE] __ALIGNED(CY_FLASH_SIZEOF_ROW) = {0u};
const uint8_t Em_EEPROM_US_Empty[Em_EEPROM_US_PHYSICAL_SIZE] __ALIGNED(CY_FLASH_SIZEOF_ROW) = {0u};
#endif /* defined (__ICCARM__) */

struct location_data {
    char GPS_lat[NMEA_MAX_SIZE];
    char GPS_lon[NMEA_MAX_SIZE];
    char GPS_spd[NMEA_MAX_SIZE];
    char GPS_EHS[NMEA_MAX_SIZE];                   // Equator hemisphere
    char GPS_MHS[NMEA_MAX_SIZE];                   // Prime meridian hemisphere
    char GPS_HDOP[NMEA_MAX_SIZE];                  // Horizontal dilution of precision
    char GPS_Alt[NMEA_MAX_SIZE];                   // Altitude, Meters, above mean sea level
};

char GPS_validity[1];
char GPS_satellites_count[1];
char event_log[GSM_BUFFER_SIZE] = "";
bool hard_reset_detected;

char NMEA_buffer[NMEA_MAX_SIZE];
char NMEA_GPRMC[NMEA_MAX_SIZE] = "GNRMC";
char NMEA_GPGGA[NMEA_MAX_SIZE] = "GNGGA";
uint8 NMEA_pointer;

char GSM_buffer[GSM_BUFFER_SIZE];
char GSM_command[GSM_BUFFER_SIZE];
char GSM_responce[GSM_BUFFER_SIZE];
uint16 GSM_pointer = 0;
bool GSM_Rx_overflow_flag = false;

void NMEA_handle_packet();
void NMEA_GetField(char *packet, uint8 field, char *result);
void NMEA_native_to_formatted(struct location_data *native, struct location_data *formatted);

struct location_data loc_native = {"","","","","","",""};
struct location_data loc_formatted = {"","","","","","",""};

void wake_up_handler();
cystatus ATCommand(char* command, uint32 timeout, char* responce);
cystatus SendSMS(char* SMS_text);
cystatus GSM_Init();
cystatus GSM_Power(uint8 toggle);
cystatus GSM_get_ext_cmd(char ext_cmd_str[EXT_CMD_MAX_SIZE]);
cystatus GSM_parse_cmd(const char ext_cmd_str[EXT_CMD_MAX_SIZE], struct cmd *ext_cmd_parsed);
cystatus check_number(const char str_number[CMD_ELEMENT_MAX_SIZE]);
cystatus execute_cmd(struct cmd ext_cmd_parsed);

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
        NMEA_handle_packet(&NMEA_buffer, &NMEA_GPGGA);
        Pin_LED_status_Write(0);
        break;
        
        default:
        NMEA_pointer++;
        break;
    }
}

CY_ISR(GSM_receive)
{      
    // Limit pointer value not to exceed GSM_BUFFER_SIZE (-1 byte for null-termination)
    if (GSM_pointer >= (GSM_BUFFER_SIZE - 1))
    {
        GSM_pointer--;
        GSM_Rx_overflow_flag = true;
    }
    if (GSM_pointer >= (GSM_BUFFER_SIZE - 1)) 
    {
        GSM_pointer = 0;
    }
    GSM_buffer[GSM_pointer] = UART_GSM_UartGetChar();
    GSM_buffer[GSM_pointer + 1] = 0;
    GSM_pointer++;
    UART_GSM_ClearRxInterruptSource(UART_GSM_INTR_RX_NOT_EMPTY);
}

int main(void)
{
    uint32 t;
    char* pointer;
    uint8 size;
    char tmp[10];
    cystatus cmd_read_status;
    cystatus cmd_parse_status;
    
    CyDelay(POWER_STAB_DELAY);
    for(t = 0; t < 20; t++)
    {
        Pin_LED_status_Write(1);
        CyDelay(100);
        Pin_LED_status_Write(0);
        CyDelay(100);
    }
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    if(!memcmp(Em_EEPROM_US_Storage, Em_EEPROM_US_Empty, Em_EEPROM_US_PHYSICAL_SIZE))
    {
        strlcat(event_log, "Hard reset detected\r", GSM_BUFFER_SIZE);
        hard_reset_detected = true;
    }
    else
    {
        strlcat(event_log, "Reset detected\r", GSM_BUFFER_SIZE);
        hard_reset_detected = false;
    }
    
    Em_EEPROM_US_Init((uint32_t)Em_EEPROM_US_Storage);    
    if(hard_reset_detected)
    {
        // Load default user settings
        user_settings.GSM_master_phone_num[0] = 0;
        strlcat(user_settings.GSM_master_phone_num, GSM_MASTER_PHONE_NUM, PHONE_NUM_MAX_LENGHT);
        memcpy(user_settings.password, default_password, PASSWORD_SIZE);
        user_settings.GSM_attempts = GSM_ATTEMPTS;
        user_settings.GSM_net_timeout_sec = GSM_NET_TIMEOUT_SEC;
        user_settings.GPS_fix_timeout_sec = GPS_FIX_TIMEOUT_SEC;
        user_settings.GPS_fix_improve_delay_ms = GPS_FIX_IMPROVE_DELAY_MS;
        
        Em_EEPROM_US_Write(0, &user_settings, sizeof(user_settings));
    }
    else
    {
        // Load user settings from flash
        Em_EEPROM_US_Read(0, &user_settings, sizeof(user_settings));
    }
        
    isr_GPS_received_StartEx(GPS_receive);
    isr_GSM_received_StartEx(GSM_receive);
    
    UART_GPS_Start();
    UART_GSM_Start();
    
    for(;;)
    {        
        /*********************** GPS *******************************/
        
        // Apply power to GPS
        Pin_GPS_power_Write(POWER_ON);  
                
        // Wait for GPS fix
        for(t = 0; t < user_settings.GPS_fix_timeout_sec; t++)
        {
            CyDelay(SEC_DELAY_MS);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_VALIDITY, GPS_validity);
            if (GPS_validity[0] == NMEA_GPRMC_VALID) 
            {                
                CyDelay(user_settings.GPS_fix_improve_delay_ms);
                break;
            }
        }
                
        // Turn off GPS
        Pin_GPS_power_Write(POWER_OFF);                    
        CyDelay(SEC_DELAY_MS);
        Pin_LED_status_Write(0);
        
        NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_VALIDITY, GPS_validity);
        NMEA_GetField(NMEA_GPGGA, NMEA_GPGGA_SATELLITES, GPS_satellites_count);
        
        if (GPS_validity[0] == NMEA_GPRMC_VALID)
        {
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_LATITUDE, loc_native.GPS_lat);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_LONGITUDE, loc_native.GPS_lon);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_EHS, loc_native.GPS_EHS);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_MHS, loc_native.GPS_MHS);
            NMEA_GetField(NMEA_GPRMC, NMEA_GPRMC_SPEED, loc_native.GPS_spd); 
            NMEA_GetField(NMEA_GPGGA, NMEA_GPGGA_ALTITUDE, loc_native.GPS_Alt);
            NMEA_GetField(NMEA_GPGGA, NMEA_GPGGA_HDOP, loc_native.GPS_HDOP);
               
            NMEA_native_to_formatted(&loc_native, &loc_formatted);
        }

        
        /*********************** GSM *******************************/
        debug_point:
        if (GSM_Init() == CYRET_SUCCESS)
        {            
            GSM_command[0] = 0;
            if (GPS_validity[0] == NMEA_GPRMC_INVALID)
            {
                strlcat(GSM_command, "No GPS data. Using last valid.\r", GSM_BUFFER_SIZE);
            }
            strlcat(GSM_command, event_log, GSM_BUFFER_SIZE);
            event_log[0] = 0; // Clear event log
      
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
            
            strlcat(GSM_command, "\rAlt:", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_native.GPS_Alt, GSM_BUFFER_SIZE);
            
            strlcat(GSM_command, "\rSat:", GSM_BUFFER_SIZE);
            strlcat(GSM_command, GPS_satellites_count, GSM_BUFFER_SIZE);
            
            strlcat(GSM_command, "\rHDOP:", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_native.GPS_HDOP, GSM_BUFFER_SIZE);
            
            strlcat(GSM_command, "\rTTF:", GSM_BUFFER_SIZE);
            itoa(t, tmp, 10);
            strlcat(GSM_command, tmp, GSM_BUFFER_SIZE);
            strlcat(GSM_command, "s", GSM_BUFFER_SIZE);
            
            strlcat(GSM_command, "\rBatt", GSM_BUFFER_SIZE);
            ATCommand("AT+CBC\r", AT_TIMEOUT_MS, GSM_responce);            
            pointer = strstr(GSM_responce, ": ");
            size = strstr(pointer, "\r") - pointer +1;            
            strlcat(GSM_command, pointer, strlen(GSM_command) + size);
            strlcat(GSM_command, "mV", GSM_BUFFER_SIZE);
            
            strlcat(GSM_command, "\rhttp://maps.google.com/?q=", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_formatted.GPS_lat, GSM_BUFFER_SIZE);
            strlcat(GSM_command, ",", GSM_BUFFER_SIZE);
            strlcat(GSM_command, loc_formatted.GPS_lon, GSM_BUFFER_SIZE);
            
            SendSMS(GSM_command);
        }
        // Wait for GSM send/receive
        CyDelay(GSM_WAIT_MS);
                
        cmd_read_status = CYRET_SUCCESS;
        while(cmd_read_status == CYRET_SUCCESS)
        {
            cmd_read_status = GSM_get_ext_cmd(ext_cmd_str);
            if(cmd_read_status == CYRET_SUCCESS)
            {
                cmd_parse_status = GSM_parse_cmd(ext_cmd_str, &ext_cmd_parsed);
                if(cmd_parse_status == CYRET_SUCCESS)
                {
                    execute_cmd(ext_cmd_parsed);
                }
            }
        }
        // Remove all SMS
        GSM_command[0] = 0; // Clear GSM_command
        strlcat(GSM_command, "AT+CMGD=1,4\r", GSM_BUFFER_SIZE);
        ATCommand(GSM_command, AT_TIMEOUT_MS, GSM_responce);
        
                
        // Turn off GSM
        GSM_Power(false);
                        
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

cystatus execute_cmd(struct cmd ext_cmd)
{
    char SMS_buffer[GSM_BUFFER_SIZE] = {0};
    char number_str[EXT_CMD_MAX_SIZE] = {0};
    char UniqueId[9] = {0};
    
    // Check password
    if(memcmp(ext_cmd.password, user_settings.password, PASSWORD_SIZE))
    {        
        wrong_pass_count++;
        if(wrong_pass_count <= WRONG_PASS_MAX)
        {
            itoa(WRONG_PASS_MAX - wrong_pass_count, number_str, DECIMAL_BASIS);
            SMS_buffer[0] = 0;
            strlcat(SMS_buffer, "Wrong password. ", GSM_BUFFER_SIZE);
            strlcat(SMS_buffer, number_str, GSM_BUFFER_SIZE);
            strlcat(SMS_buffer, " attempts left.", GSM_BUFFER_SIZE);
            SendSMS(SMS_buffer);
        }
        return CYRET_LOCKED;
    }
    else
    {
        switch(ext_cmd.cmd)
        {
            case EXTCMD_GET_SETTINGS:
            {
                SMS_buffer[0] = 0;
                
                strlcat(SMS_buffer, "MPN:", GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, user_settings.GSM_master_phone_num, GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, "\r", GSM_BUFFER_SIZE);
                
                itoa(user_settings.GSM_attempts, number_str, DECIMAL_BASIS);
                strlcat(SMS_buffer, "GSM_attempts:", GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, number_str, GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, "\r", GSM_BUFFER_SIZE);
                
                itoa(user_settings.GSM_net_timeout_sec, number_str, DECIMAL_BASIS);
                strlcat(SMS_buffer, "GSM_net_timeout_sec:", GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, number_str, GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, "\r", GSM_BUFFER_SIZE);
                
                itoa(user_settings.GPS_fix_timeout_sec, number_str, DECIMAL_BASIS);
                strlcat(SMS_buffer, "GPS_fix_timeout_sec:", GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, number_str, GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, "\r", GSM_BUFFER_SIZE);
                
                itoa(user_settings.GPS_fix_improve_delay_ms, number_str, DECIMAL_BASIS);
                strlcat(SMS_buffer, "GPS_fix_improve_delay_ms:", GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, number_str, GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, "\r", GSM_BUFFER_SIZE);
                
                //CyGetUniqueId((uint32*)UniqueId);
                strlcat(SMS_buffer, "Device ID:", GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, UniqueId, GSM_BUFFER_SIZE);
                strlcat(SMS_buffer, "\r", GSM_BUFFER_SIZE);                
                SendSMS(SMS_buffer);                
                break;
            }
            case EXTCMD_RESET:
            {
                SendSMS("EXTCMD_RESET received. Executing soft reset...");
                CySoftwareReset();                
                break;
            }
            case EXTCMD_HARD_RESET:
            {
                SendSMS("EXTCMD_HARD_RESET received. All settings will be set to defaults. Executing hard reset...");
                // TODO 
                CySoftwareReset();                
                break;
            }
            case EXTCMD_HELP:
            {
                SendSMS("TODO: help message");
                break;
            }
            default:
                SendSMS("Unsupported command");
                break;
            
        }
    }
    return CYRET_SUCCESS;
}

cystatus GSM_parse_cmd(const char ext_cmd_str[EXT_CMD_MAX_SIZE], struct cmd *ext_cmd_parsed)
{
    char cmd[EXT_CMD_MAX_SIZE];
    char parameter[EXT_CMD_MAX_SIZE];
    char password[EXT_CMD_MAX_SIZE];
    char* next_element;
    uint8 element_size;
    uint8 tmp = 100;
    
    // Read command number
    element_size = strchr(ext_cmd_str, CMD_DELIMITER) + 1 - ext_cmd_str;
    strlcpy(cmd, ext_cmd_str, element_size);
    
    // Read parameter
    next_element = strchr(ext_cmd_str, CMD_DELIMITER) + 1;
    element_size = (uint8) (strchr(next_element + 1, CMD_DELIMITER) - next_element);
    strlcpy(parameter, next_element, element_size + 1); //Plus 1 for null-terminator
    
    // Read password
    next_element = strchr(next_element, CMD_DELIMITER) + 1;
    element_size = (uint8) (strchr(next_element + 1, CMD_DELIMITER) - next_element);
    strlcpy(password, next_element, PASSWORD_SIZE + 1);  //Plus 1 for null-terminator
       
    // Validate parsed data
    if(check_number(cmd) == CYRET_BAD_DATA) return CYRET_BAD_DATA;
    if(check_number(parameter) == CYRET_BAD_DATA) return CYRET_BAD_DATA;
    if(check_number(password) == CYRET_BAD_DATA) return CYRET_BAD_DATA;
    
    // Format and copy data to struct that contains parsed data
    ext_cmd_parsed->cmd = atoi(cmd);
    ext_cmd_parsed->parameter = atoi(parameter);
    memcpy(ext_cmd_parsed->password, password, PASSWORD_SIZE);
        
    return CYRET_SUCCESS;
}

cystatus GSM_get_ext_cmd(char ext_cmd_str[EXT_CMD_MAX_SIZE])
{
    cystatus status = CYRET_STARTED;
    char index[2] = {0};
    char* SMS_index_pointer = NULL;
    char* CMD_start_pointer = NULL;
    char GSM_responce[GSM_BUFFER_SIZE] = {0};
    char GSM_command[GSM_BUFFER_SIZE]; 
    
    while(CMD_start_pointer == NULL)
    {    
        // Get all SMS info
        status = ATCommand("AT+CMGL=\"ALL\",1\r", AT_TIMEOUT_MS, GSM_responce);
                
        // Find next SMS index
        SMS_index_pointer = strstr(GSM_responce, AT_CMGL_CONST_PATTERN);
        if(SMS_index_pointer == NULL) return CYRET_EMPTY;
        SMS_index_pointer+= sizeof(AT_CMGL_CONST_PATTERN) - 1;
        index[0] = *SMS_index_pointer;            
        
        // Read SMS and extract command
        GSM_command[0] = 0; // Clear GSM_command
        strlcat(GSM_command, "AT+CMGR=", GSM_BUFFER_SIZE);
        strlcat(GSM_command, index, GSM_BUFFER_SIZE);
        strlcat(GSM_command, ",0\r", GSM_BUFFER_SIZE);
        ATCommand(GSM_command, AT_TIMEOUT_MS, GSM_responce);
        CMD_start_pointer = strstr(GSM_responce, AT_SMS_CMD_PATTERN);
        if(CMD_start_pointer != NULL)
        {
            CMD_start_pointer += sizeof(AT_SMS_CMD_PATTERN) - 1;
            strlcat(ext_cmd_str, CMD_start_pointer, EXT_CMD_MAX_SIZE);
            *strstr(ext_cmd_str, AT_OK) = 0;    // Cut of AT OK with null-terminator
        }

        // Remove current SMS
        GSM_command[0] = 0; // Clear GSM_command
        strlcat(GSM_command, "AT+CMGD=", GSM_BUFFER_SIZE);
        strlcat(GSM_command, index, GSM_BUFFER_SIZE);
        strlcat(GSM_command, "\r", GSM_BUFFER_SIZE);
        status = ATCommand(GSM_command, AT_TIMEOUT_MS, GSM_responce);
    }
    return status;
}

cystatus GSM_Init()
{
    cystatus status;
    uint32 timeout = user_settings.GSM_net_timeout_sec;
    char GSM_responce[GSM_BUFFER_SIZE];
    
    status = GSM_Power(true);
    
    if(status == CYRET_SUCCESS)
    {
        // Check if SIM card PIN is disabled
        status = ATCommand("AT+CPIN?\r", AT_TIMEOUT_MS, GSM_responce);
        if(strstr(GSM_responce, "+CPIN: READY\r\n\r\nOK") == NULL) status++;
    }
    
    if(status == CYRET_SUCCESS)
    {
        // Check if SIM card PIN is disabled
        status = ATCommand("AT+CSDT?\r", AT_TIMEOUT_MS, GSM_responce);
        if(strstr(GSM_responce, "+CSDT: 0\r\n\r\nOK") == NULL) status++;
    }
    
    if(status == CYRET_SUCCESS)
    {
        // Check if registered in home network
        GSM_responce[0] =0;
        while((timeout > 0) & (strstr(GSM_responce, "1\r\n\r\nOK") == NULL))
        {
            status += ATCommand("AT+CREG?\r", AT_TIMEOUT_MS, GSM_responce);
            if(status != CYRET_SUCCESS) break;
            timeout--;
            CyDelay(SEC_DELAY_MS - AT_INTERCOMM_DELAY_MS);
        }
        if ((status == CYRET_SUCCESS) & (timeout == 0)) return CYRET_TIMEOUT;
    }
    
    if(status == CYRET_SUCCESS)
    {
        // Check if module is ready
        status = ATCommand("AT+CPAS\r", AT_TIMEOUT_MS, GSM_responce);
        if(strstr(GSM_responce, "+CPAS: 0\r\n\r\nOK") == NULL) status++;
        
        // Check operator for debug
        status += ATCommand("AT+COPS?\r", AT_TIMEOUT_MS, GSM_responce);
    }
        
    // Configure GSM module
    if(status == CYRET_SUCCESS)
    {        
        // Switch to SMS text mode
        status = ATCommand("AT+CMGF=1\r", AT_TIMEOUT_MS, GSM_responce);
        // Switch to SMS GSM coding
        status += ATCommand("AT+CSCS=\"GSM\"\r", AT_TIMEOUT_MS, GSM_responce);
    }
    
    return status;
}

cystatus ATCommand(char* command, uint32 timeout, char* response)
{   
    GSM_pointer = 0;
    GSM_Rx_overflow_flag = false;
    GSM_buffer[GSM_pointer] = 0;
    response[0] = 0;
    UART_GSM_UartPutString(command);    
    CyDelay(AT_INTERCOMM_DELAY_MS);
    if(timeout == 0) return CYRET_SUCCESS;
    while((timeout > 0) & (strstr(GSM_buffer, AT_OK) == NULL))
    {
        if (strstr(GSM_buffer, AT_ERROR) != NULL) return CYRET_UNKNOWN;
        CyDelayUs(MS_DELAY_US);
        timeout--;
    }
    if((timeout == 0) & !GSM_Rx_overflow_flag) return CYRET_TIMEOUT;
    else 
    {
        strlcpy(response, GSM_buffer, GSM_BUFFER_SIZE);
        if(GSM_Rx_overflow_flag) return CYRET_MEMORY;
        else return CYRET_SUCCESS;
    }
}

cystatus GSM_Power(uint8 toggle)
{    
    uint8 attempts = user_settings.GSM_attempts;
    char GSM_responce[GSM_BUFFER_SIZE];
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
    while((ATCommand("AT\r", AT_TIMEOUT_MS, GSM_responce) != CYRET_SUCCESS) ^ !toggle);
    if(attempts) return CYRET_SUCCESS;
    else return CYRET_TIMEOUT;
}

cystatus SendSMS(char* SMS_text)
{
    char command[GSM_BUFFER_SIZE] = "";
    char responce[GSM_BUFFER_SIZE];
    uint8 error = 0;
    
    strlcat(command, "AT+CMGS=\"", GSM_BUFFER_SIZE);
    strlcat(command, user_settings.GSM_master_phone_num, GSM_BUFFER_SIZE);
    strlcat(command, "\"\r", GSM_BUFFER_SIZE);
    error += ATCommand(command, 0, responce);
    command[0] = 0;     // Free GSM command buffer
    strlcat(command, SMS_text, GSM_BUFFER_SIZE);
    error += ATCommand(command, 0, responce);    
    command[0] = ASCII_SUB;
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
    double lat;
    double lon;
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
    
    sprintf(formatted->GPS_lat, "%.8lf", lat);
    sprintf(formatted->GPS_lon, "%.8lf", lon);
    sprintf(formatted->GPS_spd, "%.1lf", spd);
}

cystatus check_number(const char str_number[CMD_ELEMENT_MAX_SIZE])
{
    uint8 i;
    for(i = 0; i < strlen(str_number); i++)
    {
        if(!isdigit(str_number[i])) return CYRET_BAD_DATA;
    }
    return CYRET_SUCCESS;
}

/* [] END OF FILE */
