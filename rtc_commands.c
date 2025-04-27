#include "rtc_commands.h"
#include "cy_rtc.h"  // Ensure RTC driver is included

#include <stdio.h>
#include <string.h>


void read_rtc_time(char *buffer, size_t buffer_size) {
    cy_stc_rtc_config_t current_time;
    Cy_RTC_GetDateAndTime(&current_time);
    Cy_SysLib_Delay(10);
    // Format the time into a string
    snprintf(buffer, buffer_size, "RTC Time: %02lu:%02lu:%02lu\r\n",
             current_time.hour, current_time.min, current_time.sec);
}
void set_rtc_time(const char *command_buffer, char *response_buffer, size_t response_size) {
    unsigned int hour, min, sec, day, month, year;

    // Print the command received for debugging
    printf("Command Buffer: %s\r\n", command_buffer);

    if (sscanf(command_buffer, "set %u:%u:%u %u/%u/%u", &hour, &min, &sec, &day, &month, &year) == 6) {
        printf("Parsed Values - Hour: %u, Min: %u, Sec: %u, Day: %u, Month: %u, Year: %u\r\n", hour, min, sec, day, month, year);

        if (hour < 24 && min < 60 && sec < 60 && day > 0 && month > 0 && month <= 12) {
            cy_stc_rtc_config_t new_time = {0};
            new_time.hour = hour;
            new_time.min = min;
            new_time.sec = sec;
            new_time.date = day;
            new_time.month = month;
            new_time.year = year;


            printf("Attempting to set RTC...\r\n");


            cy_en_rtc_status_t status = Cy_RTC_SetDateAndTimeDirect(new_time.sec, new_time.min, new_time.hour, new_time.date, new_time.month, new_time.year);
            Cy_SysLib_Delay(100);

            printf("RTC Set Status: %d\r\n", status);


            if (status == CY_RTC_SUCCESS) {
                printf("RTC Time Set: %02u:%02u:%02u on %02u/%02u/%04u\r\n", hour, min, sec, day, month, year);
            } else {
                printf("Error: Failed to set RTC time\r\n");
            }
        } else {
            printf("Error: Invalid time or date values\r\n");
        }
    } else {
        printf("Error: Command format is 'set HH:MM:SS DD/MM/YYYY'\r\n");
    }
}
