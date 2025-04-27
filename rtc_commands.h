#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>
#include <stddef.h>

// Function prototypes
void read_rtc_time(char *buffer, size_t buffer_size);
void set_rtc_time(const char *command_buffer, char *response_buffer, size_t response_size);

#endif // COMMAND_H
