#ifndef PRINTF_CONFIG_H
#define PRINTF_CONFIG_H

// disable unused options in the printf implementation to save FLASH space
// printf_config.h file required for the action to work
#define PRINTF_DISABLE_SUPPORT_FLOAT
#define PRINTF_DISABLE_SUPPORT_EXPONENTIAL
#define PRINTF_DISABLE_SUPPORT_PTRDIFF_T
#define PRINTF_DISABLE_SUPPORT_LONG_LONG

#endif // PRINTF_CONFIG_H
