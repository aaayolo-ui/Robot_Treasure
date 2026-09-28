#ifndef UART_DEBUG_H
#define UART_DEBUG_H

void UartDebug_SendString(const char *text);
void UartDebug_Printf(const char *format, ...);
void UartDebug_Task(void);

#endif /* UART_DEBUG_H */
