/**
 * @brief "Bottom" of native tcp uart driver
 *
 * When built with the native_simulator this will be built in the runner context,
 * that is, with the host C library, and with the host include paths.
 *
 */

#ifndef DRIVERS_SERIAL_UART_NATIVE_TCP_BOTTOM_H
#define DRIVERS_SERIAL_UART_NATIVE_TCP_BOTTOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int native_tcp_poll_bottom(int fd);
int native_tcp_open_tcp_bottom(const char *ip, int port);
long native_tcp_recv_bottom(int fd, void *buffer, unsigned long size);
long native_tcp_send_bottom(int fd, const void *buffer, unsigned long size);
int native_tcp_close_bottom(int fd);
int native_tcp_server_listen_bottom(int *fd,const char *ip, int port);


#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_SERIAL_UART_NATIVE_TCP_BOTTOM_H */
