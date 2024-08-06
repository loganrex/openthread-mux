/**
 * @brief "Bottom" of native tcp uart driver
 *
 * Copyright (c) 2023 Marko Sagadin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uart_native_tcp_bottom.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define SOCKET_ERROR    (-1)
#define INVALID_SOCKET  (~0)
#define TRUE  1
#define FALSE 0
#define closesocket close

#include <nsi_tracing.h>

#define WARN(...)  nsi_print_warning(__VA_ARGS__)
#define ERROR(...) nsi_print_error_and_exit(__VA_ARGS__)

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))


int native_tcp_poll_bottom(int fd)
{
    if (fd > 0)
    {
        struct pollfd pfd = {.fd = fd, .events = POLLIN};

        return poll(&pfd, 1, 0);
    }
    return 0;
}

int native_tcp_server_listen_bottom(int *fd,const char *ip, int port)
{
    volatile int isRunning =1;
    int listener = -1;
    struct sockaddr_in serveraddr;
    

    if ((listener = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)) < 0)
    {
        return -1;
    }

    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = inet_addr(ip);
    serveraddr.sin_port = htons(port);
    memset(&(serveraddr.sin_zero), '\0', 8);

    if (bind(listener, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0 )
    {
        perror("Server-bind() error!");
        return -1;
    }

    int timeout = 5000; // Timeout in milliseconds (5 seconds)

    fd_set read_fds;
    fd_set master;
    int fdMax = -1;
    FD_ZERO(&read_fds);
    FD_ZERO(&master);

    while(isRunning)
    {
        fdMax = -1;
        FD_ZERO(&master);
        if (listener > -1)
        {
            if (listener > fdMax)
            {
                fdMax = listener;
            }
            FD_SET(listener, &master);

            read_fds = master;
            struct timeval tv = { timeout, 0L };

            int result = select(fdMax + 1, &read_fds, NULL, NULL, &tv);
            if (result > 0)
            {
                if (FD_ISSET(listener, &read_fds))
                {
                    struct sockaddr_in clientaddr;
                    socklen_t addrlen = sizeof(clientaddr);
                    int fdNew = -1;
                    if ((fdNew = accept(listener, (struct sockaddr *)&clientaddr, (socklen_t *)&addrlen)) > 0)
                    {
                        native_tcp_close_bottom(*fd);
                        *fd=fdNew;
                    }
                }
            }
        }
        else
        {
            return -1;
        }
        
    }

    return 0;
}

int native_tcp_open_tcp_bottom(const char *ip, int port)
{
    int fd = -1;
    struct sockaddr_in serveraddr;

    if ((fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)) < 0)
    {
        return -1;
    }

    memset(&serveraddr, 0x00, sizeof(struct sockaddr_in));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons((uint16_t)port);

    serveraddr.sin_addr.s_addr = inet_addr(ip);

    if ((connect(fd, (struct sockaddr *)&serveraddr, sizeof(serveraddr))) < 0)
    {
        if (fd > -1) { close(fd); }
        fd= -1;
    }

    if (fd < 0) {
        WARN("Failed to open serial port host:%s port:%d, errno: %i\n", ip, port, errno);
    }

    return fd;
}

long native_tcp_recv_bottom(int fd, void *buffer, unsigned long size)
{
    int flags = 0;
    if (fd > 0)
    {
        return recv(fd, buffer, size, flags);
    }
    return 0;
}


long native_tcp_send_bottom(int fd, const void *buffer, unsigned long size)
{
    int flags = 0;
    if (fd > 0)
    {
        return send(fd, buffer, size, flags);
    }
    return 0;
}

int native_tcp_close_bottom(int fd)
{
    if (fd > 0)
    {
        return close(fd);
    }
    return 0;
}

