/**
 * @brief UART Driver for interacting with host TCP client ports
 *
 * @note  Driver can open and send characters to the TCP ports
 *
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <nsi_tracing.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

#include <pthread.h>
#include <limits.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define SOCKET_ERROR (-1)
#define INVALID_SOCKET (~0)
#define TRUE 1
#define FALSE 0
#define closesocket close

#include "cmdline.h"
#include "posix_native_task.h"

#define WARN(...) nsi_print_warning(__VA_ARGS__)
#define ERROR(...) nsi_print_error_and_exit(__VA_ARGS__)

#define DT_DRV_COMPAT etc_native_tcp_uart_1

#define RING_BUF_SIZE 1024


struct native_tcp_data
{
    /* File descriptor used for the tcp device. */
    int fd;
    int listen_fd;
    char *tcp_host;
    int tcp_port;
    bool is_server;
    char *cmd_tcp_host;
    int cmd_tcp_port;
    bool cmd_is_server;
    /* Emulated tx irq is enabled. */
    bool tx_irq_enabled;
    /* Emulated rx irq is enabled. */
    bool rx_irq_enabled;
    /* IRQ callback */
    uart_irq_callback_user_data_t callback;
    /* IRQ callback data */
    void *cb_data;

    uint8_t iface_rb_buf[RING_BUF_SIZE];
    struct ring_buf rx_rb;
    struct k_mutex rx_mutex;
    struct k_sem rxtx_sem;
};

struct native_tcp_config
{
    struct uart_config uart_config;
};


//static struct k_thread rx_thread;
//static K_KERNEL_STACK_DEFINE(rx_stack, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);
//static struct k_thread tx_thread;
//static K_KERNEL_STACK_DEFINE(tx_stack, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);
static struct k_thread irq_thread;
static K_KERNEL_STACK_DEFINE(irq_stack, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);

#define NATIVE_TCP_INIT_LEVEL POST_KERNEL

/*
 * @brief Output a character towards the serial port
 *
 * @param dev		UART device structure.
 * @param out_char	Character to send.
 */
static void native_tcp_uart_poll_out(const struct device *dev, unsigned char out_char)
{
    struct native_tcp_data *data = dev->data;
    if (data->fd > 0)
    {
        send(data->fd, &out_char, 1, 0);
    }
}

/**
 * @brief Poll the device for input.
 *
 * @param dev		UART device structure.
 * @param p_char	Pointer to a character.
 *
 * @retval 0	If a character arrived.
 * @retval -1	If no character was available to read.
 */
static int native_tcp_uart_poll_in(const struct device *dev, unsigned char *p_char)
{
    struct native_tcp_data *data = dev->data;
    k_mutex_lock(&data->rx_mutex, K_FOREVER);
    int nbytes = ring_buf_get(&data->rx_rb, p_char, 1);
    k_mutex_unlock(&data->rx_mutex);
    return (nbytes ? 0 : -1);
}

static int native_tcp_uart_fifo_fill(const struct device *dev,
                                     const uint8_t *tx_data,
                                     int size)
{
    size_t sent=0;
    struct native_tcp_data *data = dev->data;
    if (data->fd > 0)
    {
        sent=send(data->fd, tx_data, size, 0);
    }
    return (int)(sent);
}

static int native_tcp_uart_fifo_read(const struct device *dev,
                                     uint8_t *rx_data,
                                     const int size)
{
    struct native_tcp_data *data = dev->data;
    k_mutex_lock(&data->rx_mutex, K_FOREVER);
    int nbytes = ring_buf_get(&data->rx_rb, rx_data, size);
    k_mutex_unlock(&data->rx_mutex);
    return nbytes;
}

static int native_tcp_uart_irq_tx_ready(const struct device *dev)
{
    struct native_tcp_data *data = dev->data;
    return data->tx_irq_enabled ? 1 : 0;
}

static int native_tcp_uart_irq_tx_complete(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 1;
}

static void native_tcp_uart_irq_tx_enable(const struct device *dev)
{
    struct native_tcp_data *data = dev->data;
    data->tx_irq_enabled = true;
    k_sem_give(&data->rxtx_sem);
}

static void native_tcp_uart_irq_tx_disable(const struct device *dev)
{
    struct native_tcp_data *data = dev->data;
    data->tx_irq_enabled = false;
}

static void native_tcp_uart_irq_rx_enable(const struct device *dev)
{
    struct native_tcp_data *data = dev->data;
    data->rx_irq_enabled = true;
}

static void native_tcp_uart_irq_rx_disable(const struct device *dev)
{
    struct native_tcp_data *data = dev->data;
    data->rx_irq_enabled = false;
}

static int native_tcp_uart_irq_rx_ready(const struct device *dev)
{
    struct native_tcp_data *data = dev->data;
    if (data->rx_irq_enabled)
    {
        k_mutex_lock(&data->rx_mutex, K_FOREVER);
        bool is_empty=ring_buf_is_empty(&data->rx_rb);
        k_mutex_unlock(&data->rx_mutex);
        return !is_empty;
    }
    return 0;
}

static int native_tcp_uart_irq_is_pending(const struct device *dev)
{
    return native_tcp_uart_irq_rx_ready(dev) ||
           native_tcp_uart_irq_tx_ready(dev);
}

static int native_tcp_uart_irq_update(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 1;
}

static int native_tcp_configure(const struct device *dev, const struct uart_config *cfg)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cfg);
    return 0;
}

static void native_tcp_uart_irq_handler(const struct device *dev)
{
    struct native_tcp_data *data = dev->data;

    if (data->callback)
    {
        data->callback(dev, data->cb_data);
    }
    else
    {
        WARN("No callback!\n");
    }
}

static void setblocking(int fd, bool val)
{
    int fl, res;

    fl = fcntl(fd, F_GETFL, 0);
 
    if (val)
    {
        fl &= ~O_NONBLOCK;
    }
    else
    {
        fl |= O_NONBLOCK;
    }

    res = fcntl(fd, F_SETFL, fl);
}

static int native_tcp_open_tcp_helper(const char *ip, int port)
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
        if (fd > -1)
        {
            close(fd);
        }
        fd = -1;
    }

    if (fd > 0)
    {
        WARN("Opend serial port host:%s port:%d\n", ip, port);
    }

    setblocking(fd, false);

    return fd;
}

static void native_tcp_uart_irq_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    struct device *dev = (struct device *)arg1;
    struct native_tcp_data *data = dev->data;
    volatile int isRunning = 1;
    
    //int rc=0;
    //uint8_t buf[128];
    //size_t bytes_read=0;

    while (isRunning)
    {
        if (data->fd > 0)
        {
#if 1
            while(native_tcp_uart_irq_is_pending(dev))
            {
                native_tcp_uart_irq_handler(dev);
                k_sleep(K_MSEC(10));
            }
            if (data->tx_irq_enabled == false && data->rx_irq_enabled == false)
            {
                k_sleep(K_MSEC(100));
            }
            else
            {
                k_sleep(K_MSEC(10));
                k_sem_take(&data->rxtx_sem, K_MSEC(100));
            }
#else
            if (data->rx_irq_enabled)
            {
                if (!k_pipe_read_avail(&RXQ))
                {
                    k_sem_take(&rxtx_sem, K_MSEC(10));
                }
                native_tcp_uart_irq_handler(dev);
            }
            if (data->tx_irq_enabled)
            {
                native_tcp_uart_irq_handler(dev);
            }
            if (data->tx_irq_enabled == false && data->rx_irq_enabled == false)
            {
                k_sleep(K_MSEC(10));
            }
            if (k_pipe_read_avail(&RXQ))
            {
                k_sem_give(&rxtx_sem);
            }
#endif
        }
        else
        {
            k_sleep(K_MSEC(10));
        }
    }
}

static void * native_tcp_uart_rx_thread(void *arg1)
{
    struct device *dev = (struct device *)arg1;
    struct native_tcp_data *data = dev->data;
    volatile int isRunning = 1;

    int listener = -1;

    uint8_t buffer[128];
    struct sockaddr_in serveraddr;

    if (data->is_server)
    {
        if (data->fd > 0)
        {
            close(data->fd);
        }
        data->fd = -1;
        int yes = 1;
        if ((listener = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)) < 0)
        {
            listener = -1;
        }
        if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, (char *)&yes, sizeof(int)) == SOCKET_ERROR)
        {
            listener = -1;
        }
        serveraddr.sin_family = AF_INET;
        serveraddr.sin_addr.s_addr = inet_addr(data->tcp_host);
        serveraddr.sin_port = htons(data->tcp_port);
        memset(&(serveraddr.sin_zero), '\0', 8);

        if (bind(listener, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0)
        {
            perror("Server-bind() error!");
            listener = -1;
        }
        setblocking(listener, false);
        if (listen(listener, 10) == -1)
        {
            listener = -1;
        }
    }

    fd_set read_fds;
    fd_set master;
    int fdMax = -1;
    FD_ZERO(&read_fds);
    FD_ZERO(&master);

    while (isRunning)
    {
        if ((data->fd > 0) || (listener > 0))
        {
            FD_ZERO(&master);

            if (data->fd > 0)
            {
                if (data->fd > fdMax)
                {
                    fdMax = data->fd;
                }
                FD_SET(data->fd, &master);
            }

            if (listener > 0)
            {
                if (listener > fdMax)
                {
                    fdMax = listener;
                }
                FD_SET(listener, &master);
            }

            read_fds = master;

            int result = select(fdMax + 1, &read_fds, NULL, NULL,NULL);
            if (result == -1)
            {
                printf("select error: %d\n", errno);
                continue;
            }
            if (result > 0)
            {
                if (FD_ISSET(listener, &read_fds))
                {
                    struct sockaddr_in clientaddr;
                    socklen_t addrlen = sizeof(clientaddr);
                    int fdNew = -1;
                    if ((fdNew = accept(listener, (struct sockaddr *)&clientaddr, (socklen_t *)&addrlen)) > -1)
                    {
                        if (data->fd > 0)
                        {
                            close(data->fd);
                        }
                        data->fd = fdNew;
                        setblocking(data->fd, false);
                    }
                }
                if (FD_ISSET(data->fd, &read_fds))
                {
                    int size = 0;
                    int flags = 0;
                    if ((size = recv(data->fd, buffer, sizeof(buffer), flags)) <= 0)
                    {
                        if (data->fd >= 0)
                        {
                            close(data->fd);
                            data->fd = -1;
                        }
                    }
                    else
                    {
                        k_mutex_lock(&data->rx_mutex, K_FOREVER);
                        uint32_t partial_size = 0;
                        uint8_t *dst;
                        partial_size = ring_buf_put_claim(&data->rx_rb, &dst,size);
                        if (partial_size)
                        {
                            memcpy(dst, buffer,partial_size);
                        }
                        ring_buf_put_finish(&data->rx_rb, partial_size);
                        k_mutex_unlock(&data->rx_mutex);
                        k_sem_give(&data->rxtx_sem);
                    }
                }
            }
            else
            {
            }
        }
        else
        {
            if (!data->is_server)
            {
                data->fd = native_tcp_open_tcp_helper(data->tcp_host, data->tcp_port);
                if (data->fd > 0)
                {
                    posix_print_trace("reconnected to the host:%s port:%d\n", data->tcp_host, data->tcp_port);
                }
            }
        }
    }

    return NULL;
}

#if 0

static void native_tcp_uart_tx_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    struct device *dev = (struct device *)arg1;
    struct native_tcp_data *data = dev->data;
    ARG_UNUSED(data);
    volatile int isRunning = 1;

    uint8_t buffer[128];
    size_t bytes_read;

    while (isRunning)
    {
        if (data->fd > 0)
        {
            int rc = k_pipe_get(&txq, buffer, 128, &bytes_read, 1, K_MSEC(10));
            if ((rc == 0) && (bytes_read > 0))
            {
                int flags = 0;
                send(data->fd, buffer, bytes_read, flags);
            }
        }
        else
        {
            k_sleep(K_MSEC(100));
        }
    }
}

#endif



static void native_tcp_uart_irq_callback_set(const struct device *dev,
                                             uart_irq_callback_user_data_t cb,
                                             void *cb_data)
{
    struct native_tcp_data *data = dev->data;

    data->callback = cb;
    data->cb_data = cb_data;
}

#define STACK_SIZE (2048)
#define PTHREAD_STACK_MIN 16384
//static K_KERNEL_STACK_DEFINE(rx_stack_, STACK_SIZE);

static int native_tcp_pthreads_init(const struct device *dev)
{
    int res;

#ifdef __ZEPHYR__
    pthread_attr_t attr;
    pthread_attr_t *attrp = &attr;
#else
    pthread_attr_t *attrp = NULL;
#endif

#ifdef __ZEPHYR__
    /* Zephyr requires a non-NULL attribute for pthread_create */
    res = pthread_attr_init(attrp);
    if (res != 0)
    {
        errno = res;
        perror("pthread_attr_init");
        return -res;
    }

    res = pthread_attr_setstacksize(attrp, PTHREAD_STACK_MIN + 0x1000);
    if (res != 0)
    {
        errno = res;
        perror("pthread_attr_setstack");
        return -res;
    }
#endif
    pthread_t cThread;
    res = pthread_create(&cThread, attrp, native_tcp_uart_rx_thread, (void *)dev);
    if (res != 0)
    {
        errno = res;
        perror("pthread_create");
        return -res;
    }
    struct sched_param sch;
    int policy; 
    pthread_getschedparam(cThread, &policy, &sch);
    sch.sched_priority = 20;
    if (pthread_setschedparam(cThread, SCHED_FIFO, &sch)) {
        
    }
    return 0;
}

static void native_tcp_thread_init(const struct device *dev)
{
    k_tid_t tid;

    tid = k_thread_create(&irq_thread, irq_stack, K_KERNEL_STACK_SIZEOF(irq_stack),
                          native_tcp_uart_irq_thread,
                          (void *)dev, NULL, NULL,
                          K_HIGHEST_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(tid, DEVICE_DT_NAME(DT_DRV_INST(0)) "-tcp_0_irq");
    posix_print_trace(DEVICE_DT_NAME(DT_DRV_INST(0)) "-tcp_0_irq" " up \n");
#if 0
    tid = k_thread_create(&tx_thread, tx_stack, K_KERNEL_STACK_SIZEOF(tx_stack),
                          native_tcp_uart_tx_thread,
                          (void *)dev, NULL, NULL,
                          K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(tid, DEVICE_DT_NAME(DT_DRV_INST(0)) "-tcp_0_tx");

    posix_print_trace(DEVICE_DT_NAME(DT_DRV_INST(0)) "-tcp_0_tx" " up \n");
#endif
#if 0
    tid = k_thread_create(&rx_thread, rx_stack, K_KERNEL_STACK_SIZEOF(rx_stack),
                          native_tcp_uart_rx_thread,
                          (void *)dev, NULL, NULL,
                          K_LOWEST_APPLICATION_THREAD_PRIO, K_USER, K_NO_WAIT);
    k_thread_name_set(tid, DEVICE_DT_NAME(DT_DRV_INST(0)) "-tcp_0_rx");
    posix_print_trace(DEVICE_DT_NAME(DT_DRV_INST(0)) "-tcp_0_rx" " up \n");
#endif
}

static int native_tcp_serial_init(const struct device *dev)
{
    struct native_tcp_data *data = dev->data;

    ring_buf_init(&data->rx_rb, sizeof(data->iface_rb_buf),data->iface_rb_buf);
    k_sem_init(&data->rxtx_sem, 0, 1);
    k_mutex_init(&data->rx_mutex);


    /* Default value for cmd_tcp_host is NULL, this is due to the set 's' type in
     * command line opts. If it is anything else then it was configured via command
     * line.
     */
    if (data->cmd_tcp_host)
    {
        data->tcp_host = data->cmd_tcp_host;
    }

    /* Default value for cmd_tcp_port is UINT32_MAX, this is due to the set 'u' type in
     * command line opts. If it is anything else then it was configured via command
     * line.
     */
    if (data->cmd_tcp_port != UINT32_MAX)
    {
        data->tcp_port = data->cmd_tcp_port;
    }

    if (data->cmd_is_server)
    {
        data->is_server = data->cmd_is_server;
    }

    if (!data->tcp_port)
    {
        ERROR("%s: tcp_port was not set.\n", dev->name);
    }

    if (data->is_server)
    {
        data->fd = -1;
    }
    else
    {
        data->fd = native_tcp_open_tcp_helper(data->tcp_host, data->tcp_port);
        posix_print_trace("%s connected to the host:%s port:%d\n", dev->name, data->tcp_host, data->tcp_port);
    }

    /* Start irq emulation threads */
    native_tcp_thread_init(dev);
    native_tcp_pthreads_init(dev);
    return 0;
}

static struct uart_driver_api native_tcp_uart_driver_api = {
    .poll_out = native_tcp_uart_poll_out,
    .poll_in = native_tcp_uart_poll_in,
    .configure = native_tcp_configure,
    .fifo_fill = native_tcp_uart_fifo_fill,
    .fifo_read = native_tcp_uart_fifo_read,
    .irq_tx_enable = native_tcp_uart_irq_tx_enable,
    .irq_tx_disable = native_tcp_uart_irq_tx_disable,
    .irq_tx_ready = native_tcp_uart_irq_tx_ready,
    .irq_tx_complete = native_tcp_uart_irq_tx_complete,
    .irq_rx_enable = native_tcp_uart_irq_rx_enable,
    .irq_rx_disable = native_tcp_uart_irq_rx_disable,
    .irq_rx_ready = native_tcp_uart_irq_rx_ready,
    .irq_is_pending = native_tcp_uart_irq_is_pending,
    .irq_update = native_tcp_uart_irq_update,
    .irq_callback_set = native_tcp_uart_irq_callback_set,
};

static const struct native_tcp_config native_tcp_cfg = {
    .uart_config =
        {
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
            .parity = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1,
            .baudrate = 115200,
        },
};

static struct native_tcp_data native_tcp_data = {
    .tcp_host = DT_INST_PROP_OR(0, tcp_host, NULL),
    .tcp_port = DT_INST_PROP(0, tcp_port),
    .is_server = DT_INST_PROP(0, is_server),
};

DEVICE_DT_INST_DEFINE(0, native_tcp_serial_init, NULL, &native_tcp_data,
                      &native_tcp_cfg, NATIVE_TCP_INIT_LEVEL, 57,
                      &native_tcp_uart_driver_api);

static void native_tcp_add_serial_options(void)
{
    static struct args_struct_t opts[] = {
        {
            .option = DEVICE_DT_NAME(DT_DRV_INST(0)) "_tcp_host",
            .name = "\"tcp_host\"",
            .type = 's',
            .dest = &native_tcp_data.cmd_tcp_host,
            .descript = "Set tcp host for " DEVICE_DT_NAME(DT_DRV_INST(0)) " uart device, "
                                                                           "overriding " DT_INST_PROP(0, tcp_host) "  in devicetree.",
        },
        {
            .option = DEVICE_DT_NAME(DT_DRV_INST(0)) "_tcp_port",
            .name = "tcp_port",
            .type = 'u',
            .dest = &native_tcp_data.cmd_tcp_port,
            .descript = "Set the tcp port for " DEVICE_DT_NAME(DT_DRV_INST(0)) " device, overriding the "
                                                                               "baudrate of " STRINGIFY(DT_INST_PROP(0, tcp_port)) " set in the devicetree.",
        },
        {.is_switch = true,
         .option = DEVICE_DT_NAME(DT_DRV_INST(0)) "_is_server",
         .type = 'b',
         .dest = &native_tcp_data.cmd_is_server,
         .descript = "have the tcp connection act as a server"},
        ARG_TABLE_ENDMARKER};

    native_add_command_line_opts(opts);
}

/**
 * @brief Cleans up any open serial ports on the exit.
 */
static void native_tcp_cleanup_uart(void)
{
    if (native_tcp_data.fd != 0)
    {
        close(native_tcp_data.fd);
    }
}

NATIVE_TASK(native_tcp_add_serial_options, PRE_BOOT_1, 11);
NATIVE_TASK(native_tcp_cleanup_uart, ON_EXIT, 99);
