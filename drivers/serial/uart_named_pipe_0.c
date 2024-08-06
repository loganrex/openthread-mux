/**
 * @brief UART Driver for interacting with named pipes
 *
 * @note  Driver can open and send characters to the named ports
 *
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <nsi_tracing.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <sys/stat.h>

#include "cmdline.h"
#include "posix_native_task.h"

#define WARN(...) nsi_print_warning(__VA_ARGS__)
#define ERROR(...) nsi_print_error_and_exit(__VA_ARGS__)

#define DT_DRV_COMPAT etc_uart_named_pipe_0

#define TX_PIPE "/tmp/tx_pipe-uart0"
#define RX_PIPE "/tmp/rx_pipe-uart0"

struct named_pipe_data
{
    /* File descriptor used for the named pipe. */
    int rx_fd;
    int tx_fd;
    char *rx_pipe;
    char *tx_pipe;
    char *cmd_rx_pipe;
    char *cmd_tx_pipe;
    /* Emulated tx irq is enabled. */
    bool tx_irq_enabled;
    /* Emulated rx irq is enabled. */
    bool rx_irq_enabled;
    /* IRQ callback */
    uart_irq_callback_user_data_t callback;
    /* IRQ callback data */
    void *cb_data;
};

struct named_pipe_config
{
    struct uart_config uart_config;
};

static struct k_thread rx_thread;
static K_KERNEL_STACK_DEFINE(rx_stack, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);
static struct k_thread tx_thread;
static K_KERNEL_STACK_DEFINE(tx_stack, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);
static struct k_thread irq_thread;
static K_KERNEL_STACK_DEFINE(irq_stack, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);

static unsigned char __aligned(4) txq_ring_buffer[256];
static struct k_pipe txq;
static unsigned char __aligned(4) rxq_ring_buffer[256];
static struct k_pipe rxq;

static K_SEM_DEFINE(tx_sem, 0, 1);
static K_SEM_DEFINE(rx_sem, 0, 1);

#define NAMED_PIPE_INIT_LEVEL POST_KERNEL

/*
 * @brief Output a character towards the serial port
 *
 * @param dev		UART device structure.
 * @param out_char	Character to send.
 */
static void named_pipe_uart_poll_out(const struct device *dev, unsigned char out_char)
{
    ARG_UNUSED(dev);
    size_t bytes_written;
    k_pipe_put(&txq, &out_char, 1, &bytes_written, 1, K_NO_WAIT);
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
static int named_pipe_uart_poll_in(const struct device *dev, unsigned char *p_char)
{
    ARG_UNUSED(dev);
    size_t bytes_read;
    return (k_pipe_get(&rxq, &p_char, 1, &bytes_read, 1, K_NO_WAIT) == 0 ? 0 : -1);
}

static int named_pipe_uart_fifo_fill(const struct device *dev,
                                     const uint8_t *tx_data,
                                     int size)
{
    ARG_UNUSED(dev);
    size_t sent;
    k_pipe_put(&txq, tx_data, size, &sent, 1, K_NO_WAIT);
    return (int)(sent);
}

static int named_pipe_uart_fifo_read(const struct device *dev,
                                     uint8_t *rx_data,
                                     const int size)
{
    ARG_UNUSED(dev);
    size_t bytes_read;
    k_pipe_get(&rxq, rx_data, size, &bytes_read, 1, K_NO_WAIT);
    return (int)(bytes_read);
}

static int named_pipe_uart_irq_tx_ready(const struct device *dev)
{
    struct named_pipe_data *data = dev->data;
    return data->tx_irq_enabled ? 1 : 0;
}

static int named_pipe_uart_irq_tx_complete(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 1;
}

static void named_pipe_uart_irq_tx_enable(const struct device *dev)
{
    struct named_pipe_data *data = dev->data;
    data->tx_irq_enabled = true;
}

static void named_pipe_uart_irq_tx_disable(const struct device *dev)
{
    struct named_pipe_data *data = dev->data;
    data->tx_irq_enabled = false;
}

static void named_pipe_uart_irq_rx_enable(const struct device *dev)
{
    struct named_pipe_data *data = dev->data;
    data->rx_irq_enabled = true;
}

static void named_pipe_uart_irq_rx_disable(const struct device *dev)
{
    struct named_pipe_data *data = dev->data;
    data->rx_irq_enabled = false;
}

static int named_pipe_uart_irq_rx_ready(const struct device *dev)
{
    struct named_pipe_data *data = dev->data;
    if (data->rx_irq_enabled && k_pipe_read_avail(&rxq))
    {
        return 1;
    }
    return 0;
}

static int named_pipe_uart_irq_is_pending(const struct device *dev)
{
    return named_pipe_uart_irq_rx_ready(dev) ||
           named_pipe_uart_irq_tx_ready(dev);
}

static int named_pipe_uart_irq_update(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 1;
}

static int named_pipe_configure(const struct device *dev, const struct uart_config *cfg)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cfg);
    return 0;
}

static void named_pipe_uart_irq_handler(const struct device *dev)
{
    struct named_pipe_data *data = dev->data;

    if (data->callback)
    {
        data->callback(dev, data->cb_data);
    }
    else
    {
        WARN("No callback!\n");
    }
}


static void named_pipe_uart_irq_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    struct device *dev = (struct device *)arg1;
    struct named_pipe_data *data = dev->data;
    volatile int isRunning = 1;

    while (isRunning)
    {
        if (data->rx_irq_enabled)
        {
            if(!k_pipe_read_avail(&rxq))
            {
                k_sem_take(&rx_sem, K_MSEC(10));
            }
            named_pipe_uart_irq_handler(dev);
        }
        if (data->tx_irq_enabled)
        {
            if(!k_pipe_read_avail(&txq))
            {
                k_sem_take(&tx_sem, K_MSEC(10));
            }
            named_pipe_uart_irq_handler(dev);
        }
        if (data->tx_irq_enabled == false && data->rx_irq_enabled == false)
        {
            k_sleep(K_MSEC(10));
        }
        if(k_pipe_read_avail(&rxq))
        {
            k_sem_give(&rx_sem);
        }
        if(k_pipe_read_avail(&txq))
        {
            k_sem_give(&tx_sem);
        }
    }
}

static void named_pipe_uart_rx_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    struct device *dev = (struct device *)arg1;
    struct named_pipe_data *data = dev->data;
    volatile int isRunning = 1;

    uint8_t buffer[128];
    size_t bytes_written;

        // Open the RX pipe for reading
    //data->rx_fd = open(data->rx_pipe, O_RDONLY|O_NONBLOCK  );
    //if (data->rx_fd == -1) {
    //    perror("open RX_PIPE");
    //}

    while (isRunning)
    {

        if (data->rx_fd > 0)
        {
            int size = read(data->rx_fd, buffer, sizeof(buffer));
            if (size > 0)
            {
                k_pipe_put(&rxq, buffer, size, &bytes_written, 1, K_NO_WAIT);
                k_sem_give(&tx_sem);
            }
        }
        else
        {
            k_sleep(K_MSEC(100));
        }
    }
}

static void named_pipe_uart_tx_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    struct device *dev = (struct device *)arg1;
    struct named_pipe_data *data = dev->data;
    ARG_UNUSED(data);
    volatile int isRunning = 1;

    uint8_t buffer[128];
    size_t bytes_read;

    while (isRunning)
    {
        int rc = k_pipe_get(&txq, buffer, 128, &bytes_read, 1, K_MSEC(100));
        if ((rc == 0) && (bytes_read > 0))
        {
            if (data->tx_fd == -1)
            {
                data->tx_fd = open(data->tx_pipe, O_WRONLY | O_NONBLOCK);
                if (data->tx_fd > 0)
                {
                    write(data->tx_fd, "Hello World\r\n", 13);
                    posix_print_trace("Hello World\r\n");
                }
            }
            if (data->tx_fd > 0)
            {
                write(data->tx_fd, buffer, bytes_read);
            }
            else
            {
                k_sleep(K_MSEC(100));
            }
        }
    }
}

static void named_pipe_uart_irq_callback_set(const struct device *dev,
                                             uart_irq_callback_user_data_t cb,
                                             void *cb_data)
{
    struct named_pipe_data *data = dev->data;

    data->callback = cb;
    data->cb_data = cb_data;
}

static void named_pipe_thread_init(const struct device *dev)
{
    k_pipe_init(&txq, txq_ring_buffer, sizeof(txq_ring_buffer));
    k_pipe_init(&rxq, rxq_ring_buffer, sizeof(rxq_ring_buffer));
    k_tid_t tid;

    tid = k_thread_create(&irq_thread, irq_stack, K_KERNEL_STACK_SIZEOF(irq_stack),
                          named_pipe_uart_irq_thread,
                          (void *)dev, NULL, NULL,
                          K_HIGHEST_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(tid, DEVICE_DT_NAME(DT_DRV_INST(0)) "-np_0_irq");

    tid = k_thread_create(&tx_thread, tx_stack, K_KERNEL_STACK_SIZEOF(tx_stack),
                          named_pipe_uart_tx_thread,
                          (void *)dev, NULL, NULL,
                          K_HIGHEST_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(tid, DEVICE_DT_NAME(DT_DRV_INST(0)) "-np_0_tx");

    tid = k_thread_create(&rx_thread, rx_stack, K_KERNEL_STACK_SIZEOF(rx_stack),
                          named_pipe_uart_rx_thread,
                          (void *)dev, NULL, NULL,
                          K_HIGHEST_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(tid, DEVICE_DT_NAME(DT_DRV_INST(0)) "-np_0_rx");
}

static int named_pipe_serial_init(const struct device *dev)
{
    struct named_pipe_data *data = dev->data;

    /* Default value for cmd_rx_pipe is NULL, this is due to the set 's' type in
     * command line opts. If it is anything else then it was configured via command
     * line.
     */
    if (data->cmd_rx_pipe)
    {
        data->rx_pipe = data->cmd_rx_pipe;
    }

    if (data->cmd_tx_pipe)
    {
        data->tx_pipe = data->cmd_tx_pipe;
    }

    //if (mkfifo(data->tx_pipe, 0666) == -1) {
    //    perror("mkfifo TX_PIPE");
    //}
//
    //if (mkfifo(data->rx_pipe, 0666) == -1) {
    //    perror("mkfifo RX_PIPE");
    //}

    /* Start irq emulation threads */
    named_pipe_thread_init(dev);
    posix_print_trace("named_pipe_serial_init finished\r\n");
    return 0;
}

static struct uart_driver_api named_pipe_uart_driver_api = {
    .poll_out = named_pipe_uart_poll_out,
    .poll_in = named_pipe_uart_poll_in,
    .configure = named_pipe_configure,
    .fifo_fill = named_pipe_uart_fifo_fill,
    .fifo_read = named_pipe_uart_fifo_read,
    .irq_tx_enable = named_pipe_uart_irq_tx_enable,
    .irq_tx_disable = named_pipe_uart_irq_tx_disable,
    .irq_tx_ready = named_pipe_uart_irq_tx_ready,
    .irq_tx_complete = named_pipe_uart_irq_tx_complete,
    .irq_rx_enable = named_pipe_uart_irq_rx_enable,
    .irq_rx_disable = named_pipe_uart_irq_rx_disable,
    .irq_rx_ready = named_pipe_uart_irq_rx_ready,
    .irq_is_pending = named_pipe_uart_irq_is_pending,
    .irq_update = named_pipe_uart_irq_update,
    .irq_callback_set = named_pipe_uart_irq_callback_set,
};

static const struct named_pipe_config named_pipe_cfg = {
    .uart_config =
        {
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
            .parity = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1,
            .baudrate = 115200,
        },
};

static struct named_pipe_data named_pipe_data = {
    .rx_pipe = DT_INST_PROP_OR(0, rx_pipe, RX_PIPE),
    .tx_pipe = DT_INST_PROP_OR(0, tx_pipe, TX_PIPE),
};

DEVICE_DT_INST_DEFINE(0, named_pipe_serial_init, NULL, &named_pipe_data,
                      &named_pipe_cfg, NAMED_PIPE_INIT_LEVEL, 57,
                      &named_pipe_uart_driver_api);

static void named_pipe_add_serial_options(void)
{
    static struct args_struct_t opts[] = {
        {
            .option = DEVICE_DT_NAME(DT_DRV_INST(0)) "_rx_pipe",
            .name = "\"rx_pipe\"",
            .type = 's',
            .dest = &named_pipe_data.cmd_rx_pipe,
            .descript = "Set rx pipe for " DEVICE_DT_NAME(DT_DRV_INST(0)) " uart device, "
                         "overriding " DT_INST_PROP(0, rx_pipe) "  in devicetree.",
        },
        {
            .option = DEVICE_DT_NAME(DT_DRV_INST(0)) "_tx_pipe",
            .name = "\"tx_pipe\"",
            .type = 's',
            .dest = &named_pipe_data.cmd_tx_pipe,
            .descript = "Set tx pipe for " DEVICE_DT_NAME(DT_DRV_INST(0)) " uart device, "
                         "overriding " DT_INST_PROP(0, tx_pipe) "  in devicetree.",
        },
        ARG_TABLE_ENDMARKER
        };

    native_add_command_line_opts(opts);
}

/**
 * @brief Cleans up any open serial ports on the exit.
 */
static void named_pipe_cleanup_uart(void)
{
    if (named_pipe_data.rx_fd != 0)
    {
        close(named_pipe_data.rx_fd);
    }
    if (named_pipe_data.tx_fd != 0)
    {
        close(named_pipe_data.tx_fd);
    }
}

NATIVE_TASK(named_pipe_add_serial_options, PRE_BOOT_1, 11);
NATIVE_TASK(named_pipe_cleanup_uart, ON_EXIT, 99);
