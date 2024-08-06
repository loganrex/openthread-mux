/**
 * @brief UART Driver for interacting with host TCP client ports
 *
 * @note  Driver can open and send characters to the TCP ports  
 *
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <nsi_tracing.h>

#include "cmdline.h"
#include "posix_native_task.h"
#include "uart_native_tcp_bottom.h"

#define WARN(...)  nsi_print_warning(__VA_ARGS__)
#define ERROR(...) nsi_print_error_and_exit(__VA_ARGS__)

#define DT_DRV_COMPAT etc_native_tcp_uart

struct native_tcp_data {
    /* File descriptor used for the tcp device. */
    int fd;
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
};

struct native_tcp_config {
    struct uart_config uart_config;
};

#define NATIVE_TCP_RX_DEF(inst) \
static struct k_thread rx_thread_##inst ; \
static K_KERNEL_STACK_DEFINE(rx_stack_##inst , CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE); \
static struct k_thread listen_thread_##inst ; \
static K_KERNEL_STACK_DEFINE(listen_stack_##inst , CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE); \


DT_INST_FOREACH_STATUS_OKAY(NATIVE_TCP_RX_DEF);

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

    int ret = native_tcp_send_bottom(data->fd, &out_char, 1);

    if (ret == -1) {
        ERROR("Could not write to %s %d\n", data->tcp_host,data->tcp_port);
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

    return native_tcp_recv_bottom(data->fd, p_char, 1) > 0 ? 0 : -1;
}

static int native_tcp_configure(const struct device *dev, const struct uart_config *cfg)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cfg);
    return 0;
}

static int native_tcp_uart_fifo_fill(const struct device *dev,
                     const uint8_t *tx_data,
                     int size)
{
    struct native_tcp_data *data = dev->data;

    return native_tcp_send_bottom(data->fd, (void *)tx_data, size);
}

static int native_tcp_uart_fifo_read(const struct device *dev,
                     uint8_t *rx_data,
                     const int size)
{
    struct native_tcp_data *data = dev->data;

    return native_tcp_recv_bottom(data->fd, rx_data, size);
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

    if (data->rx_irq_enabled && native_tcp_poll_bottom(data->fd) == 1) {
        return 1;
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

static void native_tcp_uart_irq_handler(const struct device *dev)
{
    struct native_tcp_data *data = dev->data;

    if (data->callback) {
        data->callback(dev, data->cb_data);
    } else {
        WARN("No callback!\n");
    }
}

static  void native_tcp_uart_listen_server(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    struct device *dev = (struct device *)arg1;
    struct native_tcp_data *data = dev->data;
    //int *inst = (int *)arg2;

    while (1) {
        native_tcp_server_listen_bottom(&(data->fd),data->tcp_host, data->tcp_port);
        k_sleep(K_MSEC(1000));
    }
}

/*
 * Emulate uart interrupts using a polling thread
 */
#define NATIVE_TCP_UART_IRQ_FUNCTION(inst) \
static void native_tcp_uart_irq_function##inst (void *arg1, void *arg2, void *arg3) \
{                                                                     \
    ARG_UNUSED(arg2);                                                 \
    ARG_UNUSED(arg3);                                                 \
    struct device *dev = (struct device *)arg1;                       \
    struct native_tcp_data *data = dev->data;                         \
    volatile int isRunning =1;                                        \
                                                                      \
    while (isRunning) {                                               \
        if (data->rx_irq_enabled) {                                   \
            int ret = native_tcp_poll_bottom(data->fd);               \
                                                                      \
            if (ret == 1) {                                           \
                native_tcp_uart_irq_handler(dev);                     \
            } else if (ret < 0) {                                     \
                WARN("Poll returned error %d\n", ret);                \
            } else {                                                  \
                k_sleep(K_MSEC(1));                                   \
            }                                                         \
        }                                                             \
        if (data->tx_irq_enabled) {                                   \
            native_tcp_uart_irq_handler(dev);                         \
        }                                                             \
        if (data->tx_irq_enabled == false && data->rx_irq_enabled == false) { \
            k_sleep(K_MSEC(100));                                     \
        }                                                             \
    }                                                                 \
}                                                                     \

DT_INST_FOREACH_STATUS_OKAY(NATIVE_TCP_UART_IRQ_FUNCTION);


static void native_tcp_uart_irq_callback_set(const struct device *dev,
                         uart_irq_callback_user_data_t cb,
                         void *cb_data)
{
    struct native_tcp_data *data = dev->data;

    data->callback = cb;
    data->cb_data = cb_data;
}


#define NATIVE_TCP_CASE(inst) \
        case inst : \
        tid=k_thread_create(&rx_thread_##inst , rx_stack_##inst , K_KERNEL_STACK_SIZEOF(rx_stack_##inst ), \
            native_tcp_uart_irq_function##inst , \
            (void *)dev, (void *)&i, NULL, \
            K_HIGHEST_THREAD_PRIO, 0, K_NO_WAIT); \
        k_thread_name_set(tid, "native_tcp_rx_"#inst ); \
        break;


static void native_tcp_irq_initx(const struct device *dev,int inst)
{

    k_tid_t tid;
    int i=inst;

    switch(inst)
    {

        DT_INST_FOREACH_STATUS_OKAY(NATIVE_TCP_CASE);
        default:
        break;
    }
}


#define NATIVE_TCP_LISTEN_CASE(inst) \
        case inst : \
        tid=k_thread_create(&listen_thread_##inst , listen_stack_##inst , K_KERNEL_STACK_SIZEOF(listen_stack_##inst ), \
            native_tcp_uart_listen_server, \
            (void *)dev, (void *)&i, NULL, \
            K_HIGHEST_THREAD_PRIO, 0, K_NO_WAIT); \
        k_thread_name_set(tid, "native_tcp_listen_"#inst ); \
        break;


static void native_tcp_listen_init(const struct device *dev,int inst)
{

    k_tid_t tid;
    int i = inst;

    switch(inst)
    {

        DT_INST_FOREACH_STATUS_OKAY(NATIVE_TCP_LISTEN_CASE);
        default:
        break;
    }
}

static int native_tcp_serial_initx(const struct device *dev, int inst)
{
    struct native_tcp_data *data = dev->data;
    struct uart_config uart_config = ((struct native_tcp_config *)dev->config)->uart_config;
    
    /* Default value for cmd_tcp_host is NULL, this is due to the set 's' type in
     * command line opts. If it is anything else then it was configured via command
     * line.
     */
    if (data->cmd_tcp_host) {
        data->tcp_host = data->cmd_tcp_host;
    }

    /* Default value for cmd_tcp_port is UINT32_MAX, this is due to the set 'u' type in
     * command line opts. If it is anything else then it was configured via command
     * line.
     */
    if (data->cmd_tcp_port != UINT32_MAX) {
        data->tcp_port = data->cmd_tcp_port;
    }

    if (data->cmd_is_server) {
        data->is_server = data->cmd_is_server;
    }

    /* Serial port needs to be set either in the devicetree or provided via command line
     * opts, if that is not the case, then abort.
     */
    //if (!data->tcp_host) {
    //	ERROR("%s: tcp_host was not set.\n", dev->name);
    //}

    //if (!data->tcp_port) {
    //	ERROR("%s: tcp_port was not set.\n", dev->name);
    //}

    /* Try to open a serial port as with read/write access, also prevent serial port
     * from becoming the controlling terminal.
     */

    if (data->is_server)
    {
        data->fd = -1;
    }
    else
    {
        data->fd = native_tcp_open_tcp_bottom(data->tcp_host, data->tcp_port);
    }

    if (native_tcp_configure(dev, &uart_config)) {
        ERROR("%s: could not configure serial port host: %s port:%d\n", dev->name, data->tcp_host,data->tcp_port);
    }

    if (data->is_server)
    {
        native_tcp_listen_init(dev, inst);
    }
    else
    {
        posix_print_trace("%s connected to the host:%s port:%d\n", dev->name, data->tcp_host, data->tcp_port);
    }

    /* Start irq emulation thread */
    native_tcp_irq_initx(dev,inst);
    return 0;
}

#define NATIVE_TCP_INIT(inst) \
static int native_tcp_serial_init_##inst (const struct device *dev) { \
    return native_tcp_serial_initx(dev, (inst) ); \
} ; \

DT_INST_FOREACH_STATUS_OKAY(NATIVE_TCP_INIT);

static struct uart_driver_api native_tcp_uart_driver_api = {
    .poll_out         = native_tcp_uart_poll_out,
    .poll_in          = native_tcp_uart_poll_in,
    .configure        = native_tcp_configure,
    .fifo_fill        = native_tcp_uart_fifo_fill,
    .fifo_read        = native_tcp_uart_fifo_read,
    .irq_tx_enable    = native_tcp_uart_irq_tx_enable,
    .irq_tx_disable	  = native_tcp_uart_irq_tx_disable,
    .irq_tx_ready     = native_tcp_uart_irq_tx_ready,
    .irq_tx_complete  = native_tcp_uart_irq_tx_complete,
    .irq_rx_enable    = native_tcp_uart_irq_rx_enable,
    .irq_rx_disable   = native_tcp_uart_irq_rx_disable,
    .irq_rx_ready     = native_tcp_uart_irq_rx_ready,
    .irq_is_pending   = native_tcp_uart_irq_is_pending,
    .irq_update       = native_tcp_uart_irq_update,
    .irq_callback_set = native_tcp_uart_irq_callback_set,
};

#define NATIVE_TCP_INSTANCE(inst)                                                  \
    static const struct native_tcp_config native_tcp_##inst##_cfg = {              \
        .uart_config =                                                             \
            {                                                                      \
                .data_bits = UART_CFG_DATA_BITS_8,                                 \
                .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,                              \
                .parity = UART_CFG_PARITY_NONE,                                    \
                .stop_bits = UART_CFG_STOP_BITS_1,                                 \
                .baudrate = 115200,                                                \
            },                                                                     \
    };                                                                             \
                                                                                   \
    static struct native_tcp_data native_tcp_##inst##_data = {                     \
        .tcp_host = DT_INST_PROP_OR(inst, tcp_host, NULL),                         \
        .tcp_port = DT_INST_PROP(inst, tcp_port),                                  \
        .is_server = DT_INST_PROP(inst, is_server),                                \
    };                                                                             \
                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, native_tcp_serial_init_##inst , NULL, &native_tcp_##inst##_data,       \
                  &native_tcp_##inst##_cfg, NATIVE_TCP_INIT_LEVEL, 55,             \
                  &native_tcp_uart_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NATIVE_TCP_INSTANCE);

#define INST_NAME(inst) DEVICE_DT_NAME(DT_DRV_INST(inst))

#define NATIVE_TCP_COMMAND_LINE_OPTS(inst)                                                 \
    {                                                                                      \
        .option = INST_NAME(inst) "_tcp_host",						                       \
        .name = "\"tcp_host\"",                                                            \
        .type = 's',                                                                       \
        .dest = &native_tcp_##inst##_data.cmd_tcp_host,                                    \
        .descript = "Set tcp host for " INST_NAME(inst) " uart device, "		           \
        "overriding " DT_INST_PROP(inst, tcp_host) "  in devicetree.",						                       \
    },                                                                                     \
    {											                                           \
        .option = INST_NAME(inst) "_tcp_port",						                       \
        .name = "tcp_port",								                                   \
        .type = 'u',									                                   \
        .dest = &native_tcp_##inst##_data.cmd_tcp_port,					                   \
        .descript = "Set the tcp port for " INST_NAME(inst) " device, overriding the "	   \
        "baudrate of " STRINGIFY(DT_INST_PROP(inst, tcp_port))			                   \
        " set in the devicetree.",							                               \
    },                                                                                     \
    {                                                                                      \
        .is_switch = true,                                                                 \
        .option = INST_NAME(inst) "_is_server",                                            \
        .type = 'b',                                                                       \
        .dest = &native_tcp_##inst##_data.cmd_is_server,                                   \
        .descript = "have the tcp connection act as a server"                              \
    },

static void native_tcp_add_serial_options(void)
{
    static struct args_struct_t opts[] = {
        DT_INST_FOREACH_STATUS_OKAY(NATIVE_TCP_COMMAND_LINE_OPTS) ARG_TABLE_ENDMARKER};

    native_add_command_line_opts(opts);
}

#define NATIVE_TCP_CLEANUP(inst)                                                                   \
    if (native_tcp_##inst##_data.fd != 0) {                                                    \
        native_tcp_close_bottom(native_tcp_##inst##_data.fd);                                       \
    }

/**
 * @brief Cleans up any open serial ports on the exit.
 */
static void native_tcp_cleanup_uart(void)
{
    DT_INST_FOREACH_STATUS_OKAY(NATIVE_TCP_CLEANUP);
}

NATIVE_TASK(native_tcp_add_serial_options, PRE_BOOT_1, 11);
NATIVE_TASK(native_tcp_cleanup_uart, ON_EXIT, 99);
