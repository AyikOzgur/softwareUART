#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>

#define UART_CONFIG _IOW('U', 1, UARTConfig)

/* @brief This struct will be available from userspace too.
 */
typedef struct
{
    int txPin;
    int rxPin;
    int baudRate;

    // These are gonna be implemented coming version. 
    // Current version params: 8N1
    int dataBits;
    int stopBits;
    int parity;
    char isInverted;
} UARTConfig;

UARTConfig uart_params;
static struct hrtimer tx_hrtimer;
static struct hrtimer rx_hrtimer;
static char tx_buffer[256];
static int tx_buffer_pos = 0;
static int tx_buffer_len = 0;

static int rx_gpio_irq;
static char rx_buffer[256];
static int rx_buffer_pos = 0;
static int bit_duration = 0;
static int bit_pos = 0;

static enum hrtimer_restart tx_hrtimer_handler(struct hrtimer *timer);
static enum hrtimer_restart rx_hrtimer_handler(struct hrtimer *timer);
static irqreturn_t rx_irq_handler(int irq, void *dev_id);

int initPeripherals(UARTConfig *uart_params)
{
    // Initialize GPIOs based on the received parameters
    if (gpio_request(uart_params->txPin, "GPIO_TX"))
    {
        return -EBUSY;
    }
    gpio_direction_output(uart_params->txPin, 1);

    if (gpio_request(uart_params->rxPin, "GPIO_RX"))
    {
        gpio_free(uart_params->txPin);
        return -EBUSY;
    }
    gpio_direction_input(uart_params->rxPin);

    // Initialize high-resolution timers
    hrtimer_init(&tx_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    tx_hrtimer.function = tx_hrtimer_handler;
    
    hrtimer_init(&rx_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    rx_hrtimer.function = rx_hrtimer_handler;

    // Request IRQ for RX
    rx_gpio_irq = gpio_to_irq(uart_params->rxPin);
    if (request_irq(rx_gpio_irq, rx_irq_handler, IRQF_TRIGGER_FALLING, "soft_uart_rx", NULL))
    {
        gpio_free(uart_params->txPin);
        gpio_free(uart_params->rxPin);
        return -1;
    }

    // Required for rx timer intruupt for proper sampling.
    bit_duration = 1000000 / uart_params->baudRate;
    return 0;
}

static enum hrtimer_restart tx_hrtimer_handler(struct hrtimer *timer)
{
    static int bit_pos = 0;
    static char current_byte;

    if (tx_buffer_pos >= tx_buffer_len)
    {
        // No more data to send
        return HRTIMER_NORESTART;
    }

    if (bit_pos == 0)
    {
        // Start bit
        gpio_set_value(uart_params.txPin, 0);
        current_byte = tx_buffer[tx_buffer_pos];
    }
    else if (bit_pos <= 8)
    {
        // Data bit not inverted.
        gpio_set_value(uart_params.txPin, (current_byte >> (8 - bit_pos)) & 0x01);
    }
    else if (bit_pos == 9)
    {
        // Stop bit
        gpio_set_value(uart_params.txPin, 0);
        bit_pos = -1;
        tx_buffer_pos++;
    }

    bit_pos++;
    hrtimer_forward_now(&tx_hrtimer, ktime_set(0, bit_duration * 1000)); // bit_duration in microseconds to nanoseconds
    return HRTIMER_RESTART;
}

static void start_tx(void)
{
    tx_buffer_pos = 0;
    if (tx_buffer_len > 0)
    {
        hrtimer_start(&tx_hrtimer, ktime_set(0, bit_duration * 1000), HRTIMER_MODE_REL);
    }
}

static irqreturn_t rx_irq_handler(int irq, void *dev_id)
{
    // Start bit detected.
    bit_pos = 0;
    hrtimer_start(&rx_hrtimer, ktime_set(0, bit_duration * 1000), HRTIMER_MODE_REL);
    disable_irq_nosync(rx_gpio_irq); // Disable gpio interrupt to avoid multiple triggers
    return IRQ_HANDLED;
}

static enum hrtimer_restart rx_hrtimer_handler(struct hrtimer *timer)
{
    static char current_byte = 0;

    if (bit_pos <= 8)
    {
        // Read data bits (inverted logic)
        current_byte >>= 1;
        if (gpio_get_value(uart_params.rxPin))
        {
            // Set the related bit.
            current_byte |= 0x80;
        }
        else 
        {
            // Clear the related bit.
            current_byte &= 0x7F;
        }

        hrtimer_forward_now(&rx_hrtimer, ktime_set(0, bit_duration * 1000));
    }
    else if (bit_pos == 9)
    {
        // Stop bit (inverted logic)
        rx_buffer[rx_buffer_pos++] = current_byte;
        current_byte = 0;
        enable_irq(rx_gpio_irq); // Re-enable GPIO interrupt for the next byte
    }

    bit_pos++;
    return HRTIMER_RESTART;
}

static int open(struct inode *inode, struct file *file)
{
    return 0; // No need to any operation for now.
}

static int close(struct inode *inode, struct file *file)
{
    return 0; // No need to any operation for now.
}

static long ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
    case UART_CONFIG:

        int bytes = copy_from_user(&uart_params, (UARTConfig *)arg, sizeof(UARTConfig));
        if (bytes < 0)
        {
            return -EFAULT;
        }

        pr_info("TX Pin: %d\n", uart_params.txPin);
        pr_info("RX Pin: %d\n", uart_params.rxPin);
        pr_info("Baud Rate: %d\n", uart_params.baudRate);
        pr_info("(Not implemented) Data Bits: %d\n", uart_params.dataBits);
        pr_info("(Not implemented) Stop Bits: %d\n", uart_params.stopBits);
        pr_info("(Not implemented) Parity: %d\n", uart_params.parity);
        pr_info("(Not implemented) Inverted: %c\n", uart_params.isInverted);

        if (initPeripherals(&uart_params) < 0)
        {
            return -1;
        }
        break;
    default:
        return -ENOTTY;
    }

    return 0;
}

static ssize_t read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    // Check if there is data to read
    if (rx_buffer_pos == 0)
        return 0; // No data available

    // Determine the number of bytes to read
    int bytes_to_read = min(count, (size_t)rx_buffer_pos);

    // Copy data to userspace
    if (copy_to_user(buf, rx_buffer, bytes_to_read))
        return -EFAULT;

    // Shift remaining data in the buffer
    memmove(rx_buffer, rx_buffer + bytes_to_read, rx_buffer_pos - bytes_to_read);
    rx_buffer_pos -= bytes_to_read;

    return bytes_to_read;
}

static ssize_t write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    // Check if there is enough space in the buffer
    if (count > sizeof(tx_buffer))
        return -ENOMEM; // Not enough space

    // Copy data from userspace
    if (copy_from_user(tx_buffer, buf, count))
        return -EFAULT;

    // Set the buffer length and start transmission
    tx_buffer_len = count;
    start_tx();

    return count;
}

static const struct file_operations miscDeviceFileOperations =
{
    .owner = THIS_MODULE,
    .open = open,
    .release = close,
    .unlocked_ioctl = ioctl,
    .read = read,
    .write = write,
};

static struct miscdevice miscDevice =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "softwareUART",
    .fops = &miscDeviceFileOperations,
};

static int __init init(void)
{
    // Register device with kernel.
    int ret = misc_register(&miscDevice);
    if (ret != 0)
    {
        pr_err("Could not register misc device : %s\n", miscDevice.name);
        return ret;
    }

    // Device info.
    pr_info("%s device got minor %d\n", miscDevice.name, miscDevice.minor);

    return 0;
}

static void __exit customExit(void)
{
    // Unregister device from kernel
    misc_deregister(&miscDevice);
    free_irq(rx_gpio_irq, NULL);
    gpio_free(uart_params.txPin);
    gpio_free(uart_params.rxPin);
    hrtimer_cancel(&tx_hrtimer);
    hrtimer_cancel(&rx_hrtimer);
}

module_init(init);
module_exit(customExit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ahmet Ozgur Ayik");
MODULE_DESCRIPTION("Software UART Device Module");