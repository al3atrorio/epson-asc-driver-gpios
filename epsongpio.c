//modified from Derek Molloy's examples
//see http://www.derekmolloy.ie/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/time.h>		//for time stamping interrupt
#include <linux/interrupt.h>
#include <linux/gpio.h>

//char driver related
#define  DEVICE_NAME "epsongpios"    // /dev/epsongpios
#define  CLASS_NAME  "epsonclass"    //

static int    majorNumber;                  // Stores the device number
static int    numberOpens = 0;              // Counts the number of times the device is opened
static struct class*  epsongpioClass  = NULL; // The device-driver class struct pointer
static struct device* epsongpioDevice = NULL; // The device-driver device struct pointer
static struct fasync_struct *gpio_irq_async_queue;

static unsigned int current_port;
static unsigned int current_value;

// must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static int     dev_fasync(int fd, struct file *filp, int on);
static irq_handler_t GPIOlkm_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

static struct file_operations fops =
{
  .open = dev_open,
  .read = dev_read,
  .write = dev_write,
  .release = dev_release,
  .fasync	= dev_fasync,
};

typedef struct Gpio_in {
  unsigned int gpio_num;
  unsigned int irq_num;
} Gpio_in;

typedef struct read_struct {
    unsigned int port;
    unsigned int value;
} read_struct;

typedef struct write_struct {
    unsigned int port;
    unsigned int value;
} write_struct;

#define NUMBER_OF_IN_GPIOS 9
#define NUMBER_OF_OUT_GPIOS 6

//We have 6 "out" gpios and 9 "in" gpios, so the index of the array belows are these ports.
//So we need to map the port number with the gpios hardware number.
static Gpio_in gpio_in[NUMBER_OF_IN_GPIOS] = {{0, 0},{1, 0},{2, 0},{3, 0},{4, 0},{5, 0},{6, 0},{7, 0},{8, 0}}; //IN are pins PA0-8
static unsigned int gpio_out[NUMBER_OF_OUT_GPIOS] = {9,11,13,10,12,14}; //OUT are pins PA9-14


static int __init epsongpios_init(void) {
  int result = 0;
  int i = 0;

  printk(KERN_INFO "epsongpios: init Linux Kernel Module.\n");

  // Try to dynamically allocate a major number for the device -- more difficult but worth it
  majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
  if (majorNumber<0) {
    printk(KERN_ALERT "epsongpios: failed to register a major number\n");
    return majorNumber;
  }
  printk(KERN_INFO "epsongpios: registered correctly with major number %d\n", majorNumber);

  // Register the device class
  epsongpioClass = class_create(THIS_MODULE, CLASS_NAME);
  if (IS_ERR(epsongpioClass)){                // Check for error and clean up if there is
    unregister_chrdev(majorNumber, DEVICE_NAME);
    printk(KERN_ALERT "Failed to register device class\n");
    return PTR_ERR(epsongpioClass);          // Correct way to return an error on a pointer
  }
  printk(KERN_INFO "epsongpios: device class registered correctly\n");

  // Register the device driver
  epsongpioDevice = device_create(epsongpioClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
  if (IS_ERR(epsongpioDevice)) {               // Clean up if there is an error
    class_destroy(epsongpioClass);           // Repeated code but the alternative is goto statements
    unregister_chrdev(majorNumber, DEVICE_NAME);
    printk(KERN_ALERT "Failed to create the device\n");
    return PTR_ERR(epsongpioDevice);
  }
  printk(KERN_INFO "epsongpios: device class created correctly\n");

  printk(KERN_INFO "epsongpios: requesting GPIO and IRQ number\n");

  for (i=0; i<NUMBER_OF_OUT_GPIOS; i++) {
    gpio_request(gpio_out[i], "sysfs"); 	  //wow. very familar
    gpio_direction_output(gpio_out[i], false); //set output to 0
    gpio_export(gpio_out[i], false);	  //false = no change in direction
  }

  for (i=0; i<NUMBER_OF_IN_GPIOS; i++) {
    Gpio_in *gpio = &gpio_in[i];

    gpio_request(gpio->gpio_num, "sysfs");
    gpio_direction_input(gpio->gpio_num);
    gpio_set_debounce(gpio->gpio_num, 500);	 //500ms debounce delay
    gpio_export(gpio->gpio_num, false);

    gpio->irq_num = gpio_to_irq(gpio->gpio_num);
    printk(KERN_INFO "epsongpios: %d irq is : %d\n", gpio->irq_num, gpio->irq_num);

    result = request_irq(gpio->irq_num, (irq_handler_t) GPIOlkm_irq_handler,
                      IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, 
                      "GPIOlkm_handler", NULL);  // /proc/interrupts

    printk(KERN_INFO "epsongpios: Interrupt assignment : %d\n", result);
  }

  return result;
}

//releasing all gpio stuff
static void __exit epsongpios_exit(void) {   
  int i = 0;

  for (i=0; i<NUMBER_OF_OUT_GPIOS; i++) {
    printk(KERN_INFO "epsongpios: releasing out gpio : %d\n", gpio_out[i]);

    gpio_unexport(gpio_out[i]);
    gpio_free(gpio_out[i]);
  } 

  for (i=0; i<NUMBER_OF_IN_GPIOS; i++) {
    Gpio_in *gpio = &gpio_in[i];

    printk(KERN_INFO "epsongpios: releasing in gpio : %d\n", gpio->gpio_num);

    free_irq(gpio->irq_num, NULL);	//release it
    gpio_unexport(gpio->gpio_num);   
    gpio_free(gpio->gpio_num);
  }

  //unregister char device driver
  printk(KERN_INFO "epsongpios: unregistering char device driver\n");
  device_destroy(epsongpioClass, MKDEV(majorNumber, 0));     // remove the device
  class_unregister(epsongpioClass);                          // unregister the device class
  class_destroy(epsongpioClass);                             // remove the device class
  unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number

  printk(KERN_INFO "epsongpios: Goodbye!\n");
}

static int dev_open(struct inode *inodep, struct file *filep){
   numberOpens++;
   printk(KERN_INFO "epsongpios: Device has been opened %d time(s)\n", numberOpens);
   return 0;
}

static int dev_fasync(int fd, struct file *filp, int on) {
	return fasync_helper(fd, filp, on, &gpio_irq_async_queue);
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
  read_struct rd;
  int error_count = 0;

  rd.port = current_port;
  rd.value = current_value;

  error_count = copy_to_user(buffer, &rd, sizeof(read_struct));

  if (error_count==0) {
    return sizeof(read_struct);
  }
  else {
    printk(KERN_INFO "epsongpios: Failed to send to the user\n");
    return -EFAULT;
  }
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
  write_struct wr;

  printk(KERN_INFO "epsongpios: Start the writing\n");
   
  /* write data to the buffer */
  if (copy_from_user(&wr, buffer, sizeof(write_struct))) {
    return -EFAULT;
  }

  printk(KERN_INFO "epsongpios: port: %d\n\n", wr.port);
  printk(KERN_INFO "epsongpios: value: %d\n", wr.value);

  if (wr.port >= NUMBER_OF_OUT_GPIOS) {
    printk(KERN_INFO "epsongpios: Invalid Port. Droping\n");
    return -EFAULT;
  }

  wr.value = wr.value?1:0;

  gpio_set_value(gpio_out[wr.port], wr.value);
  
  return sizeof(write_struct);
}

static int dev_release(struct inode *inodep, struct file *filep){
  printk(KERN_INFO "epsongpios: Device successfully closed\n");
  return 0;
}

static int irq_to_port(unsigned int irq) {
  int i;

  for (i=0; i<NUMBER_OF_IN_GPIOS; i++) {
    if (gpio_in[i].irq_num == irq) {
      return i;
    }
  }
  return -1;
}

static irq_handler_t GPIOlkm_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
  printk(KERN_INFO "GPIOlkm Epson: IRQ.\n");

  int port = irq_to_port(irq);

  if (port < 0) {
    printk(KERN_INFO "epsongpios: Invalid Irq number. Discarding.\n");
    return (irq_handler_t) IRQ_HANDLED;
  }

  current_port = port;
  current_value = gpio_get_value(gpio_in[port].gpio_num);

  kill_fasync(&gpio_irq_async_queue, SIGIO, POLL_IN);

  return (irq_handler_t) IRQ_HANDLED;
}

module_init(epsongpios_init);
module_exit(epsongpios_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("gpio driver");

