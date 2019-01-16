#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>


MODULE_LICENSE("Dual BSD/GPL");

/* GPIO registers base address. */
#define BCM2708_PERI_BASE   (0x3F000000)
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)

/* GPIO Function Select 0. */
#define GPFSEL0 (GPIO_BASE + 0x00000000)
/* GPIO Function Select 1. */
#define GPFSEL1 (GPIO_BASE + 0x00000004)
/* GPIO Function Select 2. */
#define GPFSEL2 (GPIO_BASE + 0x00000008)
/* GPIO Function Select 3. */
#define GPFSEL3 (GPIO_BASE + 0x0000000C)
/* GPIO Function Select 4. */
#define GPFSEL4 (GPIO_BASE + 0x00000010)
/* GPIO Function Select 5. */
#define GPFSEL5 (GPIO_BASE + 0x00000014)
/* GPIO Pin Output Set 0. */
#define GPSET0 (GPIO_BASE + 0x0000001C)
/* GPIO Pin Output Set 1. */
#define GPSET1 (GPIO_BASE + 0x00000020)
/* GPIO Pin Output Clear 0. */
#define GPCLR0 (GPIO_BASE + 0x00000028)
/* GPIO Pin Output Clear 1. */
#define GPCLR1 (GPIO_BASE + 0x0000002C)
/* GPIO Pin Level 0. */
#define GPLEV0 (GPIO_BASE + 0x00000034)
/* GPIO Pin Level 1. */
#define GPLEV1 (GPIO_BASE + 0x00000038)
/* GPIO Pin Pull-up/down Enable. */
#define GPPUD (GPIO_BASE + 0x00000094)
/* GPIO Pull-up/down Clock Register 0. */
#define GPPUDCLK0 (GPIO_BASE + 0x00000098)
/* GPIO Pull-up/down Clock Register 1. */
#define GPPUDCLK1 (GPIO_BASE + 0x0000009C)


/* Broadcom Serial Controller 1 registers base address. */
#define BSC1_BASE (0x3F804000)

/* BSC1 Control Register */
#define BSC1_REG_C (BSC1_BASE + 0x00000000)
/* BSC1 Status Register */
#define BSC1_REG_S (BSC1_BASE + 0x00000004)
/* BSC1 Data Length Register */
#define BSC1_REG_DLEN (BSC1_BASE + 0x00000008)
/* BSC1 Slave Address Register */
#define BSC1_REG_A (BSC1_BASE + 0x0000000C)
/* BSC1 Data FIFO Register */
#define BSC1_REG_FIFO (BSC1_BASE + 0x00000010)
/* BSC1 Clock Divider Register */
#define BSC1_REG_DIV (BSC1_BASE + 0x00000014)
/* BSC1 Data Delay Register */
#define BSC1_REG_DEL (BSC1_BASE + 0x00000018)
/* BSC1 Clock Stretch Timeout Register */
#define BSC1_REG_CLKT (BSC1_BASE + 0x0000001C)

/* PUD - GPIO Pin Pull-up/down */
typedef enum {PULL_NONE = 0, PULL_DOWN = 1, PULL_UP = 2} PUD;

/* GPIO Alternative Function Assignment ALT0 (0100) ( GPIO2 set to SDA1, GPIO3 set to SCL1 ) */
#define GPIO_DIRECTION_ALT0 (4)

/* Definition of necessary GPIO pins */
#define GPIO_02 (2)
#define GPIO_03 (3)


/* Declaration of i2c_driver.c functions */
int i2c_driver_init(void);
void i2c_driver_exit(void);
static int i2c_driver_open(struct inode *, struct file *);
static int i2c_driver_release(struct inode *, struct file *);
static ssize_t i2c_driver_read(struct file *, char *buf, size_t , loff_t *);
static ssize_t i2c_driver_write(struct file *, const char *buf, size_t , loff_t *);

void unmapRegisters(void);
int SendData(unsigned int bytes);
int ReceiveData(unsigned int bytes);

/* Structure that declares the usual file access functions. */
struct file_operations i2c_driver_fops =
{
    open    :   i2c_driver_open,
    release :   i2c_driver_release,
    read    :   i2c_driver_read,
    write   :   i2c_driver_write
};

/* Declaration of the init and exit functions. */
module_init(i2c_driver_init);
module_exit(i2c_driver_exit);

/* Global variables of the driver */

/* Major number. */
int i2c_driver_major;

/* Buffer to store data. */
#define BUF_LEN 80
static char i2c_driver_buffer[BUF_LEN];
static char data_buffer[BUF_LEN];

/* Register addresses */
void *virt_reg_c;
void *virt_reg_s;
void *virt_reg_dlen;
void *virt_reg_a;
void *virt_reg_fifo;


/*
 * GetGPFSELReg function
 *  Parameters:
 *   pin    - number of GPIO pin;
 *
 *   return - GPFSELn offset from GPIO base address, for containing desired pin control
 *  Operation:
 *   Based on the passed GPIO pin number, finds the corresponding GPFSELn reg and
 *   returns its offset from GPIO base address.
 */
unsigned int GetGPFSELReg(char pin)
{
    unsigned int addr;

    if(pin >= 0 && pin <10)
        addr = GPFSEL0;
    else if(pin >= 10 && pin <20)
        addr = GPFSEL1;
    else if(pin >= 20 && pin <30)
        addr = GPFSEL2;
    else if(pin >= 30 && pin <40)
        addr = GPFSEL3;
    else if(pin >= 40 && pin <50)
        addr = GPFSEL4;
    else /*if(pin >= 50 && pin <53) */
        addr = GPFSEL5;

  return addr;
}

/*
 * GetGPIOPinOffset function
 *  Parameters:
 *   pin    - number of GPIO pin;
 *
 *   return - offset of the pin control bit, position in control registers
 *  Operation:
 *   Based on the passed GPIO pin number, finds the position of its control bit
 *   in corresponding control registers.
 */
char GetGPIOPinOffset(char pin)
{
    if(pin >= 0 && pin <10)
        pin = pin;
    else if(pin >= 10 && pin <20)
        pin -= 10;
    else if(pin >= 20 && pin <30)
        pin -= 20;
    else if(pin >= 30 && pin <40)
        pin -= 30;
    else if(pin >= 40 && pin <50)
        pin -= 40;
    else /*if(pin >= 50 && pin <53) */
        pin -= 50;

    return pin;
}

/*
 * SetInternalPullUpDown function
 *  Parameters:
 *   pin    - number of GPIO pin;
 *   pull   - set internal pull up/down/none if PULL_UP/PULL_DOWN/PULL_NONE selected
 *  Operation:
 *   Sets to use internal pull-up or pull-down resistor, or not to use it if pull-none
 *   selected for desired GPIO pin.
 */
void SetInternalPullUpDown(char pin, PUD pull)
{
    unsigned int gppud_base;
    unsigned int gppudclk_base;
    unsigned int tmp;
    unsigned int mask;
    void *addr = NULL;

    /* Get the base address of GPIO Pull-up/down Register (GPPUD). */
    gppud_base = GPPUD;

    /* Get the base address of GPIO Pull-up/down Clock Register (GPPUDCLK). */
    gppudclk_base = (pin < 32) ? GPPUDCLK0 : GPPUDCLK1;

    /* Get pin offset in register . */
    pin = (pin < 32) ? pin : pin - 32;

    /* Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-Down or neither
       to remove the current Pull-up/down). */
    /* A successful call to ioremap() returns a kernel virtual address corresponding to start
       of the requested physical address range. */
    addr = ioremap(gppud_base, 4);
    iowrite32(pull, addr);

    /* Wait 150 cycles – this provides the required set-up time for the control signal */

    /* Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to
       modify – NOTE only the pads which receive a clock will be modified, all others will
       retain their previous state. */
    addr = ioremap(gppudclk_base, 4);
    tmp = ioread32(addr);
    mask = 0x1 << pin;
    tmp |= mask;
    iowrite32(tmp, addr);

    /* Wait 150 cycles – this provides the required hold time for the control signal */

    /* Write to GPPUD to remove the control signal. */
    addr = ioremap(gppud_base, 4);
    iowrite32(PULL_NONE, addr);

    /* Write to GPPUDCLK0/1 to remove the clock. */
    addr = ioremap(gppudclk_base, 4);
    tmp = ioread32(addr);
    mask = 0x1 << pin;
    tmp &= (~mask);
    iowrite32(tmp, addr);
}

/*
 * SetGpioPinDirection function
 *  Parameters:
 *   pin       - number of GPIO pin;
 *   direction - GPIO_DIRECTION_ALT0 ( 4 decimal or 100 binary )
 *  Operation:
 *   Sets the desired GPIO pin to be used as it's alternative function indicates
 */
void SetGpioPinDirection(char pin, char function)
{
    unsigned int GPFSELReg_base;
    unsigned int tmp;
    unsigned int mask;
    void *addr = NULL;

    /* Get base address of function selection register. */
    GPFSELReg_base = GetGPFSELReg(pin);

    /* Calculate gpio pin offset. */
    pin = GetGPIOPinOffset(pin);

    /* Set gpio pin direction. */
    addr = ioremap(GPFSELReg_base, 4);
    tmp = ioread32(addr);
    
  	mask = ~(0b111 << (pin*3));
  	tmp &= mask;
  	
  	mask = (function & 0b111) << (pin*3);
    tmp |= mask;
  	
    iowrite32(tmp, addr);
}


/*
 * Initialization:
 *  1. Register device driver
 *  2. Allocate memory
 *	3. Initialize buffers
 *  4. Map BSC1 Physical address space to virtual address
 *  5. Initialize GPIO pins
 */
int i2c_driver_init(void)
{
    int result;

    printk(KERN_INFO "Inserting i2c_driver module\n");

    /* Registering device. */
    result = register_chrdev(0, "i2c_driver", &i2c_driver_fops);
    if (result < 0)
    {
        printk(KERN_INFO "i2c_driver: cannot obtain major number %d\n", i2c_driver_major);
        return result;
    }

    i2c_driver_major = result;
    printk(KERN_INFO "i2c_driver major number is %d\n", i2c_driver_major);

    /* Allocating memory. */
    if (request_mem_region(BSC1_BASE, 0x20, "ic2_driver") == NULL)
    {
        printk(KERN_INFO "Unable to obtain BSC1 memory address\n");
		return -1;
    }
    
    /* Initialize data buffers. */
    memset(i2c_driver_buffer, 0, BUF_LEN);
    memset(data_buffer, 0, BUF_LEN);
    
	/* Map BSC1 Physical address space to virtual address. */
	virt_reg_c = ioremap(BSC1_REG_C, 4);
	if(!virt_reg_c)
    {
        result = -ENOMEM;
        goto fail_no_virt_mem;
    }
    
	virt_reg_s = ioremap(BSC1_REG_S, 4);
	if(!virt_reg_s)
    {
        result = -ENOMEM;
        goto fail_no_virt_mem;
    }
    
	virt_reg_dlen = ioremap(BSC1_REG_DLEN, 4);
	if(!virt_reg_dlen)
    {
        result = -ENOMEM;
        goto fail_no_virt_mem;
    }
    
	virt_reg_a = ioremap(BSC1_REG_A, 4);
	if(!virt_reg_a)
    {
        result = -ENOMEM;
        goto fail_no_virt_mem;
    }
    
	virt_reg_fifo = ioremap(BSC1_REG_FIFO, 4);
	if(!virt_reg_fifo)
    {
        result = -ENOMEM;
        goto fail_no_virt_mem;
    }

	/* Setting pull up, and pin alt functions. */
	SetGpioPinDirection(GPIO_02, GPIO_DIRECTION_ALT0);
	SetGpioPinDirection(GPIO_03, GPIO_DIRECTION_ALT0);
	SetInternalPullUpDown(GPIO_02, PULL_UP);
	SetInternalPullUpDown(GPIO_03, PULL_UP);


    return 0;
    
fail_no_virt_mem:
	/* Unmap GPIO Physical address space. */
	unmapRegisters();
	
	/* Freeing the major number. */
	unregister_chrdev(i2c_driver_major, "i2c_driver");
	
	return result;
}


/*
 * SendData i2c serial communication function
 *  Parameters:
 *   bytes 		- number of bytes to be transfered
 *  Operation:
 *   Sends data recieved from Application to Device.
 *   Returns 0 on succesful send, -1 when an error has been encountered.
 */
int SendData(unsigned int bytes)
{
	int i;
	int status;

	/* Clear the Status register so we can get an accurate reading after completion of transfer. */
	iowrite32(0x00000302, virt_reg_s); 	// 0b 0000 0000 0000 0000 0000 0011 0000 0010
	
	/* Setup Control register for sending data, clear FIFO before writing. */
	iowrite32(0x00008110, virt_reg_c); 	// 0b 0000 0000 0000 0000 1000 0001 0001 0000
									  	// bit 15 enables I2C, bit 8 sets generate interrupt on DONE,
									  	// bit 4 clears FIFO, bit 0 sets write
									  
  	/* Write to DLEN register the number of bytes to be sent. */
  	iowrite32(bytes, virt_reg_dlen);
									  
	/* Write to FIFO register from data buffer. */
	for(i = 0; i < bytes; i++)
	{
		iowrite32(data_buffer[i], virt_reg_fifo); 
	} 
	
	/* Start transfer by setting 8th bit in Control register to 1. */
	iowrite32(0x00008180, virt_reg_c); 	// 0b 0000 0000 0000 0000 1000 0001 1000 0000
	
	/* Reading from the Status register and checking if the transfer is complete. */
	status = ioread32(virt_reg_s);
	
	while(!(status & 0x00000002)) 		// Once the transfer is complete the 2nd bit will be set to 1 (0010 or 0x2 in hex).
	{							  		// While loop prevents further progress until the transfer is finished.
		status = ioread32(virt_reg_s);	// Keep reading status register until transfer is complete.
		
		/*
										// OPTION A FOR INTERRUPT
		if(!(status & 0x00000001))		// First bit indicates if there is a transfer in progress.
		{								// If the transfer is not complete, and there is no transfer in progress,
			return -1;					// the while loop is interrupted and we return a -1 to signify an error.
		}
		*/
		/*
										// OPTION B FOR INTERRUPT
		if(status & 0x00000200)			// Slave has held the SCL signal low (clock stretching)
		{								// for longer and that specified in the I2CCLKT register. [sic]
			return -1;					// The while loop is interrupted and we return a -1 to signify an error.
		}
		*/
	}
	
	return 0;								  
}


/*
 * RecieveData i2c serial communication function
 *  Parameters:
 *   bytes 		- number of bytes to be transfered
 *  Operation:
 *   Recieves data from Device and writes to data buffer.
 */
int ReceiveData(unsigned int bytes)
{
	int i;
	int status;
	int data;
	
	memset(i2c_driver_buffer, '\0', BUF_LEN);
	
	/* Clear the Status register so we can get an accurate reading after completion of transfer. */
	iowrite32(0x00000302, virt_reg_s); 	// 0b 0000 0000 0000 0000 0000 0011 0000 0010
	
	/* Setup Control register for recieving data, clear FIFO before writing.  */
	iowrite32(0x00008111, virt_reg_c);	// 0b 0000 0000 0000 0000 1000 0001 0001 0001
										// bit 15 enables I2C, bit 8 sets generate interrupt on DONE,
										// bit 5 clears FIFO, bit 0 sets read
	
	/* Write to DLEN register the number of bytes to be recieved. */
  	iowrite32(bytes, virt_reg_dlen);
  	
  	/* Start transfer by setting 8th bit in Control register to 1. */
	iowrite32(0x00008181, virt_reg_c); 	// 0b 0000 0000 0000 0000 1000 0001 1000 0001
	
	/* Reading from the Status register and checking if the transfer is complete. */
	status = ioread32(virt_reg_s);
	
	while(!(status & 0x00000002)) 		// Once the transfer is complete the 2nd bit will be set to 1 (0010 or 2 in hex).
	{							  		// While loop prevents further progress until the transfer is finished.
		status = ioread32(virt_reg_s);	// Keep reading status register until transfer is complete.
		/*
										// OPTION A FOR INTERRUPT
		if(!(status & 0x00000001))		// First bit indicates if there is a transfer in progress.
		{								// If the transfer is not complete, and there is no transfer in progress,
			return -1;					// the while loop is interrupted and we return a -1 to signify an error.
		}
		*/
		/*
										// OPTION B FOR INTERRUPT
		if(status & 0x00000200)			// Slave has held the SCL signal low (clock stretching) (?)
		{								// for longer and that specified in the I2CCLKT register. [sic]
			return -1;					// The while loop is interrupted and we return a -1 to signify an error.
		}
		*/
	}
	
	i = 0;
	
	/* Read data from FIFO until it's empty or we have reached the expected number of bytes */
	while(!(status & 0x00000040)) 		// Once the FIFO register is empty the 7th bit will be set to 1 (0100 0000 or 0x40 in hex).
	{
		data = ioread32(virt_reg_fifo);
		i2c_driver_buffer[i] = data;
		++i;
		if( i == bytes)
		{
			break;
		}
		status = ioread32(virt_reg_s);
	}
	
	return 0;
}


/*
 * Cleanup:
 *  1. Release GPIO pins (set pull-none to minimize the power consumption)
 *  2. Free memory
 *  3. Unregister device driver
 */
void i2c_driver_exit(void)
{
    printk(KERN_INFO "Removing i2c_driver module\n");

    /* Set pull to none. */
	SetInternalPullUpDown(GPIO_02, PULL_NONE);
	SetInternalPullUpDown(GPIO_03, PULL_NONE);
	
    /* Unmap i2c Physical address space. */
    unmapRegisters();
	
	/* Free memory */
	release_mem_region(BSC1_BASE, 0x20);

    /* Freeing the major number. */
    unregister_chrdev(i2c_driver_major, "i2c_driver");
}


/* File open function. */
static int i2c_driver_open(struct inode *inode, struct file *filp)
{
    return 0;
}


/* File close function. */
static int i2c_driver_release(struct inode *inode, struct file *filp)
{
    return 0;
}


/*
 * File read function
 *  Parameters:
 *   filp  - a type file structure;
 *   buf   - a buffer, from which the user space function (fread) will read;
 *   len - a counter with the number of bytes to transfer, which has the same
 *           value as the usual counter in the user space function (fread);
 *   f_pos - a position of where to start reading the file;
 *  Operation:
 *   The i2c_driver_read function transfers data from the driver buffer (i2c_driver_buffer)
 *   to user space with the function copy_to_user.
 */
static ssize_t i2c_driver_read(struct file *filp, char *buf, size_t len, loff_t *f_pos)
{
    /* Size of valid data in i2c_driver - data to send in user space. */
    int data_size = 0;

    if (*f_pos == 0)
    {
        /* Get size of valid data. */
        data_size = strlen(i2c_driver_buffer);

        /* Send data to user space. */
        if (copy_to_user(buf, i2c_driver_buffer, data_size) != 0)
        {
            return -EFAULT;
        }
        else
        {
            (*f_pos) += data_size;

            return data_size;
        }
    }
    else
    {
        return 0;
    }
}


/*
 * File write function
 *  Parameters:
 *   filp  - a type file structure;
 *   buf   - a buffer in which the user space function (fwrite) will write;
 *   len - a counter with the number of bytes to transfer, which has the same
 *           values as the usual counter in the user space function (fwrite);
 *   f_pos - a position of where to start writing in the file;
 *  Operation:
 *   The function copy_from_user transfers the data from user space to kernel space.
 *   Function reads from user space into the i2c_driver_buffer and depending on the
 *   first element executes 3 kinds of operations, or calls an interupt if an error
 *   has occured. The second element is the number of elements to be read after the
 *   first two.
 */
static ssize_t i2c_driver_write(struct file *filp, const char *buf, size_t len, loff_t *f_pos)
{
	unsigned int temp;
	int n, i;
	
	if (copy_from_user(i2c_driver_buffer, buf, len) != 0)
	{
		return -1;
	} 
	else
	{
		/*If A is the first element in the buffer, driver is expecting Slave address from the application. */
		/*If S is the first element in the buffer, the driver is expected to send buffer information to device. */
		/*If R is the first element in the buffer, the driver is expected to recieve information from device. */
		if (i2c_driver_buffer[0] == 'A')
		{
			temp = i2c_driver_buffer[1];		// The second element is the Slave address.
			iowrite32(temp, virt_reg_a);		// Writing the Slave address into the virtual Slave register of the BSC.
			temp = ioread32(virt_reg_a);		// Reading the Slave address from the register to confirm success.
			printk(KERN_ALERT "Slave Address is set to: %x\n", temp);	
		}
		else if (i2c_driver_buffer[0] == 'S')
		{
			n = (int)i2c_driver_buffer[1];		// Read number of bytes to send to device.

			for(i = 2; i < 2+n; i++)			// Shifted for 2 because the first two bytes are informational.
			{									
				data_buffer[i - 2] = i2c_driver_buffer[i];
			}

			if(SendData((unsigned int)n) < 0)
			{
				printk(KERN_ALERT "Driver failed to send data to device!\n");
			}
		}
		else if (i2c_driver_buffer[0] == 'R')
		{

			n = (int)i2c_driver_buffer[1];		// Read number of bytes to recieve from device.
			
			if(ReceiveData((unsigned int)n) < 0)
			{
				printk(KERN_ALERT "Driver failed to recieve data from device!\n");	
				i2c_driver_buffer[0] = 'E';		// Send interrupt case to application.
			}
		}
		return len;
	}
}

void unmapRegisters()
{
	if (virt_reg_c)
    {
        iounmap(virt_reg_c);
    }
    
    if (virt_reg_s)
    {
        iounmap(virt_reg_s);
    }
    
    if (virt_reg_dlen)
    {
        iounmap(virt_reg_dlen);
    }
    
    if (virt_reg_a)
    {
        iounmap(virt_reg_a);
    }
    
    if (virt_reg_fifo)
    {
        iounmap(virt_reg_fifo);
    }
}
