#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/version.h>
#include <linux/time.h>

#define MAJOR_NUM 100
#define IOCTL_WRITE _IOW(MAJOR_NUM, 0, int16_t*)
#define IOCTL_WRITE1 _IOW(MAJOR_NUM, 1, int16_t*)								//write user input to kernel

int16_t channel;
int16_t align;
static dev_t first;
char binaryNum[11] = "0000000000";
char input[17];
int j,i; 
uint16_t number;
	 													// variable for device number
static struct cdev c_dev; 											// variable for the character device structure
static struct class *cls; 	
static long adc_ioctl(struct file *file, unsigned int ioctl_cmd, unsigned long ioctl_param);												// variable for the device class

/*****************************************************************************
STEP 4 
my_close(), my_open(), my_read(), my_write() functions are defined here
these functions will be called for close, open, read and write system calls respectively. 
*****************************************************************************/

static int adc_open(struct inode *i, struct file *f)
{
	printk(KERN_INFO "ADC_Driver Opened...!! \n");
	return 0;
}

static int adc_close(struct inode *i, struct file *f)
{
	printk(KERN_INFO "ADC_Driver closed...!!\n");
	return 0;
}

static ssize_t adc_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
	printk(KERN_INFO "Reading 10 bit data of ADC\n");
	get_random_bytes(&number, sizeof(number));
   	number=(number%1024);
	printk(KERN_INFO"Random Number: %d\n",number);
	
	//decimal to binary
 	
	i = 0;

	while (number % 1024 != 0) 
	{ 
		// storing remainder in binary array

		binaryNum[9-i] = (number % 2) + '0'; 
		number = number / 2; 
		i++; 
    	}  

	if(align == 0)			//0-9 bits
	{
		for(j = 0;j < 10; j++)
		{
	          input[j] = binaryNum[j];
;		}
		
		for(j = 10;j < 16; j++)
		{
	          input[j] = 0 + '0';
		}
		
		input[16] = '\0';
		printk("Binary value = %s\n", binaryNum);
		printk("Aligned value = %s\n", input);

		if(copy_to_user(buf,input,sizeof(input)));
		return sizeof(input);
	}
	
	else if(align == 1)
	{	
		for(j = 6;j < 16; j++)
		{
	          input[j] = binaryNum[j-6];
;		}
		
		for(j = 0;j < 6; j++)
		{
	          input[j] = 0 + '0';
		}

		input[16] = '\0';
		printk("Binary value = %s\n", binaryNum);
		printk("Aligned value = %s\n", input);

		if(copy_to_user(buf,input,sizeof(input)));
		return sizeof(input);
	}
	
	else 
		return 0;
}
//##############################################################################//

//############################### IOCTL Function ###############################//

static long adc_ioctl(struct file *file, unsigned int ioctl_cmd, unsigned long ioctl_param)
{		
	switch(ioctl_cmd)
	{
	    case IOCTL_WRITE:		     
			     copy_from_user(&channel, (int16_t*) ioctl_param, sizeof(channel));
			     printk(KERN_INFO "Channel Selected = %d\n", channel);
			     break;
	    case IOCTL_WRITE1:		     
			     copy_from_user(&align, (int16_t*) ioctl_param, sizeof(align));
			     printk(KERN_INFO "Alignment Selected = %d\n", align);
			     break;
}
return 0;
}

//##############################################################################//
static struct file_operations fops =
				{
				  .owner 	  = THIS_MODULE,
				  .open 	  = adc_open,
				  .read 	  = adc_read,
				  .unlocked_ioctl = adc_ioctl,
				  .release 	  = adc_close,
				};

//########## INITIALIZATION FUNCTION ##################//
// STEP 1,2 & 3 are to be executed in this function ### // 

static int __init adc_driver_init(void) 
{
	printk(KERN_INFO "ADC driver registered");
	
	// STEP 1 : reserve <major, minor>

	if (alloc_chrdev_region(&first, 0, 1, "BITS-PILANI") < 0)			//Dynamically Allocating Major no.
	{
		printk(KERN_INFO "Cannot allocate major number \n");
		return -1;
	}
	
	// STEP 2 : dynamically create device node in /dev directory
    	
	if ((cls = class_create(THIS_MODULE, "chardrv")) == NULL)			//Creating struct class
	{
		printk(KERN_INFO "Cannot create struct class \n");
		unregister_chrdev_region(first, 1);
		return -1;
	}
    	
	if (device_create(cls, NULL, first, NULL, "adc8") == NULL)		//Creating Device
	{
		printk(KERN_INFO "Cannot create the device \n");
		class_destroy(cls);
		unregister_chrdev_region(first, 1);
		return -1;
	}
	
	// STEP 3 : Link fops and cdev to device node

    	cdev_init(&c_dev, &fops);							//Creating cdev structure
    
	if (cdev_add(&c_dev, first, 1) == -1)						//Adding character device to the system
	{
		printk(KERN_INFO "Cannot add the characterdevice to the system");
		device_destroy(cls, first);
		class_destroy(cls);
		unregister_chrdev_region(first, 1);
		return -1;
	}
	
	printk(KERN_INFO "ADC driver inserted successfully");
	return 0;
}
 
static void __exit adc_driver_exit(void) 
{
	cdev_del(&c_dev);
	device_destroy(cls, first);
	class_destroy(cls);
	unregister_chrdev_region(first, 1);
	printk(KERN_INFO "ADC driver unregistered\n\n");
}
 
module_init(adc_driver_init);
module_exit(adc_driver_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vandana Garg");
MODULE_DESCRIPTION("ADC Driver");
