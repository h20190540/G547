#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/usb.h>		//handles all usb functionality in kernel
#include<linux/timer.h>
#include<linux/slab.h>

#define be_to_int32(buf) (((buf)[0]<<24)|((buf)[1]<<16)|((buf)[2]<<8)|(buf)[3])
#define HP_VID 0x03f0		//pendrive by hp
#define HP_PID 0x7F40
#define SAN_VID 0x0781		//pendrive by sandisk
#define SAN_PID 0x5567
#define READ_CAPACITY_LENGTH          0x08
#define RETRY_MAX                     5

enum error {
    SUCCESS             =  0,
    ERROR_PIPE          = -9,
    ENDPOINT_OUT	    = 0x00,
    ENDPOINT_IN	    = 0x80,
    BOMS_RESET		    = 0xff,
    BOMS_RESET_REQ_TYPE = 0x21,
};

//**********************************************************************************************************//

int send_mass_storage_command(struct usb_device*, uint8_t, uint8_t,uint8_t*, uint8_t, uint8_t, uint32_t*);
int read_capacity(struct usb_device*,uint8_t, uint8_t);
int get_mass_storage_status(struct usb_device*, uint8_t, uint8_t);

//Command Block Wrapper (CBW)
struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

//Command Status Wrapper (CSW)
struct command_status_wrapper {
	uint8_t dCSWSignature[4];
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
};

static uint8_t cdb_length[256] = {
//	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  0
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  1
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  2
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  3
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  4
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  5
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  6
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  7
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  8
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  9
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  A
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  B
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  C
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  D
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  E
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  F
};

//********************************************************************************************************//

//*****************************************read_capacity*************************************************//

int read_capacity(struct usb_device *device, uint8_t endpoint_in,uint8_t endpoint_out)
{	
	uint8_t *buffer = NULL;		//Allocate memory for buffer
	int size = 0 ,read_ep,r;
	uint8_t lun = 0;
	uint32_t tag;
	uint8_t cdb[16];			// SCSI Command Descriptor Block
	long max_lba, block_size, device_size = 100;

	printk("\nReading Capacity:\n");
	//memset(buffer, 0, sizeof(uint8_t));
	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x25;				// Read Capacity
	
	if ( !(buffer = (uint8_t *)kmalloc(sizeof(uint8_t)*64, GFP_KERNEL)) ) {
		printk(KERN_INFO"Cannot allocate memory for rcv buffer\n");
		return -1;
	}

	//Resetting the usb_device
	r = usb_control_msg(device,usb_sndctrlpipe(device,0),BOMS_RESET,BOMS_RESET_REQ_TYPE,0,0,NULL,0,1000);
	if(r < 0)
		printk(KERN_INFO "Cannot reset\n");
	else
		printk(KERN_INFO "Reset has been done\n");

	//usb_reset_device(device);			//This fnc can also be used to reset the device

	if(send_mass_storage_command(device, endpoint_out, lun, cdb,ENDPOINT_IN, READ_CAPACITY_LENGTH, &tag) != 0 ) 
	{
		printk(KERN_INFO "Send command error\n");
		return -1;
	}

	//Sending the command to receive data
	read_ep = usb_bulk_msg(device,usb_rcvbulkpipe(device,endpoint_in),(unsigned char*)buffer, READ_CAPACITY_LENGTH, &size, 1000);
	printk(KERN_INFO "read_ep = %d\n",read_ep);
	if (read_ep != 0) 
	{
		printk(KERN_INFO"Reading endpoint command error\n");
		return -1;
	}
	
	printk("Received %d bytes\n", size);

	max_lba = be_to_int32(buffer);
	block_size = be_to_int32(buffer + 4);
	device_size = ((max_lba+1))*block_size/(1024*1024*1024);		//calculate device size
	printk(KERN_INFO "Device Size: %ld GB\n",device_size);

	kfree(buffer);											//free allocating memory

	if ( get_mass_storage_status(device, endpoint_in,tag) == -1 )
	{
		printk(KERN_INFO "cannot get command status block\n");
		return -1;
	}
	
	return 0;
}

//********************************************************************************************************//

//**********************************send_mass_storage_command********************************************//

int send_mass_storage_command(struct usb_device *device, uint8_t endpoint_out, uint8_t lun,uint8_t *cdb, uint8_t direction, uint8_t data_length, uint32_t *ret_tag)
{
	static uint32_t tag = 1;
	uint8_t cdb_len;
	int i, r = 0, size = 0;
	struct command_block_wrapper *cbw; 

	cbw = (struct command_block_wrapper*)kmalloc(sizeof(struct command_block_wrapper), GFP_KERNEL);
	if ( cbw == NULL ) {
		printk(KERN_INFO "Cannot allocate memory\n");
		return -1;
	}

	if (cdb == NULL) 
	{
		return -1;
	}

	if (endpoint_out & ENDPOINT_IN) 
	{
		printk(KERN_INFO "send_mass_storage_command: cannot send command on IN endpoint\n") ;
		return -1;
	}

	cdb_len = cdb_length[cdb[0]];

	if ((cdb_len == 0) || (cdb_len > sizeof(cbw -> CBWCB))) 
	{
		printk(KERN_INFO "send_mass_storage_command: don't know how to handle this command (%02X, length %d)\n", cdb[0], cdb_len);
		return -1;
	}

	memset(cbw,'\0',sizeof(struct command_block_wrapper)); 
	cbw->dCBWSignature[0] = 'U';
	cbw->dCBWSignature[1] = 'S';
	cbw->dCBWSignature[2] = 'B';
	cbw->dCBWSignature[3] = 'C';
	*ret_tag = tag;
	cbw->dCBWTag = tag++;
	cbw->dCBWDataTransferLength = data_length;
	cbw->bmCBWFlags = direction;
	cbw->bCBWLUN = lun;

	// Subclass is 1 or 6 => cdb_len

	cbw->bCBWCBLength = cdb_len;
	memcpy(cbw->CBWCB, cdb, cdb_len);

	i = 0;
	do {
		// The transfer length must always be exactly 31 bytes.
		r = usb_bulk_msg(device,usb_sndbulkpipe(device,endpoint_out),(unsigned char*)cbw, 31, &size, 1000);

		if (r == ERROR_PIPE) 
		{
			usb_clear_halt(device, usb_sndbulkpipe(device,endpoint_out));
		}
		i++;
	} while ((r == ERROR_PIPE) && (i<RETRY_MAX));

	if (r != SUCCESS) 
		{
			printk(KERN_INFO "Send_mass_storage_command: %d\n",r);
			return -1;
		}

	printk(KERN_INFO "Send %d CDB bytes\n", cdb_len);
	
	kfree(cbw);

	return 0;
}

//********************************************************************************************************//

//************************************get_mass_storage_status********************************************//

int get_mass_storage_status(struct usb_device *device, uint8_t endpoint_in,uint8_t tag)
{
	int i = 0, r, size;
	struct command_status_wrapper *csw;

	if( !(csw = (struct command_status_wrapper*)kmalloc(sizeof(struct command_status_wrapper), GFP_KERNEL)) ) 
	{
		printk(KERN_INFO "Cannot allocate memory for command status buffer\n");
		return -1;
	}
		
	do {
		r = usb_bulk_msg(device,usb_rcvbulkpipe(device,endpoint_in), (unsigned char*)csw, 13, &size, 1000);
		if (r != 0 ) 
			usb_clear_halt(device,usb_sndbulkpipe(device,endpoint_in));
		i++;
	}while ((r == ERROR_PIPE) && (i<RETRY_MAX));

	if (r != SUCCESS) 
	{
		printk(KERN_INFO "get_mass_storage_status: %d\n",r);
		return -1;
	}

	if (size != 13) 
	{
		printk(KERN_INFO "get_mass_storage_status: received %d bytes (expected 13)\n", size);
		return -1;
	}

	if (csw->dCSWTag != tag) 
	{
		printk(KERN_INFO "get_mass_storage_status: mismatched tags (expected %08X, received %08X)\n",tag, csw->dCSWTag);
		return -1;
	}

	printk(KERN_INFO "csw received!!! with status %d\n", csw->bCSWStatus);
	
	kfree(csw);				//free allocating memory
	
	return 0;
}

//********************************************************************************************************//

//*****************************************usbdev_probe**************************************************//

static int usbdev_probe(struct usb_interface *interface, const struct usb_device_id *id)  
{
	int i;
	unsigned char epAddr, epAttr, endpoint_in, endpoint_out;

	struct usb_device *device;
	struct usb_endpoint_descriptor *ep_desc;
	
	// get device structure out of interface
	device = interface_to_usbdev(interface);
	
	
	if(device == NULL) {
		printk(KERN_INFO "Cannot fetch device structure from interface structure\n");
		return -1;
	}

	endpoint_in = endpoint_out = 0;

	if(id->idProduct == HP_PID && id->idVendor == HP_VID)
	{
		printk(KERN_INFO "\nHP Pendrive Plugged in\n");
	}
	
	else	if(id->idProduct == SAN_PID && id->idVendor == SAN_VID)
	{
		printk(KERN_INFO "\nSandisk Pendrive Plugged in\n");
	}

	else
	{	
		printk(KERN_INFO "\nUnknown media plugged-in\n");
	}

	printk(KERN_INFO "VID:PID: %d:%d\n", id->idVendor, id->idProduct);
	printk(KERN_INFO "No. of Altsettings = %d\n", interface->num_altsetting);
	printk(KERN_INFO "No. of Endpoints = %d\n", interface->cur_altsetting->desc.bNumEndpoints);

	for(i = 0; i < interface->cur_altsetting->desc.bNumEndpoints;i++)
	{
		ep_desc = &interface->cur_altsetting->endpoint[i].desc; 	
		epAddr = ep_desc->bEndpointAddress;					
		epAttr = ep_desc->bmAttributes;								
	
		if((epAttr & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)  
		{ 
			if(epAddr & 0x80)		
				{
	 				if (!endpoint_in)
					{
						endpoint_in = ep_desc->bEndpointAddress;

						printk(KERN_INFO "EP %d is Bulk IN\n", i);
						printk(KERN_INFO "Endpoint IN address = %d\n ", endpoint_in);
						printk(KERN_INFO "Endpoint IN type = %d\n", epAttr);
					}
				}
			
			else 
				{
					if (!endpoint_out)
					{
						endpoint_out = ep_desc->bEndpointAddress;

						printk(KERN_INFO "EP %d is Bulk OUT\n",i);
						printk(KERN_INFO "Endpoint OUT address = %d\n", endpoint_out);
						printk(KERN_INFO "Endpoint OUT type = %d\n", epAttr);
					}
				}
		}
	}
		
		if((interface->cur_altsetting->desc.bInterfaceSubClass == 0x06) && (interface->cur_altsetting->desc.bInterfaceProtocol == 0x50))
		{
			printk(KERN_INFO "\nSCSI type device is connected\n");
		}	
		else
		{
			printk(KERN_INFO "\nOther type of device is connected\n");
		}

	printk(KERN_INFO "USB DEVICE CLASS : %x\n", interface->cur_altsetting->desc.bInterfaceClass);
	printk(KERN_INFO "USB DEVICE SUB CLASS : %x\n", interface->cur_altsetting->desc.bInterfaceSubClass);
	printk(KERN_INFO "USB DEVICE PROTOCOL : %x\n", interface->cur_altsetting->desc.bInterfaceProtocol);

	read_capacity(device, endpoint_in, endpoint_out);						//calling read_capacity function
	
	return 0;
 }

//********************************************************************************************************//

//*****************************************usbdev_disconnect*********************************************//

static void usbdev_disconnect(struct usb_interface *interface)
{
	printk(KERN_INFO "USBDEV Device Removed\n");
	return;
}

//********************************************************************************************************//

static struct usb_device_id usbdev_table [] = {		//device id table
	{USB_DEVICE(HP_VID, HP_PID)},					//passing device that driver going to support
	{USB_DEVICE(SAN_VID,SAN_PID)},
	{}										//terminating entry
};

/*operation structure*/
static struct usb_driver usbdev_driver = {
	name: "my_usbdev",					//name of the device
	probe: usbdev_probe,				//whenever device is plugged in
	disconnect: usbdev_disconnect,		//when we remove a device
	id_table: usbdev_table,				//list of devices served by this driver
};

int device_init(void)
{
	printk(KERN_NOTICE "\n\n********************UAS READ Capacity driver inserted************************\n");
	usb_register(&usbdev_driver);
	return 0;
}

void device_exit(void)
{
	usb_deregister(&usbdev_driver);
	printk(KERN_NOTICE "\n********************UAS READ Capacity driver leaving kernel******************\n");
}

module_init(device_init);
module_exit(device_exit);

MODULE_DESCRIPTION("Assignment_2");
MODULE_AUTHOR("Vandana Garg <vandana13.garg@gmail.com>");
MODULE_LICENSE("GPL");
