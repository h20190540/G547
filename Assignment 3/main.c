#include<linux/kernel.h>
#include<linux/init.h>
#include<linux/module.h>
#include<linux/usb.h>				//for usbcd
#include<linux/slab.h>
#include<linux/log2.h> 				// for log function
#include<linux/genhd.h>
#include<linux/blkdev.h>
#include<asm/unaligned.h> 			// for put_alligned func
#include<linux/spinlock.h> 			// for put_alligned func

#define KINGSTON_VID 		 	 0x0951
#define KINGSTON_PID 		 	 0x1643
#define HP_VID 				 0x03f0
#define HP_PID 				 0x7f40

#define REQUEST_SENSE_LENGTH 0x12

#define SECTOR_SIZE 512

// for reading command
#define READ_COMMAND 			0x28
#define READ_SIZE 			 512

// for reading capacity
#define READ_CAPACITY_COMMAND  0x25
#define READ_CAPACITY_SIZE  0x08


//for block device
#define DEVICE_NAME 		"akanksha"
#define MAJOR_NUM    		 251
#define CAPACITY 		 31948800
#define DEVICE_RETRY    	 4

#define be_to_int32(buf) (((buf)[0]<<24)|((buf)[1]<<16)|((buf)[2]<<8)|(buf)[3])
#define MAX_RETRY 1


enum ep_directions {
	ENDPOINT_IN = 0x80,
	ENDPOINT_OUT = 0x00
};

//////////////////////////////////////// data structures ///////////////////////////////////////
//for block driver
typedef struct blkdev_private 
{
	struct request_queue *rq;
	struct gendisk *gd;
	spinlock_t lock;
	struct workqueue_struct *device_workqueue;
}blkdev;

//for usb driver
struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

struct command_status_wrapper {
	uint8_t dCSWSignature[4];
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
};

typedef struct usb_dev_info {
	struct usb_device *device;
	uint8_t epin,epout;

}dev_info;

//for work queue
struct dev_work {
	struct work_struct work;
	struct request *req;
};

//////////////////////////////////////////////////////////////////////////////////////////////////

extern blkdev *blockdevice;
extern dev_info device_info;
extern uint8_t command_length[256];
extern int driver_major_num;

// function declarations
//block fnc
extern int add_device(void);
extern void request_function(struct request_queue*);
//usb fnc
extern int read_capacity(void);
extern int test_mass_storage(void);
extern int read_usb(sector_t, uint16_t, uint8_t*);
extern int write_usb(sector_t, uint16_t, uint8_t*);

uint8_t command_length[256] = {
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

blkdev *blockdevice;
dev_info device_info;
int probe_var = 0;
int driver_major_num;

static void usbdev_disconnect(struct usb_interface *interface)
{
	printk(KERN_INFO "USBDEV Device is Removed\n");
	return;
}

static struct usb_device_id usbdev_table [] = {
	{USB_DEVICE(KINGSTON_VID, KINGSTON_PID)},
	{USB_DEVICE(HP_VID, HP_PID)},
	{} 
};

int send_command_mass_storage(uint8_t*, uint8_t, uint32_t, uint8_t*);
int get_mass_storage(uint8_t);

sector_t sectors_to_logical(sector_t sector)
{
	return sector >> (ilog2(SECTOR_SIZE) -9);
}

int test_mass_storage()
{

	uint8_t command[16] = {0};
	uint8_t tag;

	command[0] = 0x00; 				// read capacity

	// SEND COMMAND TO USB DEVICE
	if( send_command_mass_storage(command, ENDPOINT_IN,0, &tag) != 0 ) {
		printk(KERN_INFO "Send command mass storage error\n");
		printk(KERN_INFO "device resetting started\n");
		usb_reset_device(device_info.device);
		return -1;
	}

	if ( get_mass_storage(tag) == -1 ) {
		printk(KERN_INFO "Device is not ready\n");
		return -1;
	}

	return 0;

}

int write_usb(sector_t start_sector, uint16_t sectors_to_xfer, uint8_t *buffer)
{
	uint8_t command[16] = {0};
	uint8_t tag;
	int ret;
	int size = 0;
	int retry = 0;

	memset(command,0,16);

	// Write command
	command[0] = 0x2A; 
	put_unaligned_be32(start_sector,&command[2]);
	put_unaligned_be16(sectors_to_xfer,&command[7]);
	

	// Send command to the device
	if( send_command_mass_storage(command, ENDPOINT_OUT, sectors_to_xfer*SECTOR_SIZE, &tag) != 0 ) {
		printk(KERN_INFO "Send command error\n");
		return -1;
	}

	// to write the data
	do {
		ret = usb_bulk_msg(device_info.device, usb_sndbulkpipe(device_info.device,device_info.epout), buffer, sectors_to_xfer*SECTOR_SIZE, &size, 2000);
		retry++;
	}while( retry< MAX_RETRY && ret!=0 );

	if ( ret !=0 ) {
		printk(KERN_INFO "Writing endpoint command error ::%d\n",ret);
		return -1;
	}
	else 
		printk(KERN_INFO "Wrote %d bytes to device\n", size);


	if ( get_mass_storage(tag) == -1 ) {
		printk(KERN_INFO "cannot get command status block\n");
		return -1;
	}
	return 0;
}


int read_usb(sector_t start_sector, uint16_t sectors_to_xfer, uint8_t *buffer)
{
	uint8_t command[16] = {0};
	uint8_t tag;
	int ret;
	int size = 0;
	int retry = 0;

	memset(command,0,16);

	// Read command
	command[0] = 0x28;
	put_unaligned_be32(start_sector,&command[2]);
	put_unaligned_be16(sectors_to_xfer,&command[7]);
	

	// To send command to usb device
	if( send_command_mass_storage(command, ENDPOINT_IN, sectors_to_xfer*SECTOR_SIZE, &tag) != 0 ) {
		printk(KERN_INFO "Send command error\n");
		return -1;
	}

	// To read data from the device
	do {
		ret = usb_bulk_msg(device_info.device, usb_rcvbulkpipe(device_info.device,device_info.epin), buffer, sectors_to_xfer*SECTOR_SIZE, &size, 2000);
		retry++;
		if(ret!=0)
			usb_clear_halt(device_info.device, usb_rcvbulkpipe(device_info.device,device_info.epin));
	}while( retry<5 && ret!=0 );

	if ( ret !=0 ) {
		printk(KERN_INFO "Reading endpoint command error ::%d\n",ret);
		return -1;
	}
	else 
	//	printk(KERN_INFO "Read %d bytes from device\n", size);

	if ( get_mass_storage(tag) == -1 ) {
		printk(KERN_INFO "cannot get command status block\n");
		return -1;
	}
	return 0;
}

int read_capacity()
{
	uint8_t command[16] = {0};
	uint8_t *buffer = NULL;
	uint8_t tag;
	int ret;
	int size = 0;
	long max_lba,block_size;
	long device_size = 99;
	int retry = 0;

	if ( !(buffer = (uint8_t *)kmalloc(sizeof(uint8_t)*64, GFP_KERNEL)) ) {
		printk(KERN_INFO "Cannot allocate memory for rcv buffer\n");
		return -1;
	}
	memset(buffer,0,64);

	command[0] = 0x25; 				// read capacity

	//  To read data from the device
	if( send_command_mass_storage(command, ENDPOINT_IN, READ_CAPACITY_SIZE, &tag) != 0 ) {
		printk(KERN_INFO "Send command error\n");
		return -1;
	}

	// command to recieve data
	do {
		ret = usb_bulk_msg(device_info.device, usb_rcvbulkpipe(device_info.device,device_info.epin), (unsigned char*)buffer, READ_CAPACITY_SIZE, &size, 2000);
		retry++;
		usb_clear_halt(device_info.device, usb_rcvbulkpipe(device_info.device,device_info.epin));
	}while( retry<5 && ret!=0 );

	if ( ret !=0 ) {
		printk(KERN_INFO "Reading endpoint command error ::%d\n",ret);
		return -1;
	}
	else 
	
	max_lba = be_to_int32(buffer);
	block_size = be_to_int32(buffer+4);
	device_size = ((max_lba+1))*block_size/(1024*1024*1024); 
	printk(KERN_INFO "SIZE OF PENDRIVE -->  %ld GB\n", device_size);
	printk(KERN_INFO "max_lba: %ld, block_size: %ld\n",max_lba,block_size);
	
	kfree(buffer);
	if ( get_mass_storage(tag) == -1 ) {
		printk(KERN_INFO "cannot get command status block\n");
		return -1;
	}

	return 0;
}

int get_mass_storage(uint8_t tag)
{
	int retry = 0;
	int ret,size;
	struct command_status_wrapper *csw;

	if( !(csw = (struct command_status_wrapper*)kmalloc(sizeof(struct command_status_wrapper), GFP_KERNEL)) ) {
		printk(KERN_INFO "Cannot allocate memory for command status buffer\n");
		return -1;
	}


	do {
		ret = usb_bulk_msg(device_info.device,usb_rcvbulkpipe(device_info.device, device_info.epin), (unsigned char*)csw, 13, &size, 2000);
		if (ret != 0 ) 
			usb_clear_halt(device_info.device, usb_sndbulkpipe(device_info.device,device_info.epin));
		retry++;
	} while ( (ret!=0) && (retry < MAX_RETRY));
	

	if( ret!=0 ) {
		printk(KERN_INFO "csw read error:%d\n", ret);
		kfree(csw);
		return -1;
	}
	else
		
	kfree(csw);
	return ret;
	
}

int send_command_mass_storage(uint8_t *cmd, uint8_t dir, uint32_t data_len, uint8_t *rtag)
{
	struct command_block_wrapper *cbw = NULL;
	static uint32_t tag = 100; 							
	uint8_t cmd_len;
	int ret,retry =0;
	int size = 0;
	int pipe;
	
	// check if command array is valid or not
	if (cmd == NULL)
		return -1;
	
	pipe = usb_sndbulkpipe(device_info.device, device_info.epout);
	cbw = (struct command_block_wrapper*)kmalloc(sizeof(struct command_block_wrapper), GFP_KERNEL);
	if ( cbw == NULL ) {
		printk(KERN_INFO "Cannot allocate memory\n");
		return -1;
	}
	memset(cbw,'\0',sizeof(struct command_block_wrapper));


	*rtag = tag;
	cmd_len = command_length[cmd[0]];
	cbw->dCBWSignature[0] = 'U';
	cbw->dCBWSignature[1] = 'S';
	cbw->dCBWSignature[2] = 'B';
	cbw->dCBWSignature[3] = 'C';
	cbw->dCBWTag = tag++;
	cbw->dCBWDataTransferLength = data_len;
	cbw->bmCBWFlags = dir;
	cbw->bCBWLUN = 0;
	cbw->bCBWCBLength = cmd_len;
	memcpy(cbw->CBWCB, cmd, cmd_len);

	// SENDING THE COMMAND
 	do {
		ret = usb_bulk_msg(device_info.device, pipe, (unsigned char*)cbw, 31, &size, 2000);
		if( ret != 0 )
			usb_clear_halt(device_info.device, pipe);
		retry++;
	} while( ret!=0 && (retry < MAX_RETRY) );


	if ( ret !=0 ) {
		printk(KERN_INFO "send endpoint command error %d\n",ret);
		kfree(cbw);
		return -1;
	}

	kfree(cbw);
	return 0;

}

static int myUSBdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	int i,tu_ret, tu_try = 0;
	uint8_t epin=0, epout=0; 												// In endpoint and out endpoint address
	struct usb_endpoint_descriptor *ep_desc;

	// get device structure out of interface
	device_info.device = interface_to_usbdev(interface);
	if ( device_info.device == NULL ) {
		printk(KERN_INFO "cannot fetch device structure from interface structure\n");
		return -1;
	}

	// Getting VID and PID of the device from device descriptor
	printk(KERN_INFO "VID of the pendrive: %04x\n", device_info.device->descriptor.idVendor);
	printk(KERN_INFO "PID of the pendrive: %04x\n", device_info.device->descriptor.idProduct);

	
	if( (id->idProduct == KINGSTON_PID) || (id->idProduct == HP_PID) )
	{
		printk(KERN_INFO "USB device is detected\n");
	}
	else {
		printk(KERN_INFO "No device is detected\n");
		return -1;
	}

	// Getting other info of the interface of the device
	printk(KERN_INFO "USB Interface class : %04x\n", interface->cur_altsetting->desc.bInterfaceClass);
	printk(KERN_INFO "USB Interface Subclass : %04x\n", interface->cur_altsetting->desc.bInterfaceSubClass);
	printk(KERN_INFO "USB Interface Protocol : %04x\n", interface->cur_altsetting->desc.bInterfaceProtocol);
	printk(KERN_INFO "No. of Endpoints = %d\n", interface->cur_altsetting->desc.bNumEndpoints);
	printk(KERN_INFO "No. of Altsettings = %d\n",interface->num_altsetting);

	for(i=0; i < interface->cur_altsetting->desc.bNumEndpoints; i++)
	{
		ep_desc = &interface->cur_altsetting->endpoint[i].desc;

		if ( (ep_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) & USB_ENDPOINT_XFER_BULK ) {
			printk(KERN_INFO  "detection of bulk endpoint\n");
			if (ep_desc->bEndpointAddress & ENDPOINT_IN) {
				if(!epin)
					device_info.epin = ep_desc->bEndpointAddress;
				printk(KERN_INFO "IN endpoint having address %d\n",device_info.epin);
			}
			else {
				if(!epout)
					device_info.epout = ep_desc->bEndpointAddress;
				printk(KERN_INFO "OUT endpoint having address %d\n",device_info.epout);
			}
		}
	}
	
	usb_reset_device(device_info.device);
	// used to check that device is ready to write or read
	do{
		tu_ret = test_mass_storage();
		if( tu_ret!= 0 )
			printk(KERN_INFO "Device processing\n");
		tu_try++;
	} while( tu_try < DEVICE_RETRY && tu_ret != 0);

	if( tu_ret == 0 )
		printk(KERN_INFO "USB device is ready to receive read/write request\n");
	else {
		printk(KERN_INFO "USB DEVICE IS NOT READY\n");
		return -1;
	}


	if(add_device() != 0 ) {
		printk(KERN_INFO "Cannot register usb block dev\n");
		return -1;
	}

	probe_var = 1;

	return 0;
}

/*Operations structure*/
static struct usb_driver usbdev_driver = {
	name: "udriver",  											//name of the device
	probe: myUSBdev_probe, 											// Whenever Device is plugged in
	disconnect: usbdev_disconnect, 									// When we remove a device
	id_table: usbdev_table, 										//  List of devices served by this driver
};

static int open_block_device(struct block_device *, fmode_t);
static void close_block_device(struct gendisk *, fmode_t);


// block devices operations
struct block_device_operations dops =
{
	.owner = THIS_MODULE,
	.open = open_block_device,
	.release = close_block_device
};

static int open_block_device(struct block_device *device, fmode_t mode)
{
	printk(KERN_INFO "Open function called\n");
	return 0;
}

static void close_block_device(struct gendisk *gd, fmode_t mode)
{
	printk(KERN_INFO "Release function called\n");
}

void request_bottomhalf_func(struct work_struct*);
int sending_data(struct request*);	


void request_function(struct request_queue *q)
{
	struct request *req;
	struct dev_work *usb_work = NULL;

	while( (req = blk_fetch_request(q)) != NULL ) {

		usb_work = (struct dev_work*)kmalloc(sizeof(struct dev_work), GFP_ATOMIC);  // we do not want this allocation to sleep
		if ( usb_work == NULL ) {
			printk("Memory allocation for deferred work failed\n");
			 __blk_end_request_all(req, 0); 		
			continue;
		}

		usb_work->req = req;
		INIT_WORK(&usb_work->work, request_bottomhalf_func);
		queue_work( blockdevice->device_workqueue, &usb_work->work);
	}

}

void request_bottomhalf_func(struct work_struct *work)
{
	struct dev_work *usb_work = NULL;
	struct request *current_request = NULL;
	int ret = 0;
	unsigned long flags; 				// for spinlock

	usb_work = container_of(work, struct dev_work, work); 		// retriving my dev_work struct
	current_request = usb_work->req;

	ret = sending_data(current_request);

	spin_lock_irqsave( &blockdevice->lock, flags); 		
	__blk_end_request_all(current_request, ret);
	

	spin_unlock_irqrestore( &blockdevice->lock, flags);

	kfree(usb_work);
	return;
}

int sending_data(struct request *current_request)
{
	int ret = 0;
	int direction = rq_data_dir(current_request);
	// starting device sector for this request
	unsigned int total_sectors = blk_rq_sectors(current_request); // number of sectors to process

	struct bio_vec bio_vector;
	struct req_iterator iter;
	sector_t sector_offset = 0;
	unsigned int bio_vector_sectors;
	uint8_t *buffer = NULL;
	uint8_t *temporary_buffer = NULL;



	rq_for_each_segment(bio_vector,current_request,iter) {
		bio_vector_sectors = bio_vector.bv_len / SECTOR_SIZE; 
		temporary_buffer = (uint8_t*)kmalloc(total_sectors*SECTOR_SIZE,GFP_ATOMIC);
		if(!temporary_buffer) {
			printk(KERN_ERR "failed to allocate memory to buffer\n");
			return -1;
		}

		if (direction == 0 ) {
			printk(KERN_INFO "Read Request\n");

			if(read_usb(iter.iter.bi_sector, bio_vector_sectors, temporary_buffer) == 0)
				printk(KERN_INFO "Read Success\n"); 
			else
				printk(KERN_INFO "Read Failure\n");
			
			buffer = __bio_kmap_atomic(iter.bio, iter.iter);
			memcpy(buffer,temporary_buffer,bio_vector_sectors*SECTOR_SIZE);
			__bio_kunmap_atomic(buffer);
			
		}
		else {
			printk(KERN_INFO "Write request\n");
			buffer = __bio_kmap_atomic(iter.bio, iter.iter);
			memcpy(temporary_buffer,buffer,bio_vector_sectors*SECTOR_SIZE);
			__bio_kunmap_atomic(buffer);
			if(write_usb(iter.iter.bi_sector, bio_vector_sectors, temporary_buffer) == 0)
				printk(KERN_INFO "Write Success\n"); 
			else
				printk(KERN_INFO "Write Failure\n");
		}
		
		kfree(temporary_buffer);
		sector_offset += bio_vector_sectors; 
	}

	if ( sector_offset == total_sectors ) 
		ret = 0;
	else {
		printk(KERN_INFO "failed to transmit requested ssectors\n");
		ret = -EIO;
	}
	return ret;
}

int add_device()
{
	// driver registration
	driver_major_num = register_blkdev(0,DEVICE_NAME);
	if( driver_major_num < 0 ) {
		printk(KERN_INFO "failed to register device\n");
		return -EBUSY;
	}
	else
		printk(KERN_INFO "Major number of the registered device: %d\n", driver_major_num);


	// Memory allocation to my private structure of device
	blockdevice = kmalloc(sizeof(blkdev), GFP_KERNEL);
	if(blockdevice == NULL) {
		printk(KERN_INFO "Cannot allocate memory to private device structure\n");
		unregister_blkdev(driver_major_num,DEVICE_NAME);
		return -ENOMEM;
	}
	memset(blockdevice,0,sizeof(struct blkdev_private));	//initialization of memory
	spin_lock_init(&blockdevice->lock); 			//allocate and initialize the spinlock to control the access of spinlock
	blockdevice->rq = blk_init_queue(request_function,&blockdevice->lock);	//request_function : performs block read nd write request...allocating queue  
	if(!blockdevice->rq) {
		printk(KERN_ERR "Cannot initialize init queue\n");
		return -1;
	}

	/*
    	 * Add the gendisk structure
     	* By using this memory allocation is involved, 
     	* the minor number we need to pass bcz the device 
     	* will support this much partitions 
     	*/

	blockdevice->gd = alloc_disk(2); //allocating minor for gendisk		
	if( blockdevice->gd == NULL ) {
		printk(KERN_INFO "allocation disk failure\n");
		unregister_blkdev(driver_major_num,DEVICE_NAME);
		blk_cleanup_queue(blockdevice->rq);
		kfree(blockdevice);
		return -1;
	}
	blockdevice->device_workqueue = create_workqueue("bdev_queue"); //workqueue used to defer the work

	// fill gendisk structure
	blockdevice->gd->major = driver_major_num;	/* Setting the major number */
	blockdevice->gd->first_minor = 0;		/* Setting the first mior number */
	blockdevice->gd->fops = &dops;			/* Initializing the device operations */

	/* Driver-specific own internal data */	
	blockdevice->gd->queue = blockdevice->rq;
	blockdevice->gd->private_data = blockdevice;
	strcpy(blockdevice->gd->disk_name, DEVICE_NAME);

	set_capacity(blockdevice->gd,CAPACITY);		/* Setting the capacity of the device in its gendisk structure */
	
	add_disk(blockdevice->gd);			/* Adding the disk to the system */
	printk(KERN_INFO "Registering block device having capacity %ld\n", get_capacity(blockdevice->gd));
	printk(KERN_INFO "Finish\n");

	return 0;
}

int device_init(void)
{
	printk(KERN_INFO "\n\n********************UAS READ Capacity driver inserted************************\n");
	usb_register(&usbdev_driver);
	return 0;
}

void device_exit(void)
{
	usb_deregister(&usbdev_driver);
	if(probe_var) {
		del_gendisk(blockdevice->gd);
		flush_workqueue(blockdevice->device_workqueue);
		destroy_workqueue(blockdevice->device_workqueue);
		blk_cleanup_queue(blockdevice->rq);
		unregister_blkdev(driver_major_num, DEVICE_NAME);
		kfree(blockdevice);
	}
	printk(KERN_INFO "\n********************USB driver leaving kernel******************\n");
}

module_init(device_init);
module_exit(device_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Akanksha Bajpai,Vandana Garg");
