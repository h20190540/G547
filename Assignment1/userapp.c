#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define MAJOR_NUM 100
	
#define IOCTL_WRITE _IOW(MAJOR_NUM, 0, int16_t*)
#define IOCTL_WRITE1 _IOW(MAJOR_NUM, 1, int16_t*)									//write user input to kernel

int main()
{
	int fd;
	int16_t channel,adc_value, align;	
	char buf[16];
	char a = 'n';
	printf("******************Opening the adc driver*****************\n");
	
	fd = open("/dev/adc8",O_RDWR);
	if(fd < 0)
	{
		printf("Cannot open device file....\n");
		return 0;
	}
	
	while(1)
	{
		if(a == 'n')
		{
			printf("\nEnter the channel(0-7))\n");
			scanf("%hd",&channel);

			printf("\nWriting channel to the driver\n");
			ioctl(fd, IOCTL_WRITE, &channel);

			printf("\nEnter the code for data alignment(0-1)\n");
			scanf("%hd",&align);

			printf("\nWriting align_code to the driver\n");
			ioctl(fd, IOCTL_WRITE1, &align);
			
			printf("Reading value from the driver\n");
			read(fd, buf, sizeof(buf));
			puts(buf);
			
			printf("\nDo you want to close the driver(y/n)\n");
			scanf("%s",&a);
		}
		
		else if( a == 'y')
			break;
	}
	printf("\nClosing adc driver\n");
	close(fd);
}

	
	


