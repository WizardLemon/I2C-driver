#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

#define BUF_LEN 80

int file_desc;
unsigned char buff[BUF_LEN];

static sem_t sem_finish_signal;
static sem_t sem_error_signal;

void* print_state(void* param)
{
	unsigned int tmp;
	unsigned int mask;

	while(1)
	{

		if(sem_trywait(&sem_finish_signal) == 0)
		{
			break;
	 	}
	 	
		file_desc = open("/dev/i2c_driver", O_RDWR);

		if(file_desc < 0)
		{
			printf("Error, 'i2c_driver' not opened\n");
			sem_post(&sem_error_signal);
			break;
		}

		system("clear");

		/* Sending conversion command. */
		buff[0] = 'S';
		buff[1] = 1;
		buff[2] = 0x00;
		write(file_desc, buff, BUF_LEN);

		usleep(100);

		/* Data read command. */
		buff[0] = 'R';
		buff[1] = 6;
		write(file_desc, buff, BUF_LEN);

		usleep(100);

		read(file_desc, buff, BUF_LEN);

		if(buff[0] == 'E')
		{
			printf("Error encountered!\n");
			sem_post(&sem_error_signal);
			break;
		}

		printf("Joystick X: %d\n", (short int)buff[0]);
		printf("Joystick Y: %d\n", (short int)buff[1]);

		tmp = buff[2];			
		tmp = tmp << 2;
		
		mask = buff[5];
		mask &= 0x0C;
		mask = mask >> 2;

		tmp = tmp ^ mask;

		printf("Accelerometer X: %d\n", (short int)tmp);

		tmp = buff[3];			
		tmp = tmp << 2;
		
		mask = buff[5];
		mask &= 0x30;
		mask = mask >> 4;

		tmp = tmp ^ mask;

		printf("Accelerometer Y: %d\n", (short int)tmp);

		tmp = buff[4];			
		tmp = tmp << 2;
		
		mask = buff[5];
		mask &= 0xC0;
		mask = mask >> 6;

		tmp = tmp ^ mask;

		printf("Accelerometer Z: %d\n", (short int)tmp);

		tmp = buff[5];
		tmp &= 0x01;

		tmp = tmp ^ 1;

		printf("Z Button: %d\n", (short int)tmp);

		tmp = buff[5];
		tmp &= 0x02;
		tmp = tmp >> 1;
		
		tmp = tmp ^ 1;

		printf("C Button: %d\n", (short int)tmp);

		printf("To exit from this program, type in 'q' and press Enter\n");

		close(file_desc);

		usleep(100);
	}

	close(file_desc);
	return 0;
}

int main()
{
	unsigned int slave_address;
	unsigned char exit_char; 	
   
	pthread_t h_print_state;

	sem_init(&sem_finish_signal, 0, 0);
	sem_init(&sem_error_signal, 0, 0);

	/* Initializig buffer. */	
	memset(buff, '\0', BUF_LEN);

	/* Open file. */
    file_desc = open("/dev/i2c_driver", O_RDWR);

    if(file_desc < 0)
    {
       	printf("Error, 'i2c_driver' not opened\n");
       	return -1;
    }

	/* Setting slave address and sending it to driver */
	slave_address = 0x52;

	buff[0] = 'A';
	buff[1] = (unsigned char)slave_address;

	write(file_desc, buff, BUF_LEN);

	/* Initialize the the first register of the Nunchuk. */
	buff[0] = 'S';
	buff[1] = 2;
	buff[2] = 0xF0;
	buff[3] = 0x55;

	write(file_desc, buff, BUF_LEN);

	usleep(100);

	/* Initialize the the second register of the Nunchuk. */
	buff[0] = 'S';
	buff[1] = 2;
	buff[2] = 0xFB;
	buff[3] = 0x00;

	write(file_desc, buff, BUF_LEN);

	usleep(100);
	
	close(file_desc);

	pthread_create(&h_print_state, NULL, print_state, 0);
 
	while(1)
	{
		if(sem_trywait(&sem_error_signal) == 0)
		{
			break;
		}
		
		scanf("%c", &exit_char);
		if(exit_char == 'q')
		{
			sem_post(&sem_finish_signal);
			break;
		}
	}

	pthread_join(h_print_state, NULL);
	sem_destroy(&sem_finish_signal);
	sem_destroy(&sem_error_signal);
    
  	return 0;
}
