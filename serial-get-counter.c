#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#define SERIAL_RESET_COUNTER 0
#define SERIAL_GET_TX_COUNTER 1
#define SERIAL_GET_RX_COUNTER 2

int main(int argc, char *argv[])
{
	unsigned int tx = 0, rx = 0;
	int fd, ret;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s /dev/UART\n", argv[0]);
		exit (1);
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Unable to open %s\n", argv[1]);
		exit (1);
	}

	ret = ioctl(fd, SERIAL_GET_TX_COUNTER, &tx);
	if (ret < 0) {
		fprintf(stderr, "Unable to get tx counter\n");
		exit (1);
	} else {
		printf("tx counter: %d\n", tx);
	}

	ret = ioctl(fd, SERIAL_GET_RX_COUNTER, &rx);
	if (ret < 0) {
		fprintf(stderr, "Unable to get rx counter\n");
		exit (1);
	} else {
		printf("rx counter: %d\n", rx);
	}
	
	close(fd);

	return 0;
}
