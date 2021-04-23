#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

int main(int ac, char*av[])
{
	struct termios tio;
	char outbuf[256];
	char inbuf[256];
	char* portname = "/dev/ttyUSB0"; // default
	printf("Upload Forth code to ESPForth\n");
	printf("(C) 2021 Jan Atle Ramsli\n");
	if (ac < 2){
		perror("Usage uf [filename] {port}");
		exit(-1);
	}
	
	FILE *f = fopen(av[1], "r");
	if (! f){
		perror(av[1]);
		exit(-1);
	}
	printf("File to upload: %s\n", av[1]);
	if (ac == 3){
		portname = av[3];
	}
	printf("Port: %s\n", portname);
	
	cfmakeraw(&tio);
	cfsetospeed(&tio,B115200);           
	cfsetispeed(&tio,B115200);           
	tio.c_cc[VMIN]=0;
	tio.c_cc[VTIME]=10;

	int serial_fd=open(portname, O_RDWR);


	if (serial_fd < 0){
		perror(portname);
	    exit(1);
	}
	tcsetattr(serial_fd,TCSANOW,&tio);
	int lno = 0;
	printf("Uploading ...\n");
	read(serial_fd, inbuf, 255);	// clean out the shit
	while(fgets(outbuf, 255, f))
	{
		lno++;
		write(serial_fd, outbuf, strlen(outbuf));
		sleep(1);
		read(serial_fd, inbuf, 255);

		printf("\rLine %d  ", lno);
		if (! strstr(inbuf, "ok")) {
			printf("%s ", inbuf);
		}
		fflush(stdout);
		if (strstr(inbuf, "ERROR")){
			printf(" %s %s\n", outbuf, inbuf);
			fflush(stdout);
			fclose(f);
			close(serial_fd);
			exit(1);		
		}		
	}
	printf(" lines successfully uploaded to ESPForth\n");
	fclose(f);
	close(serial_fd);
}

