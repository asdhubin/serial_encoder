#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include "odometer.h"

#define portname  "/dev/ttyUSB0"
#define revolution 1000 //every 1000 of encoder number is 1mm

static struct termios oldtty;

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);

        if (tcgetattr (fd, &oldtty) != 0)
        {
                // error_message ("error %d from tcgetattr", errno);
                printf("error opening the device");
                return -1;
        }

        memcpy( &tty,  &oldtty, sizeof (struct termios) );

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
            // error_message ("error %d from tcsetattr", errno);
            printf("error opening the device");
            return -1;
        }

        return 0;
} // end function: set_interface_attribs


int set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
            //error_message ("error %d from tggetattr", errno);
            printf("error opening the device");
            return -1;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
          //  error_message ("error %d setting term attributes", errno);
            printf("error opening the device");
    return 0;
} // end function: set_blocking


int main()
{
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
        printf("error opening the device\n");
    }


    /*CHANGES*/
    if(set_interface_attribs(fd, B9600, 0)!=0)
    {            printf("hello");
        printf("error set interface\n");
    }

    else


    if(set_blocking(fd, 0)!=0)
    {
        printf("error set blocking\n");
    }

    else
        printf("done");

    // what is this stray end of comment?
    // suggest enabling all the compiler warings
    // and fixing the warnings
    // */
    if( set_interface_attribs (fd, B9600, 0) )
    { // then set_interface_attribs failed
        return -1;
    }

    // implied else set_interface_attribs successful

    if( set_blocking (fd, 0) )                // set no blocking
    { // then set_blocking failed
        return -1;   // might need to also restore oldtty attributes
    }

    // implied else, set_blocking successful


    char receivebuffer [20];



    // implied else, read successful
    //initialize parameter
    unsigned char A5A5[2];A5A5[0]=0xA5;A5A5[1]=0xA5;

    int old_data=0;int new_data=0;
    float delta;
    class odometer test1;
    unsigned char var[4];//for saving 4bytes raw data

    while(1){
        write (fd,A5A5 , 2);

        usleep (3000);
    if( 6> read (fd, receivebuffer, sizeof receivebuffer) )
        { // then read failed
        return -1;
        break;
    }
        /*printf("value of buffer is %2d %2d %2d %2d %2d %2d \n\n",
        receivebuffer[0],
        receivebuffer[1],
        receivebuffer[2],
        receivebuffer[3],
        receivebuffer[4],
        receivebuffer[5]
        );*/  //here is test output,output like E1 9B FE FF FF 79
    for(int i=0;i<4;i++){
	    var[i]=receivebuffer[i+1];
    }


    old_data=new_data;
	if(receivebuffer[5]!=receivebuffer[0]+receivebuffer[1]+receivebuffer[2]+receivebuffer[3]+receivebuffer[4]){
        continue;
    }
	new_data=(var[3]<<24)|(var[2]<<16)|(var[1]<<8)|(var[0]);

    delta=new_data-old_data;
	test1.odo_add_mm(delta/revolution);
	test1.odo_print();
    }


    // cleanup
    tcsetattr (fd, TCSANOW, &oldtty);
    return 0;
} // end function: main
