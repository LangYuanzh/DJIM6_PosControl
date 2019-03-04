#include <sys/types.h> 
#include <sys/stat.h> 
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


struct uwb{
	u_int8_t header;
	u_int8_t mark;
	u_int8_t id;
	u_int8_t reserved1;
	struct{
		int x;
		int y;
		int z;
	} tag_position;
	struct{
		int x;
		int y;
		int z;
	} tag_velocity;
	int dis[8];
	struct {
		float x;
		float y;
		float z;
	} gyro;
	struct{
		float x;
		float y;
		float z;
	} acc;
	u_int8_t reserved2[12];
	struct{
		int x;
		int y;
		int z;
	} angle;
	float q0;
	float q1;
	float q2;
	float q3;
	u_int8_t reserved3[8];
	u_int32_t sys_time;
	u_int8_t sen_status;
	u_int8_t user_data[10];
	u_int8_t checksum;
};												//协议信息

int Byte16ToInt(u_int8_t *byteArray);			//协议转换函数
int Byte24ToInt(u_int8_t *byteArray);
int Byte32ToInt(u_int8_t *byteArray);
u_int32_t Byte32ToUint(u_int8_t *byteArray);
float Byte32ToFloat(u_int8_t *byteArray);
void BytetoStruct(uwb &tag0, u_int8_t buf[128]);

int open_port(int fd)							//开串口
{
	fd = open("/dev/ttyUSB1",O_RDWR);
	if(fd == -1)
	{
		perror("Can't Open SerialPort");
	}
	if(fcntl(fd, F_SETFL, 0)<0) 
		printf("fcntl failed!\n");
	else
		printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
	if(isatty(STDIN_FILENO)==0)
		printf("standard input is not a terminal device\n");
	else
		printf("isatty success!\n");
	printf("fd-open=%d\n",fd);
	return fd;
}
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)		//设置串口
{
	struct termios newtio,oldtio;
	if ( tcgetattr( fd,&oldtio) != 0) 
	{
		perror("SetupSerial 1");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	switch( nBits )
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
	switch( nEvent )
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
	}
	switch( nSpeed )
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}
	if( nStop == 1 )
    	newtio.c_cflag &= ~CSTOPB;
	else if ( nStop == 2 )
		newtio.c_cflag |= CSTOPB;
	newtio.c_cc[VTIME] = 5;
	newtio.c_cc[VMIN] = 50;
	tcflush(fd,TCIFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}
	printf("set done!\n");
  	return 0;
}
int main(int argc, char *argv[])  
{
	 int fd, res, i, count;
	 u_int8_t buf[128], sum;
	 uwb tag0;
	 fd = open_port(fd);
	 if(set_opt(fd,115200, 8, 'N', 1)!=0)
	{  
    	perror("UART_ERROR");  
        exit(1);  
    }
	while(1)
    {
		res = read(fd, buf, 128);		//读串口
		//printf("%d\n",res);
		if(buf[0] != 0x55)				//串口数据检测
		{
			for(i=0; i<128; i++)
				if(buf[i] == 0x55)
					if(buf[i+1] == 0x01)
						break;
			count = i;
			for(i=0; i<(128-count); i++)
				buf[i] = buf[count+i];
			read(fd, (buf+(128-count)) , count);
		}
		sum = 0;						//累加和校验
		for(i=0; i<127; i++)
			sum += buf[i];
		if(sum != buf[127])
			continue;
		/*for(i=0; i<128; i++)
		{
			printf("%u ",buf[i]);
		}*/
		BytetoStruct(tag0, buf);
		printf("%d\n",tag0.angle.x);
	
    }  	
	return 0;
}

int Byte16ToInt(u_int8_t *byteArray)
{
	u_int8_t i,temp[4];
	for(i=0; i<2; i++)
		temp[i] = *(byteArray+i);
	if(((*(byteArray+1))&0x80) != 0)
	{
		temp[2] = 0xff;
		temp[3] = 0xff;		
	}
	else
	{
		temp[2] = 0x00;
		temp[3] = 0x00;
	}
	return *((int *)temp);
}

int Byte24ToInt(u_int8_t *byteArray)
{
	u_int8_t i,temp[4];
	for(i=0; i<3; i++)
		temp[i] = *(byteArray+i);
	if(((*(byteArray+2))&0x80) != 0)
		temp[3] = 0xff;
	else
		temp[3] = 0x00;
	return *((int *)temp);
}
int Byte32ToInt(u_int8_t *byteArray)
{
	return *((int *)byteArray);
}
u_int32_t Byte32ToUint(u_int8_t *byteArray)
{
	return *((u_int32_t *)byteArray);
}
float Byte32ToFloat(u_int8_t *byteArray)
{
	return *((float*)byteArray);
}
void BytetoStruct(uwb &tag0, u_int8_t buf[128])
{
	int i;
	tag0.header = buf[0];
	tag0.mark	= buf[1];
	tag0.id		= buf[2];
	tag0.reserved1 = buf[3];
	tag0.tag_position.x = Byte24ToInt(buf+4);
	tag0.tag_position.y = Byte24ToInt(buf+7);
	tag0.tag_position.z = Byte24ToInt(buf+10);
	tag0.tag_velocity.x = Byte24ToInt(buf+13);
	tag0.tag_velocity.y = Byte24ToInt(buf+16);
	tag0.tag_velocity.z = Byte24ToInt(buf+19);
	for(i=0; i<8; i++)
		tag0.dis[i] = Byte24ToInt(buf+22+3*i);
	tag0.gyro.x	= Byte32ToFloat(buf+46);
	tag0.gyro.y = Byte32ToFloat(buf+50);
	tag0.gyro.z = Byte32ToFloat(buf+54);
	tag0.acc.x	= Byte32ToFloat(buf+58);
	tag0.acc.y	= Byte32ToFloat(buf+62);
	tag0.acc.z	= Byte32ToFloat(buf+66);
	for(i=0; i<12; i++)
		tag0.reserved2[i] = buf[70+i];
	tag0.angle.x = Byte16ToInt(buf+82);
	tag0.angle.y = Byte16ToInt(buf+84);
	tag0.angle.z = Byte16ToInt(buf+86);
	tag0.q0 = Byte32ToFloat(buf+88);
	tag0.q1 = Byte32ToFloat(buf+92);
	tag0.q2 = Byte32ToFloat(buf+96);
	tag0.q3 = Byte32ToFloat(buf+100);
	for(i=0; i<8; i++)
		tag0.reserved3[i] = buf[104+i];
	tag0.sys_time = Byte32ToUint(buf+112);
	tag0.sen_status = buf[116];
	for(i=0; i<10; i++)
		tag0.user_data[i] = buf[117];
	tag0.checksum = buf[127];
}
