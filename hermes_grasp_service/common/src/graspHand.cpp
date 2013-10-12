#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hermes_grasp_service/azzura.h"
#include "hermes_grasp_service/graspHand.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>

using namespace std;


int GraspHand::init(int port)
{
  /** Create ttyUSB0 paralela */
  file_description = open_port(port);
  

  if(configure_port(file_description)== -1) // if open is unsucessful
  {
	return -1;
  }
   
  return 0;
}

int GraspHand::open_port(int port)
{
	int fd;	// file description for the serial port

	if(port==1)	
		fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
	else if(port==2)
		fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	else
		fd=-1;
	if(fd == -1) // if open is unsucessful
	{
		//perror("open_port: Unable to open /dev/ttyS0 - ");
		printf("open_port: Unable to open /dev/ttyUSBS%i. \n", port-1);
	}
	else
	{
		fcntl(fd, F_SETFL, FNDELAY);
		printf("/dev/ttyUSBS%i Successfully Opened \n", port-1);
		//printf("port is open.\n");
	}
	
	return(fd);
} //open_port

int GraspHand::configure_port(int fd)      // configure the port
{
	struct termios port_settings;      // structure to store the port settings in

	cfsetispeed(&port_settings, B115200);    // set baud rates
	cfsetospeed(&port_settings, B115200);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;
	
	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
	return(fd);

} //configure_port


int GraspHand::sendByte(int fd, char data)   // query modem with an AT command
{
		
	
	
	
	write(fd, &data, 1);  //Send data
	printf("Serial Data Write: [%c]",data);
	//printf("Data wrote. \n");
	
	
	return 0;
	
} //open Hand

void GraspHand::strongGrip(int fd)
{
	ros::Rate loop_rate(4);
	char data[2];

	data[0]=195;
	data[1]=255;
	write(fd, &data, 2);
	loop_rate.sleep();
	loop_rate.sleep();
	data[0]=199;
	data[1]=255;
	write(fd, &data, 2);
	loop_rate.sleep();
	data[0]=203;
	data[1]=255;
	write(fd, &data, 2);
	loop_rate.sleep();
	data[0]=207;
	data[1]=255;
	write(fd, &data, 2);
	loop_rate.sleep();
	data[0]=211;
	data[1]=255;
	write(fd, &data, 2);
}

void GraspHand::doPeineta(int fd)
{
	
	char data[2];



	data[0]=142;
	data[1]=255;
	write(fd, &data, 2);


}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void GraspHand::executeGrasp(int type, int force)
{
  std::string ss; 

  if(type==CYLINDER)
  {
	if(force<34)
		sendByte(file_description,CYLLOW_C);
	else if(force <67)
		sendByte(file_description,CYLMED_C);	
	else
		sendByte(file_description,CYLHIGH_C);
  }
  else if(type==BIFINGER)
  {

	sendByte(file_description,BILOW_C);

  }
  else if(type==BIFINGER2)
  {
	sendByte(file_description,BI2LOW_C);
  }
  else if(type==TRIFINGER)
  {
	if(force<34)
		sendByte(file_description,TRILOW_C);
	else if(force <67)
		sendByte(file_description,TRIMED_C);	
	else
		sendByte(file_description,TRIHIGH_C);
  }
  else if(type==TRIFINGER2)
  {
	sendByte(file_description,TRI2LOW_C);
  }
  else if(type==LATERAL)
  {
	sendByte(file_description,LATHIGH_C);
  }

  else if(type==OPENALL)
  {
	sendByte(file_description,OPEN_ALL);
  }
  else if(type==STOPALL)
  {
	sendByte(file_description,STOP_ALL);
  }
  else if(type==FIRSTCALIBRATION)
  {
	sendByte(file_description,FIC);
  }
  else if(type==FASTCALIBRATION)
  {
	sendByte(file_description,FAC);
  }
  else if(type==STRONGGRIP)
  {
	strongGrip(file_description);
  }
  else if(type==PEINETA)
  {
	doPeineta(file_description);
  }
  
  else if(type==PREGRASP)
  {
	char data[2];
	data[0]=195;
	data[1]=255;
	write(file_description, &data, 2);
  }

 
 
}



