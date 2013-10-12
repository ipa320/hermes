/*
 * hermesInterface.cpp
 *
 *  Created on: 29/07/2013
 *      Author: vr2user
 */


#include "hermes_robot_interface/HermesInterface.h"


HermesInterface::HermesInterface()
{
	dev=-1;
	pInitString="PCAN:/dev/pcan0,1000";
	q_left.resize(7,0.0);
	q_right.resize(7,0.0);


}

HermesInterface::~HermesInterface()
{
	if(dev==0)
	{
		PCube_closeDevice(dev);
	}


}

void HermesInterface::init()
{

	int ret=0;
	int nModulos=-1;
	const char* nameDev;
	const char* nameInit;

	ret=PCube_openDevice(&dev,pInitString.c_str());
	nameDev=PCube_getDeviceName(dev);
	std::cout << "dev: " <<  dev << "  Nombre:  " << nameDev <<std::endl;
	nameInit=PCube_getDeviceRevision( dev );
	std::cout << "InitString: "  << nameInit <<std::endl;

	if(ret==0){
		nModulos=PCube_getModuleCount(dev);
		std::cout << "Modules Conected:  " <<  nModulos << std::endl;

		// Read init position
		readPositionLeftArm();
		readPositionRightArm();

		ret=PCube_homeAll(dev);  // Home all Modules
		ret=PCube_haltAll(dev);  // Stop all modules
		for(int i=1;i<=7;i++)
			PCube_resetModule(dev,i); // Reset leftArm
		for(int i=10;i<=17;i++)
					PCube_resetModule(dev,i);// Reset rightArm

	}

}

void HermesInterface::moveLeftArm(std::vector<float> &q_in)
{


	for(int i=1;i<=7;i++)
		PCube_moveRamp(dev,i,(float)q_in[i-1],0.2,0.2);



}

void HermesInterface::moveRightArm(std::vector<float> &q_in)
{


	for(int i=11;i<=17;i++)
			PCube_moveRamp(dev,i,(float)q_in[i-1-10],0.2,0.2);



}

void HermesInterface::readPositionLeftArm()
{

	int ret = -1;
	// Read init position
	for(int i=i;i<=7;i++)
		ret=PCube_getPos(dev,i,&q_left[i-1]); // Read leftArm Joints



}

void HermesInterface::readPositionRightArm()
{

	int ret = -1;
	for(int i=11;i<=17;i++)
			ret=PCube_getPos(dev,i,&q_right[i-1-10]);// Read rightArm Joints



}

void HermesInterface::get_leftJoints(std::vector<float> &q)
{
	readPositionLeftArm();
	q=this->q_left;
}

void HermesInterface::get_rightJoints(std::vector<float> &q)
{
	readPositionRightArm();
	q=this->q_right;
}


void HermesInterface::moveLeftArmVel(std::vector<float> &q_Vel)
{


	for(int i=1;i<=7;i++)
			PCube_moveVel(dev,i,(float)q_Vel[i-1]);



}

void HermesInterface::moveRightArmVel(std::vector<float> &q_Vel)
{


	for(int i=11;i<=17;i++)
			PCube_moveVel(dev,i,(float)q_Vel[i-1-10]);



}

void HermesInterface::softStopAll()
{
	PCube_haltAll(dev);
	for(int i=1;i<=7;i++)
			PCube_resetModule(dev,i); // Reset leftArm
	for(int i=10;i<=17;i++)
			PCube_resetModule(dev,i);// Reset rightArm

}

