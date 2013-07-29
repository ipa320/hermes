/*
 * hermesInterface.cpp
 *
 *  Created on: 29/07/2013
 *      Author: vr2user
 */


#include "hermes_move_arm_action/HermesInterface.h"


HermesInterface::HermesInterface()
{
	dev=-1;
	pInitString="PCAN:/dev/pcan0,1000";


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

	/*if(ret==0){
		cout << "Leyendo módulos conectados..." << endl;
		nModulos=PCube_getModuleCount(dev);
		cout << "Módulos conectados:  " <<  nModulos <<endl;
		ret=PCube_getPos(dev,1,&posicion);
		cout << "Posición modulo 1:   " <<  posicion <<endl;
		ret=PCube_homeAll(dev);
		cout << "Code:   " <<  ret <<endl;
		ret=PCube_haltAll(dev);
		PCube_resetModule(dev,7);
		cout << "Code:   " <<  ret <<endl;
		ret=PCube_moveRamp(dev,7,1,0.2,0.2);
		cout << "Code:   " <<  ret <<endl;
		PCube_closeDevice(dev);*/

}
