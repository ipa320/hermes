#include <iostream>
#include "m5apiw32.h"


//hello.cpp
using namespace std;
int main()
{
	int ret=0;
	int dev=-1;
	int nModulos=-1;
	float posicion=-1.0;
	char pInitString[]="PCAN:/dev/pcan0,1000";
	const char* nameDev;
	const char* nameInit;
  	cout << "Leyendo módulos conectados..." << endl;
	ret=PCube_openDevice(&dev,pInitString);
	nameDev=PCube_getDeviceName(dev);
	cout << "dev: " <<  dev << "  Nombre:  " << nameDev <<endl;
	nameInit=PCube_getDeviceRevision( dev );
	cout << "InitString: "  << nameInit <<endl;
	if(ret==0){
		cout << "Leyendo módulos conectados..." << endl;
		nModulos=PCube_getModuleCount(dev);
		cout << "Módulos conectados:  " <<  nModulos <<endl;
		ret=PCube_getPos(dev,1,&posicion);
		cout << "Posición modulo 1:   " <<  posicion <<endl;
		ret=PCube_homeAll(dev);
		cout << "Code:   " <<  ret <<endl;
		ret=PCube_haltAll(dev);
		int number_joint = -1;
		while (true)
		{
			std::cout << "enter joint number: ";
			std::cin >> number_joint;

			if (number_joint == 0)
				break;

			float value = 0.f;
			std::cout << "enter the value for joint " << number_joint << ": ";
			std::cin >> value;

			PCube_resetModule(dev,number_joint);
			cout << "Code:   " <<  ret <<endl;
			ret=PCube_moveRamp(dev,number_joint,value,0.1,0.1);
			cout << "Code:   " <<  ret <<endl;
		}
		PCube_closeDevice(dev);
		
	}	
  return 0;
}
