#ifndef HERMES_INTERFACE_H_
#define HERMES_INTERFACE_H_

#include "m5apiw32.h"
#include <string>
#include <iostream>
#include <vector>




class HermesInterface
{
	private:
		int dev;
		std::string pInitString;
		std::vector <float> q_left;
		std::vector <float> q_right;


	public:
		HermesInterface();
		~HermesInterface();
		void moveLeftArm(std::vector<float> &q_in);
		void moveRightArm(std::vector<float> &q_in);
		void moveRightArmVel(std::vector<float> &q_Vel);
		void moveLeftArmVel(std::vector<float> &q_Vel);
		void readPositionLeftArm();
		void readPositionRightArm();
		void get_leftJoints(std::vector<float> &q);
		void get_rightJoints(std::vector<float> &q);
		void init();
		void softStopAll();




};
#endif
