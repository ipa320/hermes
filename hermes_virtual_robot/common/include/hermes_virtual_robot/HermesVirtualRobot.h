/*
 * HermesVirtualRobot.h
 *
 *  Created on: 04/10/2013
 *      Author: ricardo
 */

#ifndef HERMESVIRTUALROBOT_H_
#define HERMESVIRTUALROBOT_H_

#include "ros/ros.h"
#include <urdf/model.h>
#include <ros/package.h>

class HermesVirtualRobot
{
	private:
		bool hermes_correct;  //Indicates if hermes virtual robot is in correct state

	public:
		HermesVirtualRobot();
		bool getHermesCorrect();

	private:
		void readUrdfFile();


};


#endif /* HERMESVIRTUALROBOT_H_ */
