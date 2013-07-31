/*
 * HermesReferencesFrames.h
 *
 *  Created on: 30/07/2013
 *      Author: vr2user
 */

#ifndef HERMESREFERENCEFRAMES_H_
#define HERMESREFERENCEFRAMES_H_

#include "tf/tf.h"

class HermesReferenceFrames
{
private:
		tf::Transform *worldTleftarm;  // World to left robot
		tf::Transform *worldTrightarm;  // World to right robot


public:
		HermesReferenceFrames();
		~HermesReferenceFrames();
		void getworldTleftarm(tf::Transform *tf);
		void getworldTrightarm(tf::Transform *tf);



};



#endif /* HERMESREFERENCEFRAMES_H_ */
