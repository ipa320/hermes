#include "hermes_reference_frames_service/HermesReferenceFrames.h"

HermesReferenceFrames::HermesReferenceFrames()
{
	worldTleftarm = new tf::Transform(tf::Matrix3x3(1,0,0,0,0,-1,0,1,0),tf::Vector3(0,0.075,1.51)); // World to left Robot
	worldTrightarm = new tf::Transform(tf::Matrix3x3(1,0,0,0,0,1,0,-1,0),tf::Vector3(0,-0.075,1.51)); // World to right Robot



}

HermesReferenceFrames::~HermesReferenceFrames()
{
		if(worldTleftarm != 0)
				delete worldTleftarm;

		if(worldTrightarm != 0)
				delete worldTrightarm;

}

void HermesReferenceFrames::getworldTleftarm(tf::Transform *tf)
{
	*tf = *worldTleftarm;



}

void HermesReferenceFrames::getworldTrightarm(tf::Transform *tf)
{
	*tf = *worldTrightarm;
}
