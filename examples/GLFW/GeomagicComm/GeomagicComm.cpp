#include "chai3d.h"
#include "GeomagicComm.h"
using namespace chai3d;
using namespace std;

// a haptic device handler
cWorld* world;
cHapticDeviceHandler* handler;
cToolCursor* tool;
cGenericHapticDevicePtr hapticDevice;
cThread* hapticsThread;

bool simulationRunning = false;
bool simulationFinished = true;

cMatrix3d rotation;
cVector3d axis;
double angle;
cVector3d force(0, 0, 0);
cStereoMode stereoMode = C_STEREO_DISABLED;

double toolRadius;

bool dllRunning = false;

extern "C" 
{
	bool prepareHaptics(double hapticScale)
	{
		dllRunning = true;

		// create a new world.
		world = new cWorld();

		cMatrix3d rotation;

		// create a haptic device handler
		handler = new cHapticDeviceHandler();

		// open a connection to the first haptic device found
		handler->getDevice(hapticDevice, 0);

		// open a connection to haptic device
		hapticDevice->open();

		// calibrate device (if necessary)
		hapticDevice->calibrate();

		// create a 3D tool and add it to the world
		tool = new cToolCursor(world);
		world->addChild(tool);

		// connect the haptic device to the tool
		tool->setHapticDevice(hapticDevice);

		// define the radius of the tool (sphere)
		toolRadius = 0.025;

		// define a radius for the tool
		tool->setRadius(toolRadius);
		tool->enableDynamicObjects(true);
		tool->setWorkspaceRadius(hapticScale / 0.1);
		tool->setWaitForSmallForce(true);
		tool->start();

		hapticsThread = new cThread();

		return true;
	}

	int stopHaptics()
	{
		simulationRunning = false;
		double zeroForce[3];
		zeroForce[0] = zeroForce[1] = zeroForce[2] = 0;
		setForce(zeroForce);
		//close();
		return 1;
	}

	void setForce(double _force[])
	{
		convertXYZToCHAI3D(_force);
		force = cVector3d(_force[0], _force[1], _force[2]);
	}
	
	void startHaptics(void)
	{
		// create a thread which starts the main haptics rendering loop
		hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	}

	void getProxyPosition(double outPosArray[])
	{
		if (simulationRunning)
		{
			outPosArray[0] = tool->m_hapticPoint->getGlobalPosProxy().x();
			outPosArray[1] = tool->m_hapticPoint->getGlobalPosProxy().y();
			outPosArray[2] = tool->m_hapticPoint->getGlobalPosProxy().z();
			convertXYZFromCHAI3D(outPosArray);
		}
		else
		{
			outPosArray[0] = 0;
			outPosArray[1] = 0;
			outPosArray[2] = 0;
		}
	}

	void getProxyRotation(double outPosArray[])
	{
		if (simulationRunning)
		{
			rotation = tool->getDeviceGlobalRot();
			rotation.toAxisAngle(axis, angle);
			outPosArray[0] = axis.x();
			outPosArray[1] = axis.y();
			outPosArray[2] = axis.z();
			outPosArray[3] = angle;			
		}
		else
		{
			outPosArray[0] = 0;
			outPosArray[1] = 0;
			outPosArray[2] = 0;
			outPosArray[3] = 0;
		}
		
	}

	
	void updateHaptics(void)
	{
		// simulation in now running
		simulationRunning = true;
		simulationFinished = false;

		// main haptic simulation loop
		while (simulationRunning)
		{
			cVector3d totalForce(0, 0, 0);
			// read linear velocity 
			cVector3d linearVelocity;
			hapticDevice->getLinearVelocity(linearVelocity);
			/////////////////////////////////////////////////////////////////////
			// HAPTIC FORCE COMPUTATION
			/////////////////////////////////////////////////////////////////////

			// compute global reference frames for each object
			world->computeGlobalPositions(true);

			// update position and orientation of tool
			tool->updateFromDevice();
			tool->computeInteractionForces();

			totalForce.add(force);
			//totalForce.mul(1);
			if (force.length() > 0)
			{
				cHapticDeviceInfo info = hapticDevice->getSpecifications();
				// compute linear damping force
				double Kv = 1.0 * info.m_maxLinearDamping;
				cVector3d forceDamping = -Kv * linearVelocity;
				totalForce.add(forceDamping);
			}
			tool->setDeviceGlobalForce(totalForce);
			//tool->addDeviceGlobalForce(force);
			// compute interaction forces


			// send forces to haptic device
			tool->applyToDevice();
		}

		// exit haptics thread
		simulationFinished = true;
	}
}

void convertXYZFromCHAI3D(double inputXYZ[])
{
	double val0 = inputXYZ[0];
	double val1 = inputXYZ[1];
	double val2 = inputXYZ[2];

	inputXYZ[0] = val1;
	inputXYZ[1] = val2;
	inputXYZ[2] = -1 * val0;
}

void convertXYZRotFromCHAI3D(double inputXYZ[])
{
	double val0 = inputXYZ[0];
	double val1 = inputXYZ[1];
	double val2 = inputXYZ[2];
	
	inputXYZ[0] = val1;
	inputXYZ[1] = val2;
	inputXYZ[2] = -1 * val0;
}

void convertXYZToCHAI3D(double inputXYZ[])
{
	double val0 = inputXYZ[0];
	double val1 = inputXYZ[1];
	double val2 = inputXYZ[2];

	inputXYZ[0] = -1 * val2;
	inputXYZ[1] = val0;
	inputXYZ[2] = val1;
}
