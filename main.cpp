#include "jointsCtrl.h"
#include "Config.h"
#include <signal.h>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <thread>

static volatile bool b_exit = false;
static void signal_handler(int sig) {
	std::cout << "signal_handler: " << sig << std::endl;
	if (sig == SIGINT) {
		std::cout << "SIGINT" << std::endl;
		b_exit = true;	
	}
}

int main(const int argc, const char *argv[]) {
	JointsCtrl jointsCtrl;

	int jointsZeroPos[JOINTS_NUM] = {519, 471, 484, 514};
	jointsCtrl.init(BUS_SERVO_BAUD_RATE);
	jointsCtrl.setJointType(JOINT_TYPE_SC);
	jointsCtrl.setEncoderStepRange(1024, 220);
	jointsCtrl.setJointsZeroPosArray(jointsZeroPos);
	double maxSpeedBuffer = jointsCtrl.getMaxJointsSpeed();
	jointsCtrl.setMaxJointsSpeed(0.1);
	jointsCtrl.linkArmFPVIK(LINK_AB + LINK_BF_1 + LINK_EF/2, 
			0, 
			LINK_BF_2,
			0);
	std::cout << "JointsCtrl initialized." << std::endl;
	jointsCtrl.setMaxJointsSpeed(maxSpeedBuffer);

	while (!b_exit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	std::cout << "main exited..." << std::endl;
}
