#include "jointsCtrl.h"
#include "Config.h"
#include <signal.h>
#include <cstdint>
#include <string>
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
	jointsCtrl.init(BUS_SERVO_BAUD_RATE, argv[1]);
	jointsCtrl.setJointType(JOINT_TYPE_SC);
	jointsCtrl.setEncoderStepRange(1024, 220);
	jointsCtrl.setJointsZeroPosArray(jointsZeroPos);
	double maxSpeedBuffer = jointsCtrl.getMaxJointsSpeed();
	jointsCtrl.setMaxJointsSpeed(0.1);
	jointsCtrl.linkArmFPVIK(LINK_AB + LINK_BF_1 + LINK_EF/2, 0, LINK_BF_2, 0);

	std::cout << "JointsCtrl initialized." << std::endl;

	jointsCtrl.setMaxJointsSpeed(maxSpeedBuffer);

	sleep(5);

#if 1
        double* lastIK = jointsCtrl.getRBZGIK();
	std::cout << lastIK[0] << " " << lastIK[1] << " " << lastIK[2] << " " << lastIK[3] << " " << lastIK[4] << std::endl;


#endif
	
	double x = 0;
	while (!b_exit) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		
		x-= 1;
		lastIK = jointsCtrl.smoothFPVAbsCtrl(120, 0, 200, x, 0.4);
		std::cout << lastIK[0] << " " << lastIK[1] << " " << lastIK[2] << " " << lastIK[3] << " " << lastIK[4] << std::endl;

	}

	std::cout << "main exited..." << std::endl;
}
