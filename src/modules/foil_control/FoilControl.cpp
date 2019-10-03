#include "FoilControl.hpp"

extern "C" __EXPORT int foil_control_main(int argc, char *argv[]);

FoilControl::FoilControl() :
	ModuleParams(nullptr),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "rover position control")) // TODO : do we even need these perf counters
{
}

FoilControl::~FoilControl()
{
	perf_free(_loop_perf);
}

void
FoilControl::run()
{
	
}


int FoilControl::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("foil_ctrl",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_POSITION_CONTROL,
				      1700,
				      (px4_main_t)&FoilControl::run_trampoline,
				      nullptr);

	if (_task_id < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

FoilControl *FoilControl::instantiate(int argc, char *argv[])
{

	if (argc > 0) {
		PX4_WARN("Command 'start' takes no arguments.");
		return nullptr;
	}

	FoilControl *instance = new FoilControl();

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate FoilControl object");
	}

	return instance;
}

int FoilControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int FoilControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the position of a ground rover using an L1 controller.

Publishes `actuator_controls_0` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Throttle and yaw controls are passed directly through to the actuators
 * Auto mission: The rover runs missions
 * Loiter: The rover will navigate to within the loiter radius, then stop the motors

### Examples
CLI usage example:
$ rover_pos_control start
$ rover_pos_control status
$ rover_pos_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("foil_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int foil_control_main(int argc, char *argv[])
{
	return FoilControl::main(argc, argv);
}