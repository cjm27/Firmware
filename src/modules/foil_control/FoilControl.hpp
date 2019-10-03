#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
//#include <lib/ecl/l1/ecl_l1_pos_controller.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/pid/pid.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/uORB.h>
#include <platforms/px4_module.h>
#include <platforms/px4_module_params.h>

//using matrix::Dcmf;

using uORB::SubscriptionData;

class FoilControl : public ModuleBase<FoilControl>, public ModuleParams
{

public:
	FoilControl();
	~FoilControl();
	FoilControl(const FoilControl &) = delete;
	FoilControl operator=(const FoilControl &other) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static FoilControl *instantiate(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;


private:
	orb_advert_t	_pos_ctrl_status_pub{nullptr};		/**< navigation capabilities publication */
	orb_advert_t    _actuator_controls_pub{nullptr};	/**< actuator controls publication */

	int		_control_mode_sub{-1};		/**< control mode subscription */
	int		_global_pos_sub{-1};
	int		_local_pos_sub{-1};
	int		_manual_control_sub{-1};		/**< notification of manual control updates */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_pos_sp_triplet_sub{-1};
	int     _vehicle_attitude_sub{-1};
	int		_sensor_combined_sub{-1};

	manual_control_setpoint_s		_manual{};			    /**< r/c channel data */
	position_setpoint_triplet_s		_pos_sp_triplet{};		/**< triplet of mission items */
	vehicle_control_mode_s			_control_mode{};		/**< control mode */
	vehicle_global_position_s		_global_pos{};			/**< global vehicle position */
	vehicle_local_position_s		_local_pos{};			/**< global vehicle position */
	actuator_controls_s			    _act_controls{};		/**< direct control of actuators */
	vehicle_attitude_s              _vehicle_att{};
	sensor_combined_s				_sensor_combined{};

	SubscriptionData<vehicle_acceleration_s>		_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	hrt_abstime _control_position_last_called{0}; 	/**<last call of control_position  */

	/* Pid controller for the speed. Here we assume we can control airspeed but the control variable is actually on
	 the throttle. For now just assuming a proportional scaler between controlled airspeed and throttle output.*/
	PID_t _speed_ctrl{};

	// estimator reset counters
	uint8_t _pos_reset_counter{0};		// captures the number of times the estimator has reset the horizontal position

	//ECL_L1_Pos_Controller				_gnd_control;

	bool _waypoint_reached{false};

	enum UGV_POSCTRL_MODE {
		UGV_POSCTRL_MODE_AUTO,
		UGV_POSCTRL_MODE_OTHER
	} _control_mode_current{UGV_POSCTRL_MODE_OTHER};			///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GND_L1_PERIOD>) _param_l1_period,
		(ParamFloat<px4::params::GND_L1_DAMPING>) _param_l1_damping,
		(ParamFloat<px4::params::GND_L1_DIST>) _param_l1_distance,

		(ParamFloat<px4::params::GND_SPEED_TRIM>) _param_gndspeed_trim,
		(ParamFloat<px4::params::GND_SPEED_MAX>) _param_gndspeed_max,

		(ParamInt<px4::params::GND_SP_CTRL_MODE>) _param_speed_control_mode,
		(ParamFloat<px4::params::GND_SPEED_P>) _param_speed_p,
		(ParamFloat<px4::params::GND_SPEED_I>) _param_speed_i,
		(ParamFloat<px4::params::GND_SPEED_D>) _param_speed_d,
		(ParamFloat<px4::params::GND_SPEED_IMAX>) _param_speed_imax,
		(ParamFloat<px4::params::GND_SPEED_THR_SC>) _param_throttle_speed_scaler,

		(ParamFloat<px4::params::GND_THR_MIN>) _param_throttle_min,
		(ParamFloat<px4::params::GND_THR_MAX>) _param_throttle_max,
		(ParamFloat<px4::params::GND_THR_CRUISE>) _param_throttle_cruise,

		(ParamFloat<px4::params::GND_WHEEL_BASE>) _param_wheel_base,
		(ParamFloat<px4::params::GND_MAX_ANG>) _param_max_turn_angle
	)

	/**
	 * Update our local parameter cache.
	 */
	void parameters_update(int parameter_update_sub, bool force = false);

	// void		manual_control_setpoint_poll();
	// void		position_setpoint_triplet_poll();
	// void		vehicle_control_mode_poll();
	// void 		vehicle_attitude_poll();

	/**
	 * Control position.
	 */
	//bool		control_position(const matrix::Vector2f &global_pos, const matrix::Vector3f &ground_speed,
	//				 const position_setpoint_triplet_s &_pos_sp_triplet);

};
