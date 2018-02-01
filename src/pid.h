#ifndef PID_H
#define PID_H

#include <vector>

#define PID_PARM_NUM (6)

class PID
{

protected:
	virtual void initialize();

public:
	bool is_initialized;

	double steeringK[PID_PARM_NUM];
	double target_speed;
	double min_speed;
	double speed_limit_factor;

	int history_depth_sec;
	std::vector<double> steering_history_buf;
	std::vector<double> speed_history_buf;

	double last_timestamp;
	double last_measurement_history_time;
	double last_steering_error;
	double last_speed_error;
	double steering_integral;
	double speed_integral;

	bool optimize;
	double optSteeringD[PID_PARM_NUM];
	double opt_passed_distance;
	int opt_parm_next;
	int opt_step_next;

	double passed_distance;
	double total_error;
	double best_error;
	
	PID();
	
	virtual ~PID();

	virtual void update(const double& cte, const double& speed, const double& angle,
		double& steering, double& throttle);
};

#endif /* PID_H */
