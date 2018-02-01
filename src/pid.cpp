#include "pid.h"

#include <iostream>
#include <time.h>
#include <math.h>

using namespace std;

#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2

PID::PID()
{
	is_initialized = false;
}

PID::~PID() {}

void PID::initialize()
{
	steeringK[0] = 0.2; // Steering P
	steeringK[1] = 0.06; // Steering D
	steeringK[2] = 0.7e-04; // Steering I
	steeringK[3] = 0.02; // Speed P
	steeringK[4] = 0.4; // Speed D
	steeringK[5] = 2.e-05; // Speed I

	target_speed = 100.0;
	min_speed = 10.0;
	speed_limit_factor = 10.0;

	history_depth_sec = 10;
	last_timestamp = (double)clock() / CLOCKS_PER_SEC;
	last_measurement_history_time = last_timestamp;
	
	steering_history_buf.clear();
	steering_history_buf.push_back(0.0);

	speed_history_buf.clear();
	speed_history_buf.push_back(0.0);

	steering_integral = 0.0;
	speed_integral = 0.0;

	optimize = false;
	optSteeringD[0] = 0.1;
	optSteeringD[1] = 0.01;
	optSteeringD[2] = 0.0;
	optSteeringD[3] = 0.01;
	optSteeringD[4] = 0.1;
	optSteeringD[5] = 0.0;

	opt_passed_distance = 0.75; // Initial distance to speedup
	opt_parm_next = -2;
	opt_step_next = 0;
	passed_distance = 0.0;
	total_error = 0.0;
	best_error = 0.0;

	is_initialized = true;
}

void PID::update(const double& cte, const double& speed, const double& angle,
	double& steering, double& throttle)
{
	if (!is_initialized)
	{
		initialize();
		last_steering_error = cte;
		last_speed_error = speed - target_speed;
	}

	double time_stamp = (double)clock() / CLOCKS_PER_SEC;
	double dt = time_stamp - last_timestamp;

	if (optimize)
	{
		if (passed_distance >= opt_passed_distance)
		{
			if (opt_parm_next < 0)
			{
				best_error = total_error;
				opt_parm_next++;
				opt_step_next = 0;
				opt_passed_distance = 2.25;
			}
			else
			{
				if (opt_step_next == 1)
				{
					if (total_error < best_error)
					{
						printf("Best: P = %e, D = %e, I = %e, sP = %e, sD = %e, sI = %e\n",
							steeringK[0], steeringK[1], steeringK[2], steeringK[3], steeringK[4], steeringK[5]);
						best_error = total_error;
						optSteeringD[opt_parm_next] *= 1.25;
					}
					else
					{
						steeringK[opt_parm_next] -= optSteeringD[opt_parm_next];
						optSteeringD[opt_parm_next] *= -0.5;
					}

					opt_parm_next = (opt_parm_next + 1) % PID_PARM_NUM;
					opt_step_next = 0;
				}
			}

			if (opt_parm_next >= 0 && opt_step_next == 0)
			{
				int cnt = 0;
				while ((optSteeringD[opt_parm_next] == 0.0) || (cnt >= PID_PARM_NUM))
				{
					opt_parm_next = (opt_parm_next + 1) % PID_PARM_NUM;
					cnt++;
				}

				steeringK[opt_parm_next] += optSteeringD[opt_parm_next];
				opt_step_next = 1;
			}

			printf("    P = %e, D = %e, I = %e, sP = %e, sD = %e, sI = %e\n",
				steeringK[0], steeringK[1], steeringK[2], steeringK[3], steeringK[4], steeringK[5]);
			printf("        dP = %e, dD = %e, dI = %e, dsP = %e, dsD = %e, dsI = %e\n",
				optSteeringD[0], optSteeringD[1], optSteeringD[2], optSteeringD[3], optSteeringD[4], optSteeringD[5]);

			passed_distance = 0;
			total_error = 0;
		}
	}

	double steering_error = cte;
	double adjusted_target_speed = target_speed;
	double speed_error = speed - adjusted_target_speed;

	// Smoothness error
	/*if (((last_steering_error <= 0 && steering_error >= 0) || (last_steering_error >= 0 && steering_error <= 0)) &&
		dt > 0 &&
		speed > 0)
	{
		total_error += fabs(steering_error - last_steering_error) / (speed * dt);
	}*/

	// CTE error
	total_error += 0.25 * (steering_error + last_steering_error) * (steering_error + last_steering_error) * dt;

	// Speed error
	//total_error += dt;

	passed_distance += speed * (dt / 3600.0);

	//Uncomment to see passes distance
	//cout << passed_distance << endl;

	if ((time_stamp - last_measurement_history_time) < 1.0)
	{
		steering_history_buf[steering_history_buf.size() - 1] += steering_error;
		steering_integral += steering_error;

		speed_history_buf[speed_history_buf.size() - 1] += speed_error;
		speed_integral += speed_error;
	}
	else
	{
		steering_history_buf.push_back(steering_error);
		steering_integral += steering_error;
		if (steering_history_buf.size() > history_depth_sec)
		{
			steering_integral -= steering_history_buf[0];
			steering_history_buf.erase(steering_history_buf.begin());
		}

		speed_history_buf.push_back(speed_error);
		speed_integral += speed_error;
		if (speed_history_buf.size() > history_depth_sec)
		{
			speed_integral -= speed_history_buf[0];
			speed_history_buf.erase(speed_history_buf.begin());
		}

		last_measurement_history_time = time_stamp;
	}

	last_timestamp = time_stamp;

	double dcte = steering_error - last_steering_error;
	double dspeed = speed_error - last_speed_error;

	last_steering_error = steering_error;
	last_speed_error = speed_error;

	double tmp_steer = (-steeringK[0] * steering_error - steeringK[2] * steering_integral) / M_PI_2;
	if (dt > 0.00001)
	{
		tmp_steer -= steeringK[1] * dcte / dt / M_PI_2;
	}

	double tmp_throttle = 1.0 - steeringK[3] * fabs(steering_error) - steeringK[5] * speed_integral;
	if (dt > 0.00001)
	{
		tmp_throttle -= steeringK[4] * dcte / dt;
	}

	// Prevent go back
	if ((speed < min_speed) && (tmp_throttle < 1.0))
	{
		tmp_throttle = 1.0;
	}

	/*
	// Debug
	if (dt > 0.00001)
	{
		cout << "P: " << -steeringK[0] * steering_error / M_PI_2 << endl;
		cout << "D: " << -steeringK[1] * dcte / dt / M_PI_2 << endl;
		cout << "I: " << -steeringK[2] * steering_integral / M_PI_2 << endl;
	}
	cout << "Steering: " << tmp_steer << endl;

	if (dt > 0.00001)
	{
		cout << "sP: " << -steeringK[3] * fabs(steering_error) << endl;
		cout << "sD: " << -steeringK[4] * dcte / dt << endl;
		cout << "sI: " << -steeringK[5] * speed_integral << endl;
	}
	cout << "Throttle: " << tmp_throttle << endl;
	*/

	// Set output values
	steering = tmp_steer;
	throttle = tmp_throttle;
}

