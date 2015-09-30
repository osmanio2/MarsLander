// Mars lander simulator
// Version 1.8
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, October 2014

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

vector3d previous_position;
bool update_first_pos = false;
double tolerance = 5.0;
double non_steady_wind_speed;
int non_steady_wind_direction;
int count_ = 0;
bool calculated_required_velocity = false;
bool reached_required_speed = false;
double required_speed;
double ground_speed_;
enum orbital_injection_states {in_orbit, leaving_orbit, changing_orbit, entering_orbit};

orbital_injection_states orbiting_state;

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
 
	// Initialise some variables
	vector3d er = position.norm();
	double ascendRate = velocity * er;
	vector3d tv = velocity - ascendRate*er;
	ground_speed_ = tv.abs();

	// check the target height for orbital injection
	if (round(target_height) != 0) {
		orbital_injection();
		return;
	}
	if (!calculated_required_velocity) {
		// Calculate the required velocity to leave an orbit
		stabilized_attitude = true;
		double ra = position.abs();
		double rp = MARS_RADIUS + (EXOSPHERE/2);
		double speed_sqr = (2 * GRAVITY * MARS_MASS) * (rp / ra) * (1 / (rp + ra));
		required_speed = sqrt(speed_sqr) - tolerance;
		calculated_required_velocity = true;
	}
	// Initialist parameters to be used for the autopilot
	static double previous_speed;
	static double previous_error;
	double Kh = 0.03;
	double Kp = 0.5;
	double range = 0.5;
	double height = (position.abs() - MARS_RADIUS);

	// If the fuel is infinite change modify some parameters
	if (fuel >= 1.0) {
		Kh = 0.01;
		range = 0.6;
	}
	double error = -(0.5 + (Kh * height) + ascendRate);
	double Pout;
	if (count_ == 1) 
	{
		// Differential parameter to enhance the autopilot
		Kp = Kp / (500 * abs(error - previous_error));
		if ( (height < 10) || ( (abs(ascendRate) < 0.5) && height < 200 ) ) {
			Kp = 0.5;
			if (parachute_status == DEPLOYED) parachute_status = LOST;
		}
	}
	
	Pout = Kp * error;
	// Autopilot controller
	if (Pout <= -range) 
	{
		throttle = 0.0;
	}
	else if (Pout < (1 - range))
	{
		throttle = range + Pout;
	}
	else
	{
		throttle = 1.0;
	}

	bool rate_check_to_deploy = false;

	if (count_ == 1) {
		rate_check_to_deploy = (-(ascendRate + previous_speed) <= 0);
	}
	// In orbit autopilot controller
	if ((height > (EXOSPHERE - tolerance)) && !reached_required_speed && ground_speed_ >= required_speed) {
		stabilized_attitude_angle = 270;
		throttle = 1.0;
	} else {
			// If for some reason the ground speed is greater than 1, maybe because of the steady wind flow
			if ((height < 1000.0) && (height < (33.33 * ground_speed)) && (ground_speed > 1.0)) {
				
				bool ground_speed_positive = velocity * mars_velocity(position) >= 0.0;
				stabilized_attitude_angle = ground_speed_positive ? 45 : 315;
			
			} else {
				stabilized_attitude_angle = 0;
			}
		}
	if (!reached_required_speed && ground_speed_ < required_speed) {
		stabilized_attitude_angle = 0;
		reached_required_speed = true;
	}
	// Condition for deploying the parachute
	if (safe_to_deploy_parachute() && parachute_status == NOT_DEPLOYED && (ascendRate < 0) && rate_check_to_deploy && ( height < (EXOSPHERE - tolerance) ) ) {

		parachute_status = DEPLOYED;

	}
	previous_speed = - ascendRate;
	previous_error = error;
	count_ = 1;
}

void orbital_injection(void) {
	static double range = 100.0;
	static double k = 10.0;
	static double required_final_speed;
	double height = position.abs() - MARS_RADIUS;
	switch (orbiting_state)
	{
	case in_orbit:
	{
		double rp = MARS_RADIUS + target_height;
		range = target_height / 5000.0;
		if (height > target_height) range = height / 2000.0;
		required_final_speed = sqrt((GRAVITY * MARS_MASS) / rp);
		stabilized_attitude = true;
		double ra = position.abs();
		double speed_sqr = (2 * GRAVITY * MARS_MASS) * (rp / ra) * (1 / (rp + ra));
		required_speed = sqrt(speed_sqr);
		orbiting_state = leaving_orbit;
		break;
	}
	case leaving_orbit:
	{
		int angle_offset = 0;
		if (height < 200000.0) {
			angle_offset = -45;
		}
		stabilized_attitude_angle = 90 + angle_offset;
		if (ground_speed_ >= required_speed) {
			stabilized_attitude_angle = 270 - angle_offset;
		}
		throttle = 1.0;
		if (required_speed < (ground_speed_ + 0.5) && required_speed > (ground_speed_ - 0.5)) {
			throttle = 0.0;
			stabilized_attitude_angle = 0;
			stabilized_attitude = false;
			orbiting_state = changing_orbit;
		}
		break;
	}
	case changing_orbit:
		if (height < (target_height + range) && height > (target_height - range)) {
			orbiting_state = entering_orbit;
		}
		break;
	case entering_orbit:
	{
		double ascendRate = velocity * position.norm();
		double error = 0.0;
		if (target_height > height) 
		{
			error = (required_final_speed * (height / target_height)) - ground_speed_;
			stabilized_attitude = true;
			double target_rate = (target_height - height) / 12.0;
			if (abs(target_rate) < 0.5) {
				target_rate = 0.5;
			}
			int orient = 45;
			if (abs(ascendRate) <= abs(target_rate) + 0.5 && abs(ascendRate) >= abs(target_rate) - 0.5)
			{
				orient = 0;
			}
			if (ascendRate < target_rate)
			{
				orient = -45;
			}
			stabilized_attitude_angle = 90 + orient;
			if (ground_speed_ >= required_final_speed) {
				stabilized_attitude_angle = 270 - orient;
			}
			double Pout = k * error;
			if (Pout <= -range)
			{
				throttle = 1.0;
			}
			else if (Pout >= range)
			{
				throttle = 1.0;
			}
			else
			{
				throttle = (abs(Pout) / range);
			}
			range = 0.5 + (abs(target_height - height) / 5000.0);
		}
		else
		{
			error = (required_final_speed * (target_height / height)) - ground_speed_;
			stabilized_attitude = true;
			double target_rate = (target_height - height) / 12.0;
			if (abs(target_rate) < 0.5) {
				target_rate = -0.5;
			}
			int orient = 45;
			if (abs(ascendRate) <= abs(target_rate) + 0.5 && abs(ascendRate) >= abs(target_rate) - 0.5)
			{
				orient = 0;
			}
			if (ascendRate < target_rate)
			{
				orient = -45;
			}
			stabilized_attitude_angle = 90 + orient;
			if (ground_speed_ >= required_final_speed) {
				stabilized_attitude_angle = 270 - orient;
			}
			double Pout = k * error;
			if (Pout <= -range)
			{
				throttle = 1.0;
			}
			else if (Pout >= range)
			{
				throttle = 1.0;
			}
			else
			{
				throttle = (abs(Pout) / range);
			}
			range = 0.5 + (abs(target_height - height) / 5000.0);
		}

		if (height < (target_height + 0.5) && height >(target_height - 0.5) && abs(error) < 1.0 && abs(ascendRate) < 1.0) {
			orbiting_state = in_orbit;
			throttle = 0.0;
			stabilized_attitude_angle = 0;
			stabilized_attitude = false;
			target_height = 0.0;
			autopilot_enabled = false;
		}
		break;
	}
	}
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
    // Infinite fuel is enabled
	if(infinite_fuel) fuel = 1;

	// Calculate the relative velocity to be used for modelling the wind and plant's rotation
	vector3d relative_velocity = velocity;
	vector3d mars_rotational_velocity = mars_velocity(position);

	// If the mechanics of the planet's rotation is enabled
	if (planet_rotation) relative_velocity = velocity - mars_rotational_velocity;
	
	// Calculate the wind velocity if it's enabled
	vector3d wind_velocity = vector3d(0,0,0);
	if (wind_flow) {
		// For steady flow used the average wind speed on Mars, i.e 10 m/s and in the same direction of the plant's rotation
		wind_velocity = 10.0 * mars_rotational_velocity.norm();
	}
	else if (wind_flow_gusts) {
		// For non-steady flow use a random direction and speed
		wind_velocity = non_steady_wind_direction * non_steady_wind_speed * mars_rotational_velocity.norm();
	}
	relative_velocity -= wind_velocity;
	// Calculate the lander mass
	double lander_mass = UNLOADED_LANDER_MASS + (fuel * FUEL_CAPACITY * FUEL_DENSITY);

	vector3d acceleration = -GRAVITY * MARS_MASS * (position.norm() / position.abs2());
	// Calculate the drag force
	vector3d lander_drag = -0.5*DRAG_COEF_LANDER*atmospheric_density(position)*M_PI*LANDER_SIZE*LANDER_SIZE*relative_velocity.abs2()*relative_velocity.norm();
	if (parachute_status == DEPLOYED)
	{
		vector3d parachute_drag = -0.5*DRAG_COEF_CHUTE*atmospheric_density(position)*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*relative_velocity.abs2()*relative_velocity.norm();
		lander_drag = lander_drag + parachute_drag;
	}
	// Calculate resultant acceleration
	acceleration = acceleration + ((lander_drag + thrust_wrt_world()) / lander_mass);

	// For the first time use Euler integrator,
	if (!update_first_pos) 
	{
		previous_position = position;
		position = previous_position + velocity * delta_t;
		velocity = velocity + acceleration * delta_t;
		update_first_pos = true;
	}
	else // Use Verlet integrator,
	{
		vector3d temp = position;
		position = (2 * position) - previous_position + (delta_t * delta_t * acceleration);
		previous_position = temp;
		velocity = (1 / delta_t) * (position - previous_position);
	}
  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
	if (autopilot_enabled)  autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "Areostationary orbit";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  // Randomness for non-steady wind flow
  double random = rand();
  non_steady_wind_direction = ((random / RAND_MAX) >= 0.5) ? 1 : -1;
  random = rand();
  non_steady_wind_speed = (random / RAND_MAX) * 30.0;
  // Initialise all variables needed
  planet_rotation = true;
  wind_flow = false;
  wind_flow_gusts = false;
  infinite_fuel = false;
  orbiting_state = in_orbit;
  target_height = 0.0;
  count_ = 0;
  update_first_pos = false;
  calculated_required_velocity = false;
  reached_required_speed = false;
  stabilized_attitude = false;
  double raduis_3 = (GRAVITY * MARS_MASS * MARS_DAY * MARS_DAY) / (4 * M_PI * M_PI);
  double raduis = cbrt(raduis_3);
  double speed = (2 * M_PI * raduis) / MARS_DAY;
  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
	  // a circular areostationary orbit
	  position = vector3d(raduis, 0.0, 0.0);
	  velocity = vector3d(0.0, speed, 0.0);
	  orientation = vector3d(0.0, 90.0, 0.0);
	  delta_t = 0.1;
	  parachute_status = NOT_DEPLOYED;
	  stabilized_attitude = false;
	  autopilot_enabled = false;

    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}