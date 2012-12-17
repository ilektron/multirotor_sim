# Dual prop system controlling angular position

function retsys = characterize_multirotor(dt, sim_time, rotor_arm_len, rotor_arm_mass, delay, pgain, igain, dgain, pvgain, ivgain, dvgain)

	if (sim_time < 3)
		sim_time = 3;
	endif
		
# 	clear;
# 	clear plot;
# 	filecounter = 0;

# 		dt = 0.005;
	motor_mass = rotor_arm_mass;
	# 	motor_mass = 0.01
	# 	arm_mass = 0.01;
	# 	rotor_arm_in = 12;
	# 	rotor_arm_len = rotor_arm_in * 2.54 / 100;
	# 	delay = .1;
	motor_force = linspace(0,0, delay/dt + 1);

	arm_angle = 0;
	arm_velocity = 0;

	desired_angle = 0;

# 	pgain = 6;
# 	igain = 0;
# 	dgain = 2;

# 	pvgain = 2;
# 	ivgain = 0;
# 	dvgain = 0.35;

pvgain
dvgain

	counter = 2;

	command = 0;

	angles(counter) = arm_angle;

	for i = 0:dt:sim_time

		motor_force = [motor_force(2:length(motor_force)), command];
		commands(counter) = command;

	# 			angular_accel = motor_force(1) / (2*(motor_mass + arm_mass)/rotor_arm_len);
		angular_accel = motor_force(1) / (2*(motor_mass)/rotor_arm_len);
		arm_velocity = arm_velocity + angular_accel*dt;
		arm_angle = arm_angle + arm_velocity*dt;
		
		angles(counter) = arm_angle;
		
		if (i > 1 )
			desired_angle = 5*pi/180;
		endif
		
		inputs(counter-1) = desired_angle;

		piderror = desired_angle - arm_angle;
		
		velocity_command = piderror*pgain - (angles(counter) - angles(counter-1))*dgain/dt;
		
		pid_vel_error = velocity_command - arm_velocity;
		
		command = pid_vel_error*pvgain - angular_accel*dvgain;
		
		counter++;
		
	endfor

# 	if (exist("figure_num", "var") == 1)
		
# 		figure(figure_num)
		figure (1);
		clf

# 		subplot(2,2,1)
		x_axis = [0:dt:sim_time];

		plot(x_axis, angles(2:length(angles))*180/pi, x_axis, inputs*180/pi, x_axis, 10*commands(2:length(angles)));
		title(sprintf("Step Response - 5 degree step with %fs delay", delay));
		axis([0 sim_time -5 15]);

# 	endif

	dataset = iddata(angles(2:length(angles))', inputs', dt);
	[sys, x0, info] = moesp(dataset, 2);
	
	retsys = sys;

endfunction