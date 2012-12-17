# Dual prop system controlling angular position
clear;
clear plot;
filecounter = 0;

for delay = 0.01:0.01:0.5

	sim_time = 10;
	dt = 0.005;
	motor_mass = 0.078;
	arm_mass = 0.01;
	rotor_arm_in = 18;
	rotor_arm_len = 18 * 2.54 / 100;
# 	delay = .1;
	motor_force = zeros(delay/dt + 1);

	arm_angle = 0;
	arm_velocity = 0;

	desired_angle = 0;

	pgain = 1;
	igain = 0;
	dgain = 0;
	
	pvgain = 1;
	ivgain = 0;
	dvgain = 0.05;

	counter = 2;

	command = 0;

	angles(counter) = arm_angle;


	for i = 0:dt:sim_time

		motor_force = [motor_force(2:length(motor_force)), command];
		commands(counter) = command;

		angular_accel = motor_force(1) / (2*(motor_mass + arm_mass)/rotor_arm_len);
		arm_velocity = arm_velocity + angular_accel*dt;
		arm_angle = arm_angle + arm_velocity*dt;
		
		angles(counter) = arm_angle;
		
		if (i > 1 && i < 1.1)
			desired_angle = 15*pi/180;
	# 		command = 10;
	# 	else
	# 		command = 0;
		endif
		
	# 	inputs(counter-1) = command;
		
		inputs(counter-1) = desired_angle;

		piderror = desired_angle - arm_angle;
		
		velocity_command = piderror*pgain - (angles(counter) - angles(counter-1))*dgain/dt;
		
		pid_vel_error = velocity_command - arm_velocity;
		
		command = pid_vel_error*pvgain - angular_accel*dvgain;
		
		counter++;
		
	endfor

	figure(1)
	clf

	subplot(2,2,1)
	x_axis = [0:dt:sim_time];

	plot(x_axis, angles(2:length(angles))*180/pi, x_axis, inputs*180/pi, x_axis, 10*commands(2:length(angles)));
	title(sprintf("Step Response - 15 degree step with %fs delay", delay));
	axis([0 sim_time -10 35]);

	dataset = iddata(angles(2:length(angles))', inputs', dt);
	[sys, x0, info] = moesp(dataset, 2);

	[mag,phase,w] = bode(sys); 
	w = w/(2*pi);
	subplot(2,2,2);
	semilogx(w, mag);
	title('Magnitude');
	axis([0.1 50 -5 5]);
	subplot(2,2,4);
	semilogx(w, phase);
	axis([0.1 50 -180 90]);
	title('Phase (degrees)');
	subplot(2,2,3);
# 	margin(sys);
	filecounter++;
	filename = sprintf("plot_%i.png", filecounter);
	print(filename);
	if (isstable(sys))
		printf("Is it stable at %f? Yes\n", delay);
	else
		printf("Is it stable at %f? No\n", delay);
		break;
	endif
		
		
# 	isstabilizable(sys)
# 	delay
endfor

