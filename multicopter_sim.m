# Dual prop system controlling angular position

function retval = multicopter_sim(dt, sim_time, rotor_arm_len, rotor_arm_mass, delay, pgain, igain, dgain, pvgain, ivgain, dvgain)

	figure_num = 1;
	sys = characterize_multirotor(dt, sim_time, rotor_arm_len, rotor_arm_mass, delay, pgain, igain, dgain, pvgain, ivgain, dvgain);
	
	# 	margin(sys);
	
	if (exist("filecounter", "var") == 1)
		filecounter++;
	else
		filecounter = 0;
	endif
	
	filename = sprintf("plot_%i.png", filecounter);
	print(filename);
	
# 	if (isstable(sys))
# 		printf("Is it stable at %f? Yes\n", delay);
# 	else
# 		printf("Is it stable at %f? No\n", delay);
# 		break;
# 	endif
	
	
# 	isstabilizable(sys)		
# 	delay
	
	if (isstable(sys))
	
		retval = sensitivity(sys);
		
	else
		
		retval = 0;
		
	endif

endfunction