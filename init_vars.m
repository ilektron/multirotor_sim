dt = 0.002		# seconds
sim_time = 5	# seconds
rotor_arm_len = 9*0.0254;	# 12 in -> m 
rotor_arm_mass = 0.2;		# Kg
delay = 0.25		# seconds

num = 15

pgain = 6
igain = 0
dgain = 2

pvgain = 2
ivgain = 0
dvgain = .35

sweep_sensitivity(dt, sim_time, rotor_arm_len, rotor_arm_mass, delay, pgain, igain, dgain, pvgain, ivgain, dvgain)