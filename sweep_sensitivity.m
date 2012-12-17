
function sweep_results = sweep_sensitivity(dt, sim_time, rotor_arm_len, rotor_arm_mass, delay, pgain, igain, dgain, pvgain, ivgain, dvgain)

num = 15

min_x = 0.1;
max_x = 2.5;
min_y = 0.5;
max_y = 2.5;
min_z = 0;
max_z = 5;

y_range = linspace(0.5, 2.5, num);
x_range = linspace(0.1, 2.5, num);
sensitivities = zeros(num,num);

counter_y = 0
for pvgain = y_range

	counter_y++;
	counter = 0;

	for dvgain = x_range
		counter++
		if (counter > 1 && sensitivities(counter_y, counter-1) < .05)
			sensitivities(counter_y, counter) = 0;
		else
			try
				sensitivities(counter_y, counter) = multicopter_sim(dt, sim_time, rotor_arm_len, rotor_arm_mass, delay, pgain, igain, dgain, pvgain, ivgain, dvgain);
			catch
				printf("Unstable system, unable to calculated sensitivity");
				sensitivities(counter_y, counter) = 0;
			end_try_catch
		endif
	endfor

endfor

legend_text = cell(1, length(y_range));

for i = 1:length(y_range)
	legend_text(1,i) = sprintf("%f", y_range(i));
endfor

figure(2)
clf
plot(x_range, sensitivities)
legend(legend_text);
axis([min_x max_x min_z max_z]);

filename = sprintf("plotx_delay_%f.png", delay);
print(filename);

figure(3)
clf
plot(y_range, sensitivities)
legend(legend_text);
axis([min_ys max_y min_z max_z]);

filename = sprintf("ploty_delay_%f.png", delay);
print(filename);

figure(4)
clf
mesh(x_range, y_range, sensitivities)
axis([min_x max_x min_y max_y min_z max_z]);
xlabel("D Gain");
ylabel("P Gaid");

filename = sprintf("plotxyz_delay_%f.png", delay);
print(filename);

# colormap([summer(); flipud(summer())]);
colormap([jet(); flipud(jet())]);
caxis([0.8 2.5]);
axis
colorbar;

endfunction