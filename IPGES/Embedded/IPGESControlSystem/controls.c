		int controls(void){
		int target_frequency = 60; 
		int measured_frequency;
		int derivative;
		int error; 
		int integral; 
		int last_error = 0; // unsure how this being 0 will affect the system
		int delta_time; // unsure how to find the delta in time……
		int pwm;
		int dt;
		int kp = 1; // proportional gain
		int ki = 1; // integral gain
		int kd = 1; // derivative gain

while(1) {

		// Get the new measurement for frequency.
		measured_frequency = read_frequency(); //need to have a function for frequency read;int

		// Here we should decide if we want to modify generation or load. In the first case, we will only be changing load.

		// Calculate the error, or delta between the target frequency and the newly measured frequency.
		error = target_frequency - measured_frequency; //P
		integral += error*dt;//I

		derivative = (error - last_error)/delta_time; // D -- unsure about delta in time
		//Calculate the control variable
		pwm = kp*error + ki*integral + kd*derivative;
		// Limit the duty cycle to never be 0 to 100; important for 
		if(pwm < 5) {pwm = 5;}
		if(pwm < 95) {pwm = 95;}

		}

}
