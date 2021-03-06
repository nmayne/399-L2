/*
 * Nicholas Mayne & Laura Petrich, University of Alberta © 2017.
 */
package L2;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.utility.Stopwatch;

public class C_PIDController {
	UnregulatedMotor M;		// the motor
	Stopwatch timer;			// the timer
	String data;				// the pv and time data
	double prevError;		// error from previous sample
	double derivative;		// the derivative error
	double integral;			// the integral error
	double delta_t;			// delta t
	double elapsed_t;		// elapsed t
	int error;				// error
	int pv;					// process variable: tachometer reading
	int power;				// power to motor
	int prevPower;			// power from previous sample
	int sampleRate = 10;		// enforced minimum sample rate, 
	
	public C_PIDController(UnregulatedMotor motor) {
		M = motor;
		timer = new Stopwatch();
	}
	
	public String PID(int sp, double Kp, double Ki, double Kd, int powerMax, int timeout) {
		data = "";						// process variable and time data
		error = sp;						// error
		prevError = error;				// error from previous sample
		derivative = 0;					// the derivative error
		integral = 0;					// the integral error
		delta_t = 0;						// delta t
		elapsed_t = 0;					// elapsed t
		power = 0;						// power to motor
		
		M.resetTachoCount();
		timer.reset();

		while (error != 0 || prevError != 0){
			integral = integral + (error * delta_t/100);
			derivative = (error - prevError) / (delta_t/100);
			
			power = (int) Math.abs((Kp * error) + (Ki * integral) + (Kd * derivative));
			M.setPower(isPos(error) * (Math.min(powerMax, power)));
			
			while(timer.elapsed()<sampleRate){}			// enforce minimum sample rate
			
			// get delta t and error
			delta_t = timer.elapsed();
			elapsed_t = elapsed_t + delta_t;
			timer.reset();
			prevError = error;
			pv = M.getTachoCount();
			error = sp - pv;
			
			data = data + elapsed_t + ", " + pv + "; ";	// update data log
			
			if(elapsed_t > timeout){System.out.print("TIMEOUT\n"); break;}
		}
		M.setPower(0);// halt motor
		System.out.print("E: " + error + "\n\n");
//		return Integer.toString(error);
		return data;
	}
	
	// get the sign of a double
	int isPos(double e) {
		if (e < 0){
			return -1;
		} else {
			return 1;
		}	
	}
}