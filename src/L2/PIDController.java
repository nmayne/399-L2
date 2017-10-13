/*
 * Nicholas Mayne & Laura Petrich, University of Alberta © 2017.
 */

package L2;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;

public class PIDController {
	
	UnregulatedMotor M;		// the motor
	Stopwatch timer;			// the timer
	double error = 0;		// error
	double prevError = 0;	// error from previous sample
	double derivative = 0;	// the derivative error
	double integral = 0;		// the integral error
	double delta_t = 0;		// delta t
	int power = 0;			// power to motor
	int prevPower = 0;		// power from previous sample
	int powerMin = 10;		// minimum power to get PID controller to finish
	int sampleRate = 10;		// enforced minimum sample rate, 
							// > 1 ms to ensure derivative never divides by 0, and avoid oversample
	
	/*
	 * 
	 */
	public PIDController(UnregulatedMotor motor) {
		M = motor;
		timer = new Stopwatch();
	}
	
	/*
	 * PID controller
	 * target angle of rotation (deg)
	 */
	public String PID(int target, double Kp, double Ki, double Kd, int powerMax) {
				
		error = target;		// error
		prevError = error;	// error from previous sample
		derivative = 0;		// the derivative error
		integral = 0;		// the integral error
		delta_t = 0;			// delta t
		power = 0;			// power to motor

		M.resetTachoCount();
		timer.reset();

		while (error != 0 || prevError != 0){
			integral = integral + (error * delta_t/100);
			derivative = (error - prevError) / (delta_t/100);
			
			power = (int) Math.abs((Kp * error) + (Ki * integral) + (Kd * derivative));
			M.setPower(isPos(error) * (Math.min(powerMax, powerMin + power)));
			
			// get error and delta t
			//System.out.println("P: " + (int)(Kp * error) + "\nI: " + (int)(Ki * integral) + "\nD: " + (int)(Kd * derivative));
			Delay.msDelay(sampleRate);
			prevError = error;
			error = target-M.getTachoCount();
			delta_t = timer.elapsed();
			timer.reset();
		}
		
		M.setPower(0); 						// halt motor
		Delay.msDelay(500);					// wait for motor to fully halt
		error = target-M.getTachoCount();	// get error again because the motor has possibly moved
											// a little after we exited the while loop
		return ("Error: " + error + "\nTacho: " + M.getTachoCount() + "\n");
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