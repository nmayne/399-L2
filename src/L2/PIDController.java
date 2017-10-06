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
	double time = 0;			// delta t
	double power = 0;		// power to motor

	
	/*
	 * need a motor
	 */
	public PIDController(UnregulatedMotor motor) {
		M = motor;
		timer = new Stopwatch();
	}
	
	
	/*
	 * PID controller
	 * target angle of rotation (deg)
	 */
	public String PID(int target, int Kp, int Kd, int Ki, int speedMax) {
				
		error = target;	// error
		prevError = 1;	// error from previous sample
		derivative = 0;	// the derivative error
		integral = 0;	// the integral error
		time = 0;		// delta t
		power = 0;		// power to motor
		
		M.resetTachoCount();
		timer.reset();
		
		while (Math.abs(prevError) > 0){
			time = timer.elapsed();
			timer.reset();
						
			error = target-M.getTachoCount();		// get proportional error
			derivative = (error - prevError) / time; // compute derivative
			integral = integral + (error*time);		// compute integral
	
			// compute and apply power
			power = Math.abs((Kp * error) + (Kd * derivative) + (Ki * integral));	
			M.setPower((int) (isPos(error) * Math.min(power, speedMax)));

			// save the error
			prevError = error;
			
			Delay.msDelay(25);
		}			

		M.setPower(0); // halt motor
		return ("Error: " + error + "\nTacho: " + M.getTachoCount() + "\n");
	}
	
	/*
	 * is e pos or neg?
	 */
	int isPos(double e) {
		if (e < 0){
			return -1;
		} else {
			return 1;
		}	
	}
}