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
	public String PID(int target, double Kp, double Ki, double Kd, int speedMax, int inputRange) {
				
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
			integral = integral + (error*time);		// compute integral
			derivative = (error - prevError) / time; // compute derivative
			prevError = error;						// save the error

			// compute and apply power
			power = Math.abs((Kp * error) + (Ki * integral) + (Kd * derivative));	
			M.setPower(map((isPos(error) *Math.min(power, inputRange)),-inputRange, inputRange, -speedMax, speedMax));
			System.out.println(error);
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
	
	/*
	 * https://stackoverflow.com/questions/7505991/arduino-map-equivalent-function-in-java
	 */
	int map(double x, double in_min, double in_max, double out_min, double out_max)
	{
	  return (int) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	}
}