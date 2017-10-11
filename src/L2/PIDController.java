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
		delta_t = 1;			// delta t
		power = 0;			// power to motor
		
		M.resetTachoCount();
		timer.reset();
		
		while (error != 0 || prevError != 0){
			integral = integral + (error * delta_t);
			derivative = (error - prevError) / delta_t;
			power = (int) (Math.min(powerMax, Math.abs((Kp * error) + (Ki * integral) + (Kd * derivative))));
			M.setPower(isPos(error) * power);
			Delay.msDelay(10);					// this delay! grrr, works reasonably well when < 15
												// the dervative only seems to have a meaningful effect
												// when it is removed altogether, but removing it altogether
												// causes bad behaviour in otherways, and it is also unclear
												// why the derivative is not functioning meaningfully when
												// there is a delay, even of as little as 1 ms!
			
			// get error and delta t
			prevError = error;
			error = target-M.getTachoCount();
			delta_t = timer.elapsed();
			timer.reset();
		}
		
		M.setPower(0); // halt motor
		Delay.msDelay(500);					// wait for motor to fully halt
		error = target-M.getTachoCount();	// get error again because the motor has possibly moved
											// a little after we exited the while loop
		return ("Error: " + error + "\nTacho: " + M.getTachoCount() + "\n");
	}
	
	/*
	 * is e pos or neg? Used to get the direction of the error... but come to think of it the Math.abs above might be 
	 * the reason things aren't working for me. Try removing that, I can't recall why I did that in the first place!
	 * Oooooh, because the clipping using Math.min currently needs an unsigned magnitude to work properly... maybe
	 * there is a better way of dividing that up... but really, the direction of the error should be the direction
	 * of the power so that we head closer toward the target every time unless we overshoot.
	 */
	int isPos(double e) {
		if (e < 0){
			return -1;
		} else {
			return 1;
		}	
	}
}