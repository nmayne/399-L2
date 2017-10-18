/*
 * Nicholas Mayne & Laura Petrich, University of Alberta © 2017.
 */

package L2;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.utility.Stopwatch;

public class PIDCInvKine {
	
	UnregulatedMotor M;			// the motor
	Stopwatch timer;				// the timer
	double[] inputCoord = {0,0};	// insert coordinate to find
	double error ;				// error
	double prevError;			// error from previous sample
	double derivative;			// the derivative error
	double integral;				// the integral error
	double delta_t;				// delta t
	double elapsed_t;			// elapsed t
	double theta;				// change in theta
	double rad;					// radian conversion of theta
	int offset;					// motor offset because of gear ratios
	int tacho;					// tachometer reading
	int power;					// power to motor
	int prevPower;				// power from previous sample
	int powerMin = 3;			// minimum power to get PID controller to finish when Kp = 1
	int sampleRate = 10;			// enforced minimum sample rate, 
	int timeout = 100000;		// timeout after x ms
	
	
	/*
	 * 
	 */
	public PIDCInvKine(UnregulatedMotor motor) {
		M = motor;
		timer = new Stopwatch();
	}
	
	/*
	 * PID controller
	 * target angle of rotation (deg)
	 */
	public double[] PID(int target, int Offset, double Kp, double Ki, double Kd, int powerMax, double[] input) {
				
		inputCoord = input;
		offset = Offset;													// offset from gear ratio
		error = target;													// error
		prevError = error;												// error from previous sample
		derivative = 0;													// the derivative error
		integral = 0;													// the integral error
		delta_t = 0;														// delta t
		elapsed_t = 0;													// elapsed t
		power = 0;														// power to motor

		M.resetTachoCount();
		timer.reset();

		while (error != 0 || prevError != 0){
			// PID
			integral = integral + (error * delta_t/100);
			derivative = (error - prevError) / (delta_t/100);
			power = (int) Math.abs((Kp * error) + (Ki * integral) + (Kd * derivative));
			M.setPower(isPos(error) * (Math.min(powerMax, powerMin + power)));

			while(timer.elapsed()<sampleRate){}
				delta_t = timer.elapsed();
				elapsed_t = elapsed_t + delta_t;
				timer.reset();
				prevError = error;
				tacho = M.getTachoCount();
				error = target - tacho;
				if(elapsed_t > timeout){break;}
			}
			M.setPower(0); 						// halt motor
			return inputCoord;
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
