/*
 * Nicholas Mayne & Laura Petrich, University of Alberta © 2017.
 */

package L2;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.utility.Stopwatch;

public class PID {
	
	UnregulatedMotor M;						// the motor
	Stopwatch timer;							// the timer
	double offset; 							// offset of motor
	double error ;							// error
	double prevError;						// error from previous sample
	double derivative;						// the derivative error
	double integral;							// the integral error
	double delta_t;							// delta t
	double elapsed_t;						// elapsed t
	double theta;							// change in theta
//	double[] rad = new double[1];			// radian conversion of theta
	int tacho;								// tachometer reading
	int power;								// power to motor
	int powerMax = 100;						// maximum power setting
	int prevPower;							// power from previous sample
	int sampleRate = 10;						// enforced minimum sample rate, 
	int timeout = 100000;					// timeout after x ms

	public PID(UnregulatedMotor motor) {
		M = motor;
		timer = new Stopwatch();
	}
	
	public void angle(double target, int Offset, double Kp, double Ki, double Kd) {
		
		offset = Offset;	
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
//			System.out.printf("error: %.2f power: %d\n", error, Math.min(power, 100));
			integral = integral + (error * delta_t/100);
			derivative = (error - prevError) / (delta_t/100);
			power = (int) Math.abs((Kp * error) + (Ki * integral) + (Kd * derivative));
			M.setPower(isPos(error) * (Math.min(powerMax, power)));
			while(timer.elapsed()<sampleRate ){}		// enforce minimum sample rate
            // get delta t and error
			delta_t = timer.elapsed();
			elapsed_t = elapsed_t + delta_t;
			timer.reset();
			prevError = error;
			tacho = M.getTachoCount();
			error = target - tacho;
            if(elapsed_t > timeout){System.out.print("TIMEOUT\n"); break;}
		}
		M.setPower(0); 						// halt motor
		// get actual angle moved 
//		double tacho = M.getTachoCount();
//		rad[0] = Math.toRadians(tacho/offset);
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