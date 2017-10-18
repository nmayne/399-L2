/*
 * Nicholas Mayne & Laura Petrich, University of Alberta © 2017.
 */

package L2;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.utility.Stopwatch;

public class PIDController {
	
	UnregulatedMotor M;		// the motor
	Stopwatch timer;			// the timer
	double error ;			// error
	double prevError;		// error from previous sample
	double derivative;		// the derivative error
	double integral;			// the integral error
	double delta_t;			// delta t
	double elapsed_t;		// elapsed t
	double theta;			// change in theta
	double rad;				// radian conversion of theta
	int offset;
	double[] coord = {-74,0};	// starting coordinate [x, y] in millimeters
	double[] newCoord = {0,0};
	int tacho;				// tachometer reading
	int power;				// power to motor
	int prevPower;			// power from previous sample
	int powerMin = 3;		// minimum power to get PID controller to finish when Kp = 1
	int sampleRate = 10;		// enforced minimum sample rate, 
	int timeout = 100000;	// timeout after x ms
	
	
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
	public double[] PID(int target, int Offset, double Kp, double Ki, double Kd, int powerMax, Boolean Matlab) {
				
		double[][] rotMat = new double[2][2];								// store rotation matrix values
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
				if(Matlab) {System.out.print(elapsed_t + ", " + tacho + "; ");}	// Output for Matlab
				if(elapsed_t > timeout){break;}
			}
		
			M.setPower(0); 						// halt motor
			// get end effector position
			double tacho = M.getTachoCount();
			rad = ((Math.PI*tacho)/(180*offset))%(2*Math.PI);
//			System.out.printf("\nradians: %.3f", rad);
			// calculate rotation matrix from true angle position change
			rotMat[0][0] = Math.cos(rad);
			rotMat[1][1] = rotMat[0][0];
			if (rad > 0) { 	// clockwise 
				rotMat[0][1] = Math.sin(rad);
				rotMat[1][0] = Math.sin(rad) * (-1);
			}
			else { 			// counterclockwise
				rotMat[0][1] = Math.sin(Math.abs(rad)) * (-1);
				rotMat[1][0] = Math.sin(Math.abs(rad));				
			}
			// print rotation matrix for error testing
//			for (int i = 0; i < 2; i++) {
//				System.out.println("\n");
//				for (int j = 0; j < 2; j++) {
//					System.out.printf("%.3f; ", rotMat[i][j]);
//				}
//			}

//			// calculate new [x y] position
			newCoord[0] = (rotMat[0][0]*coord[0]) + (rotMat[0][1]*coord[1]);
			newCoord[1] = (rotMat[1][0]*coord[0]) + (rotMat[1][1]*coord[1]);
//			System.out.printf("\nCoord x: %.3f", newCoord[0]);
//			System.out.printf("\nCoord y: %.3f", newCoord[1]);
			return newCoord;
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
