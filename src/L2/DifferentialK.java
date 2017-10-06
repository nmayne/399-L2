package L2;
import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;


public class DifferentialK {
	
	public static void main(String[] args){
		int Kp = 100;								// k proportional gain, > 0
		int Kd = 1;									// k differential gain, > 0
		int target = 1800;							// target angle of rotation (deg)
		int error = target;							// error term
		int prevError = 0;							// error from previous loop
		double errorDiff = 0;						// the error difference
		double start = 0;							// start time (overall)
		double elapsed = 0;							// elapsed time (overall)
		double time = 0;							// elapsed time (loop)
		double power = 0;							// power magnitude to motor

		int pp = 5;									// precise-move power, > 4
		long iRange = (Kp+Kd)*Math.abs(target);		// map range for input value
		long oRange = 100;							// map range for output value
		double threshold = Math.abs(target)*0.02;	// stopping threshold for getting close, e.g. *0.05

		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.A);
		M1.resetTachoCount();
		
		Stopwatch timer = new Stopwatch();
		start = timer.elapsed();
		
//		// oscillate
//		while (Math.abs(error) != 0 ){
//			time = timer.elapsed();
//			timer.reset();
//			
//			prevError = error;
//			error = target-M1.getTachoCount();
//			errorDiff = (error - prevError) / time;
//			
//			power = (Kp * error) + (Kd * errorDiff);
//			M1.setPower((int) power);
//		}
		
		// get close
		System.out.println("Rapid Move");
		while (Math.abs(error) > threshold){
			time = timer.elapsed();
			elapsed = elapsed + time;
			timer.reset();
			
			prevError = error;
			error = target-M1.getTachoCount();
			errorDiff = (error - prevError) / time;
			
			power = (Kp * error) + (Kd * errorDiff);	
			M1.setPower(map(power, -iRange, iRange, -oRange, oRange));
		}
		System.out.printf("Error: " + error + "\nTacho: " + M1.getTachoCount() + "\n\n");

		
		// get precise
		elapsed = elapsed + timer.elapsed();
		System.out.println("Precise Move");
		Delay.msDelay(100);
		error = target-M1.getTachoCount();
		while (error != 0){
			M1.setPower(isPos(error) * pp);
			error = target-M1.getTachoCount();
		}
		
		// halt motor
		M1.setPower(0);
		System.out.printf("Error: " + error + "\nTacho: " + M1.getTachoCount() + "\nTime: " + (elapsed-start));

		M1.close();
		
		Button.waitForAnyPress(); // press any button to exit

	}
	
	// is e pos or neg or 0?
	static int isPos(int e) {
		if (e > 0){
			return 1;
		} else if (e < 0) {
			return -1;
		} else {
			return 0;
		}
	}
	
	// map an input value from one range to another
	// https://stackoverflow.com/questions/7505991/arduino-map-equivalent-function-in-java
	static int map(double power, long in_min, long in_max, long out_min, long out_max)
	{
	  return (int) ((power - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	}
}