package L2;
import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

public class ConstantK {
	
	public static void main(String[] args){
		int Kp = 6;									// k proportional gain, > 0
		int target = 90;							// target angle of rotation (deg)
		int error = target;							// error term
		int pp = 5;									// precise-move power, > 4
		long iRange = Kp*Math.abs(target);			// map range for input value
		long oRange = 100;							// map range for output value
		double threshold = Math.abs(target)*0.05;	// stopping threshold for getting close, e.g. *0.05

		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.A);
		M1.resetTachoCount();
		
		
		// oscillate
//		while (Math.abs(error) != 0 ){
//			error = target-M1.getTachoCount();
//			System.out.println(error);
//			M1.setPower(Kp*error);
//		}
		
		// get close
		while (Math.abs(error) > threshold){
			error = target-M1.getTachoCount();
			M1.setPower(map(Kp*error, -iRange, iRange, -oRange, oRange));
		}
		
		// get precise
		Delay.msDelay(100);
		error = target-M1.getTachoCount();
		while (error != 0){
			M1.setPower(isPos(error) * pp);
			error = target-M1.getTachoCount();
		}
		
		// halt motor
		M1.setPower(0);
		System.out.printf("Error: " + error + "\nTacho: " + M1.getTachoCount());
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
	static int map(long x, long in_min, long in_max, long out_min, long out_max)
	{
	  return (int) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	}
}