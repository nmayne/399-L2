package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

import L2.PIDController;

public class Main {

	public static void main(String[] args) {
		String result;
		
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.A);
		PIDController PIDController1 = new PIDController(M1);

		int angles[] = {-360, -180, -90, 90, 180, 360};
			
		for(int i = 0; i < angles.length; i++) {
			result = PIDController1.PID(angles[i], 5, 1, 0, 300, 500);
			while(result==null) {}
			System.out.println(result);
			Delay.msDelay(100);
			result = null;
		}
//		Button.waitForAnyPress();	// press any button to exit
	}

}
