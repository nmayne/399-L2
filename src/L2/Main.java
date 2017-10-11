package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

import java.util.Random;

import L2.PIDController;

public class Main {

	public static void main(String[] args) {
		String result;
		
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.A);
		PIDController PIDController1 = new PIDController(M1);
		Random rand = new Random();
		
//		result = PIDController1.PID(90, 2, 1, 0, 100);
//		System.out.println(result);
//		Delay.msDelay(1000);
//		Button.waitForAnyPress();	// press any button to exit
		
//		int angles[] = {90};
		int angles[] = {-180, -90, 90, 180};			
		
//		double Kp = 7;
		double Ki = 0;
		double Kd = 0;
		
		for(double Kp = 1; Kp < 15; Kp++) {
//		for(double Kd = 100; Kd < 1000; Kd = Kd + 10) {

				for(int i = 0; i < angles.length; i++) {
					System.out.printf("P:%.1f I:%.1f D:%.1f\n", Kp, Ki, Kd);

//					result = PIDController1.PID(rand.nextInt()%360, Kp, Ki, Kd, 100);
					result = PIDController1.PID(angles[i], Kp, Ki, Kd, 100);

					System.out.println(result);
					Delay.msDelay(1000);
//					Button.waitForAnyPress();	// press any button to exit
				}
			}

		
		
	}
}
