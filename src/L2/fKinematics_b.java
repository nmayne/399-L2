package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

import L2.PID;
	
public class fKinematics_b {
	
	static class Motor implements Runnable {
		PID PID;
		double target;
		double Kp;
		double Ki;
		double Kd;
		double[] rad;
		int powerMax;
		int offset;
		
		Motor(PID PID, double target, int offset, double Kp, double Ki, double Kd) {
			this.PID = PID;
			this.offset = offset;
			this.target = target;
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
		}
		
		public void run() {
			PID.angle(target,offset, Kp, Ki, Kd);
		}
	}	

	public static void fkine(int A, int B) {
		int[] offset = {62, 69};
		double[] target = {(-1) * A, (-1) *B}; // account for different motor rotation
		double[] L = {72, 72};
//		double[] power = {100, 100};
//        double[] Kp = {0.0004637, -0.08011, 5.18};
//        double[] Kd = {-0.0001348, 0.02578, 0.6731};	
//		double[][] Kval = new double[3][2];

		// calculate Kp and Kd values for each motor
//		for (int i = 0; i < 2; i++) {			
//			Kval[0][i] = (Kp[0] * Math.pow(Math.abs(power[i]), 2)) + (Kp[1] * Math.abs(power[i])) + Kp[2] + 5.0;
//			Kval[1][i] = 0;
//			Kval[2][i] = (Kd[0] * Math.pow(Math.abs(power[i]), 2)) + (Kd[1] * Math.abs(power[i])) + Kd[2] + 0.1;
//		}
		// check the values
//		System.out.println("\nKp1: " + Kval[0][0] + "\nKd1: " + Kval[2][0] + "\nKp2: " + Kval[0][1] + "\nKd2: " + Kval[2][1]);   
		// set up motors
 		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.D);
 		UnregulatedMotor M2 = new UnregulatedMotor(MotorPort.A);
 		// set up PID controllers
 		PID PIDController1 = new PID(M1);
 		PID PIDController2 = new PID(M2);
 		// first joint
 		Thread motor1 = new Thread(new Motor(PIDController1, offset[0] * target[0], offset[0], 3, 0, 2.5));
 		// second joint
 		Thread motor2 = new Thread(new Motor(PIDController2, offset[1] * target[1], offset[1], 3, 0, 2.5));
 		// start threads
 		motor1.start();
 		motor2.start();
 
		double x = (L[0] * Math.cos(Math.toRadians(target[0]))) + (L[1] * Math.cos(Math.toRadians(target[1])));
		double y = (-1) * (L[0] * Math.sin(Math.toRadians(target[0]))) + (L[1] * Math.sin(Math.toRadians(target[1])));
		System.out.printf("(x, y) = (%.0f, %.0f)\n", x, y);
		
		Delay.msDelay(2000);	
		Button.waitForAnyEvent();

	}
}

