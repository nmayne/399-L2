package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import L2.PID;

public class iKinematics_b{
	
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
	
	public static void ikine(double X, double Y) {
		System.out.printf("X: %.2f Y: %.2f", X, Y);
		
		int[] offset = {62, 69};			// gear ratio offset
		double a;
		double b;
		double c;
		double d;
		double det; 						// determinant of J matrix; det = ad - bc
		double coeff;
		double[] q = {0, 0};				// change in angles
		double[] L = {72, 72};			// length of each arm
		double[] x = {144,  X};			// end effector and target x positions
		double[] y = {0.0,  Y};			// end effector and target y positions
		double[] angle = {0, 0};
		double deltaX = x[1] - x[0];
		double deltaY = y[1] - y[0];

		// set up motors
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.D);
		UnregulatedMotor M2 = new UnregulatedMotor(MotorPort.A);
 		// set up PID controllers
 		PID PIDController1 = new PID(M1);
 		PID PIDController2 = new PID(M2);
 		// first joint
		
		while((Math.abs(deltaX) > 3) || (Math.abs(deltaY) > 3)) {
			// Analytical Jacobian matrix for absolute inverse kinematics:
			// J = 	[ 	-sin(deltaX) 	-sin(deltaY) ]
			// 		[	cos(deltaX) 		cos(deltaY)  ]
			// J inverse =  (1/det)	[  cos(deltaY) 	sin(deltaY)  ] = (1/(ad - bc)) *	[  d	 -b ]
			// 						[ -cos(deltaX) 	-sin(deltaX) ]	   				[ -c  a	]
			a = (-1) * Math.sin(deltaX);
			b = (-1) * Math.sin(deltaY);
			c = Math.cos(deltaX);
			d = Math.cos(deltaY);
			det = (a * d) - (b * c);
			coeff = 1 / det;
			q[0] = q[0] + (coeff * (d - b));
			q[1] = q[1] + (coeff * (a - c));
//	 		// store new end effector positions
			x[0] = (L[0] * Math.cos(q[0])) + (L[1] * Math.cos(q[1]));
			y[0] = (L[0] * Math.sin(q[0])) + (L[1] * Math.sin(q[1]));
//			System.out.printf("x: %.2f y: %.2f\n", x[0], y[0]);
			deltaX = x[1] - x[0];
			deltaY = y[1] - y[0];
		}
		for (int i = 0; i < 2; i++) {
			angle[i] = Math.toDegrees(q[i]) % 180;
			if (angle[i] < 0) {
				angle[i] += 180;
			}
//			angle[i] = angle[i] * (-1);
		}
		// use forward kinematics to move to calculate new end effector position
 		Thread motor1 = new Thread(new Motor(PIDController1, offset[0] * angle[0], offset[0], 3, 0, 2.5));
 		Thread motor2 = new Thread(new Motor(PIDController2, offset[1] * angle[1], offset[1], 3, 0, 2.5));
 		// start threads
 		motor1.start();
 		motor2.start();
		
		System.out.printf("angle 0: %.0f angle 1: %.0f\n", (-1) * angle[0], (-1) * angle[1]);
		Delay.msDelay(2000);	
		Button.waitForAnyEvent();
	}
}

