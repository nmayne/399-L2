package L2;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import L2.PIDCInvKine;
	
public class invKine{
	
	static class Motor implements Callable<double[]> {
		PIDCInvKine PID;
		int angle;
		int powerMax;
		int offset;
		double Kp;
		double Ki;
		double Kd;
		double[] input = new double[2];
		double[] angles = new double[2];

		Stopwatch timer = new Stopwatch();
		
		Motor(PIDCInvKine PID, int angle, int offset, double Kp, double Ki, double Kd, int powerMax, double[] input) {
			this.PID = PID;
			this.offset = offset;
			this.angle = angle;
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			this.powerMax = powerMax;
			this.input = input;
		}
		
		public double[] call() {
//			timer.reset();
			angles = PID.PID(angle, offset, Kp, Ki, Kd, powerMax, input);
//			System.out.println("\nTimer: " + timer.elapsed());
			return angles;
		}
	}	
	
	public static void ikine(int x, int y) {
		int X = x;
		int Y = y;
		int[] angle = {0, 0};
		int[] offset = {41, 72};
		double cosA;
		double a;
		double b;
		double c;
		double A;
		double[] power = {0, 0};
		double[] e = {-0.08604, 1.238, 1.272, -0.2078};
		double[] p = {3.259, 0.1189, -0.002427, 1.354e-05};
		double[] d = {-3.806, 0.2893, -0.004203,  1.888e-05};
		double[][] Kval = new double[3][2];
		double[][] point = {	{  1	,  0  }, 
							{ -74, 0  }, 
							{ X, Y }}; 

		// calculate a, b, c (distance between points)
		a = Math.sqrt(Math.pow(point[2][0] - point[1][0], 2) + Math.pow(point[2][1] - point[1][1], 2));
		b = Math.sqrt(Math.pow(point[2][0] - point[0][0], 2) + Math.pow(point[2][1] - point[0][1], 2));
		c = Math.sqrt(Math.pow(point[1][0] - point[0][0], 2) + Math.pow(point[1][1] - point[0][1], 2));
		// use law of cosines --> cos(A) = (b^2 + c^2 - a^2) / 2bc to calculate angle from home to end effector
		cosA = (Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2)) / (2 * b * c);
		A = Math.toDegrees(Math.acos(cosA));
		
//		System.out.printf("a: %.2f b: %.2f c: %.2f\n", a, b, c);
//		System.out.printf("cosA: %.2f angle: %.2f\n", cosA, A);
		
		// calculate appropriate angles for each joint
		angle[0] = (int)(2*A) / 3;
		angle[1] = (int)(A - angle[0]);
		// PID calculations
		if (angle[1] != 0) {
			double n = Math.abs(angle[0]/angle[1]);
			double powerRatio = (e[0]*Math.exp(e[1]*n) + e[2]*Math.exp(e[3]*n));
			power[1] = 100;
			power[0] = power[1]*powerRatio;
			if (powerRatio > 1) {
				power[0] = (power[1]/power[0])*100; // scale to within range [0 100]
			}
		}
		else { 
			power[1] = 0; 
			power[0] = 100; 
		}
		// calculate Kp and Kd values for each joint
		for (int i = 0; i < 2; i++) {
			Kval[0][i] = p[0] + (p[1] * power[i]) + (p[2] * Math.pow(power[i], 2)) + (p[3] * Math.pow(power[i], 3));
			Kval[1][i] = 0;
			Kval[2][i] = d[0] + (d[1] * power[i]) + (d[2] * Math.pow(power[i], 2)) + (d[3] * Math.pow(power[i], 3));
		}   
		// set up motors
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.D);
		UnregulatedMotor M2 = new UnregulatedMotor(MotorPort.A);
		// set up PID controllers
		PIDCInvKine PIDController1 = new PIDCInvKine(M1);
		PIDCInvKine PIDController2 = new PIDCInvKine(M2);
		// execute threads
		ExecutorService executor = Executors.newFixedThreadPool(3);
	    Callable<double[]> angle1 = new Motor(PIDController1, offset[0] * angle[0], offset[0], Kval[0][0], Kval[1][0], Kval[2][0], (int) power[0], point[2]);
	    Future<double[]> future1 = executor.submit(angle1);
	    double[] p1 = null;
		try {
			p1 = future1.get();
		} catch (InterruptedException | ExecutionException e1) {
			// catch block
			e1.printStackTrace();
		}
		System.out.printf("\n\nx: %.2f  y: %.2f \nangle1: %d", p1[0], p1[1], angle[0]);
	    
		Callable<double[]> angle2 = new Motor(PIDController2, offset[1] * angle[1], offset[1], Kval[0][1], Kval[1][1], Kval[2][1], (int) power[0], point[2]);
	    Future<double[]> future2 = executor.submit(angle2);
	    double[] p2 = null;
		try {
			p2 = future2.get();
		} catch (InterruptedException | ExecutionException e1) {
			// catch block
			e1.printStackTrace();
		}
		System.out.printf("\n\nx: %.2f  y: %.2f \nangle2: %d", p2[0], p2[1], angle[1]);
		executor.shutdown();
		Delay.msDelay(2000);			
	}
}

