package L2;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
//import lejos.utility.Stopwatch;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import L2.PIDCfKine;
	
public class fKine {
	
	static class Motor implements Callable<double[]> {
		PIDCfKine PID;
		int target;
		double Kp;
		double Ki;
		double Kd;
		double coord[] = new double[2];
		int powerMax;
		int offset;
//		Stopwatch timer = new Stopwatch();
		
		Motor(PIDCfKine PID, int target, int offset, double Kp, double Ki, double Kd, int powerMax) {
			this.PID = PID;
			this.offset = offset;
			this.target = target;
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			this.powerMax = powerMax;
		}
		
		public double[] call() {
//			timer.reset();
			coord = PID.PID(target, offset, Kp, Ki, Kd, powerMax, false);
//			System.out.println("\nTimer: " + timer.elapsed());
			return coord;
		}
	}	

	public static void fkine(int A, int B) {
		int[] target = {A, B};
		int[] offset = {62, 69};
	    double[] p1 = null;
	    double[] p2 = null;
		double[] homeBase = {-72, 0};
		double[] power = {100, 100};
		double[] e = {-0.08604, 1.238, 1.272, -0.2078};
        double[] Kp = {0.0004637, -0.08011, 5.18};
        double[] Kd = {-0.0001348, 0.02578, 0.6731};	
		double[][] Kval = new double[3][2];
//		if (target[1] != 0) {
//			double x = Math.abs(target[0]/target[1]);
//			double powerRatio = (e[0]*Math.exp(e[1]*x) + e[2]*Math.exp(e[3]*x));
//			power[1] = 100;
////			power[0] = 50;
//			power[0] = Math.abs(power[1]*powerRatio);
//			System.out.printf("power0: %.0f", power[0]);
//			if (powerRatio > 1) {
//				power[0] = (power[1]/power[0])*100; // scale to within range [0 100]
//				System.out.printf("power0: %.0f", power[0]);
//			}
//		}
//		else { 
//			power[1] = 0; 
//			power[0] = 100; 
//		}

		// calculate Kp and Kd values for each motor
		for (int i = 0; i < 2; i++) {			
			Kval[0][i] = (Kp[0] * Math.pow(Math.abs(power[i]), 2)) + (Kp[1] * Math.abs(power[i])) + Kp[2] + 5.0;
			Kval[1][i] = 0;
			Kval[2][i] = (Kd[0] * Math.pow(Math.abs(power[i]), 2)) + (Kd[1] * Math.abs(power[i])) + Kd[2] + 0.1;
		}
		// check the values
//		System.out.println("\nPowerRatio: " + powerRatio + "\nJ1 Power: " + power[0] + "\nJ2 Power: " + power[1]); 
//		System.out.println("\nKp1: " + Kval[0][0] + "\nKd1: " + Kval[2][0] + "\nKp2: " + Kval[0][1] + "\nKd2: " + Kval[2][1]);   
		// set up motors
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.D);
		UnregulatedMotor M2 = new UnregulatedMotor(MotorPort.A);
		// set up PID controllers
		PIDCfKine PIDController1 = new PIDCfKine(M1);
		PIDCfKine PIDController2 = new PIDCfKine(M2);
		
		ExecutorService executor = Executors.newFixedThreadPool(3);
		// first joint
	    Callable<double[]> coord1 = new Motor(PIDController1, offset[0] * target[0], offset[0], Kval[0][1], Kval[1][1], Kval[2][1], (int) power[0]);
	    Future<double[]> future1 = executor.submit(coord1);
		try {
			p1 = future1.get();
		} catch (InterruptedException | ExecutionException e1) {
			// catch block
			e1.printStackTrace();
		}
//		System.out.printf("\n\nx: %.3f  y: %.3f", p1[0], p1[1]);
	    // second joint
		Callable<double[]> coord2 = new Motor(PIDController2, offset[1] * target[1], offset[1], Kval[0][1], Kval[1][1], Kval[2][1], (int) power[0]);
	    Future<double[]> future2 = executor.submit(coord2);
		try {
			p2 = future2.get();
		} catch (InterruptedException | ExecutionException e1) {
			// catch block
			e1.printStackTrace();
		}
		
		double J1x = (2.067 * p1[0]) + 0.3235;
		double J1y = (1.717 * p1[1]) + 0.9889;
	    	System.out.printf("J1x: %.2f J1y: %.2f", J1x, J1y);
	    	
	    	double EEx;
	    	double EEy;
	    	
	    if (target[1] == 0) {
	    		EEx = J1x;
	    		EEy = J1y;
	    	}
	    	else if (target[0] == 0){
	    		EEx = -72 + p2[0];
	    		EEy = p2[1];
	    	}
	    	else {
	    		EEx = J1x - p2[0] - 72;
	    		EEy = J1y; 
	    	}

	    	System.out.printf("\nEEx : %.0f EEy : %.0f" , EEx, EEy);
		
		
//	    for (int i = 0; i < 2; i++) {
//			endEffector[i] = p1[i] + p2[i] - homeBase[i];
//	    }
//		System.out.printf("\nEnd Effector Position: \n(%.2f, %.2f)\n", endEffector[0], endEffector[1]);
		executor.shutdown();
		Delay.msDelay(2000);	
		
	}
}

