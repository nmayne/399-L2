package L2;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import L2.PIDController;
	
class PIDtest {
	
	static class Motor implements Callable<double[]> {
		PIDController PID;
		int target;
		double Kp;
		double Ki;
		double Kd;
		double coord[] = new double[2];
		int powerMax;
		int offset;
		Stopwatch timer = new Stopwatch();
		
		Motor(PIDController PID, int target, int offset, double Kp, double Ki, double Kd, int powerMax) {
			this.PID = PID;
			this.offset = offset;
			this.target = target;
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			this.powerMax = powerMax;
		}
		
		public double[] call() {
			timer.reset();
			coord = PID.PID(target, offset, Kp, Ki, Kd, powerMax, false);
//			System.out.println("\nTimer: " + timer.elapsed());
			return coord;
		}
	}	
	public static void main(String[] args) throws Exception{
		//////////////////////////////////
		int[] target = {0, -45};
//		int[] target = {90, 90};
		//////////////////////////////////
		int[] offset = {41, 72};
		double[] homeBase = {-74, 0};
		double[] power = {0, 0};
		double[] e = {-0.08604, 1.238, 1.272, -0.2078};
		double[] p = {3.259, 0.1189, -0.002427, 1.354e-05};
		double[] d = {-3.806, 0.2893, -0.004203,  1.888e-05};
		double[] endEffector = {0,0};
		double[][] Kval = new double[3][2];
		double x = Math.abs(target[0]/target[1]);
		double powerRatio = (e[0]*Math.exp(e[1]*x) + e[2]*Math.exp(e[3]*x));
		
		power[1] = 100;
		power[0] = power[1]*powerRatio;
		if (powerRatio > 1) {
			power[0] = (power[1]/power[0])*100; // scale to within range [0 100]
		}
		// calculate Kp and Kd values for each joint
		for (int i = 0; i < 2; i++) {
			Kval[0][i] = p[0] + (p[1] * power[i]) + (p[2] * Math.pow(power[i], 2)) + (p[3] * Math.pow(power[i], 3));
			Kval[1][i] = 0;
			Kval[2][i] = d[0] + (d[1] * power[i]) + (d[2] * Math.pow(power[i], 2)) + (d[3] * Math.pow(power[i], 3));
		}
		// check the values
//		System.out.println("\nPowerRatio: " + powerRatio + "\nJ1 Power: " + power[0] + "\nJ2 Power: " + power[1]); 
//		System.out.println("\nKp1: " + Kval[0][0] + "\nKd1: " + Kval[2][0] + "\nKp2: " + Kval[0][1] + "\nKd2: " + Kval[2][1]);   
		// set up motors
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.D);
		UnregulatedMotor M2 = new UnregulatedMotor(MotorPort.A);
		// set up PID controllers
		PIDController PIDController1 = new PIDController(M1);
		PIDController PIDController2 = new PIDController(M2);
		
		ExecutorService executor = Executors.newFixedThreadPool(3);
	    Callable<double[]> coord1 = new Motor(PIDController1, offset[0] * target[0], offset[0], Kval[0][0], Kval[1][0], Kval[2][0], (int) power[0]);
	    Future<double[]> future1 = executor.submit(coord1);
	    double[] p1 = future1.get();
//		System.out.printf("\n\nx: %.3f  y: %.3f", p1[0], p1[1]);
	    
		Callable<double[]> coord2 = new Motor(PIDController2, offset[1] * target[1], offset[1], Kval[0][1], Kval[1][1], Kval[2][1], (int) power[0]);
	    Future<double[]> future2 = executor.submit(coord2);
	    double[] p2 = future2.get();
		
	    for (int i = 0; i < 2; i++) {
			endEffector[i] = p1[i] + p2[i] - homeBase[i];
	    }
		System.out.printf("End Effector Position: \n(%.2f, %.2f)\n", endEffector[0], endEffector[1]);

		
		executor.shutdown();

		Delay.msDelay(2000);
		
	
	}

}

