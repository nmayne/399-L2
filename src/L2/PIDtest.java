package L2;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;
import L2.PIDController;
	
class PIDtest {
	
	static class Motor implements Runnable {
		PIDController PID;
		int target;
		double Kp;
		double Ki;
		double Kd;
		int powerMax;
		Stopwatch timer = new Stopwatch();
		
		Motor(PIDController PID, int target, double Kp, double Ki, double Kd, int powerMax) {
			this.PID = PID;
			this.target = target;
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			this.powerMax = powerMax;
		}
		
		@Override
		public void run() {
			timer.reset();
			PID.PID(target, Kp, Ki, Kd, powerMax, false);
			System.out.println("\nTimer: " + timer.elapsed());
		}
	}	

	public static void main(String[] args) {
		//////////////////////////////////
		int[] target = {45, 90};
		//////////////////////////////////
		int[] offset = {41, 72};
		double[] power = {0, 0};
		double[] e = {-0.08604, 1.238, 1.272, -0.2078};
		double[] p = {3.259, 0.1189, -0.002427, 1.354e-05};
		double[] d = {-3.806, 0.2893, -0.004203,  1.888e-05};
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
		System.out.println("\nPowerRatio: " + powerRatio + "\nJ1 Power: " + power[0] + "\nJ2 Power: " + power[1]); 
		System.out.println("\nKp1: " + Kval[0][0] + " Kd1: " + Kval[2][0] + "\nKp2: " + Kval[0][1] + " Kd2: " + Kval[2][1]);   
		// set up motors
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.D);
		UnregulatedMotor M2 = new UnregulatedMotor(MotorPort.A);
		// set up PID controllers
		PIDController PIDController1 = new PIDController(M1);
		PIDController PIDController2 = new PIDController(M2);
		// first joint
		Thread motor1 = new Thread(new Motor(PIDController1, offset[0] * target[0], Kval[0][0], Kval[1][0], Kval[2][0], (int) power[0]), "M1");
		// second joint
		Thread motor2 = new Thread(new Motor(PIDController2, offset[1] * target[1], Kval[0][1], Kval[1][1], Kval[2][1], (int) power[0]), "M2");
		// start threads
		motor1.start();
		motor2.start();
		Delay.msDelay(2000);
	}
}

