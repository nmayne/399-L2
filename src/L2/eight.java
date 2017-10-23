package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import L2.iKinematics_b;

public class eight {
	
	static UnregulatedMotor[] motors = {
		new UnregulatedMotor(MotorPort.D),	// left motor motors[0]
		new UnregulatedMotor(MotorPort.A)	// right motor motors[1]
	};
	static int[] offset = {61,69};
	static int tachoCount;
	static int count = 0;
	static int command;
	static int n = 2;
	static double a;
	static double b;
	static double c;
	static double X;
	static double Y;
	static double diff;
	static double[] x = new double[3];
	static double[] y = new double[3];
	static double cosA;
	static double distance;
	static double[] mid = new double[2];
	static double[] L = {72, 72};
	static double[][] targetAng = new double[n][n];

	public static void main(String[] args) {
		System.out.println("Press a button\n");   
		
		for(int i=0; i<2; i++){
			motors[i].setPower(100);
			motors[i].stop();
			motors[i].resetTachoCount();
		}

		while (count < n) {
			command = Button.readButtons();
			switch(command) {
				case 16:
					// left button pressed
            			motors[1].forward();
            			Delay.msDelay(100);
            			motors[1].stop();
            			break;
				case 1:
					// up button pressed
					motors[1].backward();
					Delay.msDelay(100);
        				motors[1].stop();			
					break;
				case 4:
					// right button pressed
        				motors[0].forward();
        				Delay.msDelay(100);	
        				motors[0].stop();
        				break;
				case 8:
					// down button pressed
					motors[0].backward();
					Delay.msDelay(100);
        				motors[0].stop();			
					break;			
				case 2:
					// enter button pressed
					Delay.msDelay(500);	// keeps down error 
					if (count == 0) {
						System.out.println("Point 1 marked.\n");
						for(int i=0; i<2; i++){
							motors[i].resetTachoCount();
						}
						x[0] = 144.0;
						y[0] = 0.0;
					}
					else {
						System.out.printf("Point %d marked.\n", count + 1);
						for(int i=0; i<2; i++){
							targetAng[count-1][i] = (-1) * motors[i].getTachoCount() / offset[i];
							System.out.printf("targetAng %d: %.2f\n", i, targetAng[count-1][i]);
						}
					}
					count++;
					Delay.msDelay(500);	// keeps down error 
					break;
				case 32:
					// escape button pressed
					count = n;
					break;
				default:
					break;		
			} 
		}
		// calculate point coordinates
		for (int j = 1; j < n; j++) {
			x[j] = (L[0] * Math.cos(Math.toRadians(targetAng[j - 1][0]))) + (L[1] * Math.cos(Math.toRadians(targetAng[j - 1][1])));
			y[j] = (L[0] * Math.sin(Math.toRadians(targetAng[j - 1][0]))) + (L[1] * Math.sin(Math.toRadians(targetAng[j - 1][1])));
			System.out.printf("\npoint 1: \n(x, y) = (%.2f, %.2f)\n", x[j], y[j]);
		}
		X = x[1] + 5; // x[0] upper bound
		if (n == 2) { 	// straight line between 2 points
			Y = (((X - x[0]) / x[1] - x[0]) * (y[1] - y[0])) + y[0];
			// with respect to origin
			X = 5; // move x incrementally by this amount
			Y = Y - x[1]; // move y incrementally by corresponding amount
			// find difference in x
			diff = Math.abs(x[1] - x[0]);
			// move in straight line between the point
			for (int k = 0; k < diff; k += 5) {
			// Call invKine from where arm is currently and move to (x, y) midpoint
				try {
					iKinematics_b.ikine(X, Y);
				} catch (Exception e) {
					// catch block
					e.printStackTrace();
				}
			}
		}
		else {
			// curved line of n points
			// TO DO fit a piecewise polynomial between each point
			System.out.println("poly\n");
		}

		for(int i=0; i<2; i++){
			motors[i].close();
		}
	}
}