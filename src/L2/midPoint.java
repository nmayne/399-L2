package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import L2.iKinematics_b;

public class midPoint {
	
	static UnregulatedMotor[] motors = {
		new UnregulatedMotor(MotorPort.D),	// left motor motors[0]
		new UnregulatedMotor(MotorPort.A)	// right motor motors[1]
	};
	static int[] offset = {61,69};
	static int tachoCount;
	static int count = 0;
	static int command;
	static double a;
	static double b;
	static double c;
	static double[] x = new double[3];
	static double[] y = new double[3];
	static double cosA;
	static double distance;
	static double[] mid = new double[2];
	static double[] L = {72, 72};
	static double[][] targetAng = new double[3][3];

	public static void main(String[] args) {
		System.out.println("Press a button\n");   
		
		for(int i=0; i<2; i++){
			motors[i].setPower(100);
			motors[i].stop();
			motors[i].resetTachoCount();
		}

		while (count < 3) {
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
						System.out.println("Origin marked.\n");
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
					break;
				default:
					break;		
			} 
		}
		// calculate distance between first point and second point
		for (int j = 1; j < 3; j++) {
			x[j] = (L[0] * Math.cos(Math.toRadians(targetAng[j-1][0]))) + (L[1] * Math.cos(Math.toRadians(targetAng[j-1][1])));
			y[j] = (L[0] * Math.sin(Math.toRadians(targetAng[j-1][0]))) + (L[1] * Math.sin(Math.toRadians(targetAng[j-1][1])));
			System.out.printf("\npoint %d: \n(x, y) = (%.2f, %.2f)\n", j, x[j], y[j]);
		}
		// find midpoint between 2nd and 3rd point
		mid[0] = (x[1] + x[2])/2;  	// x coordinate at midpoint
		mid[1] = (y[1] + y[2])/2;	// y coordinate at midpoint
		System.out.printf("\nMidpoint\nx: %.2f y: %.2f\n", mid[0], mid[1]);	
		// iKinematics starts from home position, so find where midpoint is relative to [144, 0]
		double X = x[0] - (x[2] - mid[0]);
		double Y = y[0] - (y[2] - mid[1]);
		// Call invKine from where arm is currently and move to (x, y) midpoint
		try {
			iKinematics_b.ikine(X, Y);
		} catch (Exception e) {
			// catch block
			e.printStackTrace();
		}
		for(int i=0; i<2; i++){
			motors[i].close();
		}
	}
}