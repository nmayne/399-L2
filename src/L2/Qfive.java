package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

public class Qfive {
	
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
						Delay.msDelay(1000);	// keeps down error 
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
			distance = Math.sqrt(Math.pow(x[j] - x[0], 2) + Math.pow(y[j] - y[0], 2));
			System.out.printf("Distance to origin: %.2f\n", distance);
		}
	    	// calculate angle at intersection of two lines defined by 3 points
		// use law of cosines --> cos(A) = (b^2 + c^2 - a^2) / 2bc to calculate target at intersection 
		// calculate a, b, c
		a = Math.sqrt(Math.pow(x[2] - x[1], 2) + Math.pow(y[2] - y[1], 2));
		b = Math.sqrt(Math.pow(x[2] - x[0], 2) + Math.pow(y[2] - y[0], 2));
		c = Math.sqrt(Math.pow(x[1] - x[0], 2) + Math.pow(y[1] - y[0], 2));
		// calculate cosA and corresponding angle
		cosA = (Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2)) / (2 * b * c);
		double angle = Math.toDegrees(Math.acos(cosA));
		System.out.printf("\na: %.2f b: %.2f c: %.2f\n", a, b, c);
		System.out.printf("cosA: %.2f angle: %.2f\n", cosA, angle);
		
		for(int i=0; i<2; i++){
			motors[i].close();
		}
	}
}