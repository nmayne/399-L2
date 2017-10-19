package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;

public class midPoint {
	
	static Stopwatch timer = new Stopwatch();				// the timer
	static UnregulatedMotor[] motors = {
		new UnregulatedMotor(MotorPort.D),	// left motor motors[0]
		new UnregulatedMotor(MotorPort.A)	// right motor motors[1]
	};
	static int[][] TC = new int[4][2];
	static int[] offset = {41,72};
	static int power = 0;
	static int count = 0;
	static int command;
	static int tacho;
	static int x;
	static int y;
	static int sampleRate = 10;			// enforced minimum sample rate, 
	static int timeout = 100000;		// timeout after x ms
	static double[] M1 = {0, 0};
	static double[] M2 = {0, 0};
	static double error ;				// error
	static double prevError;			// error from previous sample
	static double derivative;			// the derivative error
	static double delta_t;				// delta t
	static double elapsed_t;			// elapsed t
	static double theta;				// change in theta
	static double rad;
	static double distance;
	static double a;
	static double b;
	static double c;
	static double[] M = new double[2];
	static double cosA;
	static double angle;
	static double[][] rotMat = new double[2][2];
	static double[][] point = new double[3][2]; // [0][0] = Ax, [0][1] = Ay, [1][0] = Bx, [1][1] = By, [2][0] = Cx, [2][1] = Cy
	static double[][] coord = new double[2][2];


	public static void main(String[] args) throws Exception {
		System.out.println("Press the enter \nbutton to begin.\n");   
		// zero out tachocount just in case
		for(int i=0; i<2; i++){
			motors[i].resetTachoCount();
		}
		// only grab 3 points (start, point 1, point 2)
		// find midpoint between point 1 and point 2
		while (count < 3) {
			command = Button.readButtons();
			switch(command) {
				case 16:
					// left button pressed
            			motors[1].setPower(100);
            			Delay.msDelay(100);
            			motors[1].setPower(0);
            			break;
				case 1:
					// up button pressed
					motors[1].setPower(-100);
					Delay.msDelay(100);
        				motors[1].setPower(0);			
					break;
				case 4:
					// right button pressed
        				motors[0].setPower(100);
        				Delay.msDelay(100);	
        				motors[0].setPower(0);
        				break;
				case 8:
					// down button pressed
					motors[0].setPower(-100);
					Delay.msDelay(100);
        				motors[0].setPower(0);			
					break;			
				case 2:
					// enter button pressed
					Delay.msDelay(1000);	// keeps down error 
					if (count == 0) {
						System.out.println("Please select two points.\n");
						Delay.msDelay(1000);	// keeps down error 
						for(int i=0; i<2; i++){
							motors[i].resetTachoCount();
						}
						point[0][0] = -74; 	// Ax --intersection point
						point[0][1] = 0;	// Ay --intersection point
					}
					else {
						System.out.printf("\nPoint %d marked.\n", count);
						for(int i=0; i<2; i++){
							int boop = motors[i].getTachoCount();
//							System.out.printf("\nTC: %d  i: %d", boop, i);
							TC[count][i] = boop;
							motors[i].resetTachoCount();
						}
					}
					count++;
					Delay.msDelay(1000);	// keeps down error 
					break;
				default:
					break;		
			} 
		} 
		// invKine opens new motors so need to close these
		for(int i=0; i<2; i++){
		motors[i].close();
		}
		// maths
		for (int k = 1; k < 3; k++) {
			for (int j = 0; j < 2; j++) { // j = motor 0 or 1
				rad = ((Math.PI*TC[k-1][j])/(180*offset[j]))%(2*Math.PI);
				rotMat[0][0] = Math.cos(rad);
				rotMat[1][1] = rotMat[0][0];
				if (rad > 0) { 	// clockwise 
					rotMat[0][1] = Math.sin(rad);
					rotMat[1][0] = Math.sin(rad) * (-1);
				}
				else { 			// counterclockwise
					rotMat[0][1] = Math.sin(Math.abs(rad)) * (-1);
					rotMat[1][0] = Math.sin(Math.abs(rad));		
				}
	
				coord[j][0] = (rotMat[0][0]*point[k-1][0]) + (rotMat[0][1]*point[k-1][1]); // x coordinate for joint j
				coord[j][1] = (rotMat[1][0]*point[k-1][0]) + (rotMat[1][1]*point[k-1][1]); // y coordinate for joint j
			}
			point[k][0] = coord[0][0] + coord[1][0] - point[k-1][0]; // new x coordinate at end effector
			point[k][1] = coord[0][1] + coord[1][1] - point[k-1][1]; // new y coordinate at end effector
			System.out.printf("point: %d \nCoord x: %.2f\n", k, point[k][0]);
			System.out.printf("Coord y: %.2f\n", point[k][1]);
		}
		// find midpoint between 2nd and 3rd point
		M[0] = (point[1][0] + point[2][0])/2;  	// x coordinate at midpoint
		M[1] = (point[1][1] + point[2][1])/2;	// y coordinate at midpoint
		System.out.printf("\nMidpoint\nx: %.2f y: %.2f\n", M[0],M[1]);
		// move the end effector to midpoint
		
		// Move to x = M[0] - point[2][0] - 74; y = M[1] - point[2][1] - 0;
		x = (int) (M[0] - point[2][0] - 74);
		y = (int) (M[1] - point[2][1]);
		System.out.printf("\ninvK\nx: %d y: %d\n", x,y);
		// TO DO ////////////////////////////////////////////////////////////////
		// Call invKine from where arm is currently and move to (x, y)

		
		
		
		
		
		////////////
		Delay.msDelay(1000);
	}
}