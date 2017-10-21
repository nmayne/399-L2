package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

public class measureAngles {
	
	static UnregulatedMotor[] motors = {
		new UnregulatedMotor(MotorPort.D),	// left motor motors[0]
		new UnregulatedMotor(MotorPort.A)	// right motor motors[1]
	};
	static int[][] TC = new int[3][2];
	static int[] offset = {61,69};
	static int count = 0;
	static int command;
	static double rad;
	static double distance;
	static double a;
	static double b;
	static double c;
	static double cosA;
	static double angle;
	static double[][] rotMat = new double[2][2];
	static double[][] point = new double[3][2]; // [0][0] = Ax, [0][1] = Ay, [1][0] = Bx, [1][1] = By, [2][0] = Cx, [2][1] = Cy
	static double[][] coord = new double[2][2];

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
					Delay.msDelay(1000);	// keeps down error 
					if (count == 0) {
						System.out.println("Intersection marked.\n");
						Delay.msDelay(1000);	// keeps down error 
						for(int i=0; i<2; i++){
							motors[i].resetTachoCount();
						}
						point[0][0] = -72; 	// Ax --intersection point
						point[0][1] = 0;	// Ay --intersection point
					}
					else {
						System.out.printf("Point %d marked.\n", count + 1);
						for(int i=0; i<2; i++){
							TC[count-1][i] = motors[i].getTachoCount();
							motors[i].resetTachoCount();
//							System.out.println("\nTC " + i + ": " + TC[count][i]);
						}
					}
					count++;
					Delay.msDelay(1000);	// keeps down error 
					break;
				case 32:
					// escape button pressed
					break;
				default:
					break;		
			} 
		} 
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
			double J1x = (2.067 * coord[0][0]) + 0.3235;
			double J1y = (1.717 * coord[0][1]) + 0.9889;
		    	System.out.printf("J1x: %.2f J1y: %.2f", J1x, J1y);
		    	if (TC[k-1][1] == 0) {
		    		point[k][0] = J1x;
		    		point[k][1] = J1y;
		    	}
		    	else if (TC[k-1][0] == 0){
		    		point[k][0] = -72 + coord[1][0];
		    		point[k][1] = coord[1][1];
		    	}
		    	else {
		    		point[k][0] = J1x - coord[1][0] - 72;
		    		point[k][1] = J1y; // 10 keeps down error
		    	}
//			point[k][0] = coord[0][0] + coord[1][0] - point[k-1][0]; // new x coordinate at end effector
//			point[k][1] = coord[0][1] + coord[1][1] - point[k-1][1]; // new y coordinate at end effector
			System.out.printf("point: %d \nCoord x: %.2f\n", k, point[k][0]);
			System.out.printf("Coord y: %.2f\n", point[k][1]);
		}


//	    	System.out.printf("\nEEx : %.0f EEy : %.0f" , EEx, EEy);
		// use law of cosines --> cos(A) = (b^2 + c^2 - a^2) / 2bc to calculate angle at intersection 
		// calculate a, b, c
		a = Math.sqrt(Math.pow(point[2][0] - point[1][0], 2) + Math.pow(point[2][1] - point[1][1], 2));
		b = Math.sqrt(Math.pow(point[2][0] - point[0][0], 2) + Math.pow(point[2][1] - point[0][1], 2));
		c = Math.sqrt(Math.pow(point[1][0] - point[0][0], 2) + Math.pow(point[1][1] - point[0][1], 2));
		// calculate cosA and corresponding angle
		cosA = (Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2)) / (2 * b * c);
		angle = Math.toDegrees(Math.acos(cosA));
		System.out.printf("a: %.2f b: %.2f c: %.2f\n", a, b, c);
		System.out.printf("cosA: %.2f angle: %.2f\n", cosA, angle);
		
		for(int i=0; i<2; i++){
			motors[i].close();
		}
	}
}