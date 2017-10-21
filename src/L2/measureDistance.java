package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

public class measureDistance {
	
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
	static double EEx;
	static double EEy;
	static double[][] rotMat = new double[2][2];
	static double[][] points = new double[2][2]; // store x, y coordinate for 2 points
	static double[][] coord = new double[2][2];

	public static void main(String[] args) {
		System.out.println("Press a button\n");   
		
		for(int i=0; i<2; i++){
			motors[i].setPower(100);
			motors[i].stop();
			motors[i].resetTachoCount();
		}

		while (count < 2) {
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
						System.out.println("First point marked.\n");
						Delay.msDelay(1000);	// keeps down error 
						for(int i=0; i<2; i++){
							motors[i].resetTachoCount();
							TC[count][i] = motors[i].getTachoCount();
						}
						points[0][0] = -72;
						points[0][1] = 0;
					}
					else {
						System.out.printf("Point %d marked.", count + 1);
						for(int i=0; i<2; i++){
							TC[count][i] = motors[i].getTachoCount();
							System.out.println("\nTC " + i + ": " + TC[count][i]);
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
		for (int j = 0; j < 2; j++) { // j = motor 0 or 1
			rad = ((Math.PI*TC[1][j])/(180*offset[j]))%(2*Math.PI);
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

			coord[j][0] = (rotMat[0][0]*points[0][0]) + (rotMat[0][1]*points[0][1]);
			coord[j][1] = (rotMat[1][0]*points[0][0]) + (rotMat[1][1]*points[0][1]);
			System.out.printf("coord %d x: %.0f y: %.0f\n", j, coord[j][0], coord[j][1]);
		}
		double J1x = (2.067 * coord[0][0]) + 0.3235;
		double J1y = (1.717 * coord[0][1]) + 0.9889;
    	if (TC[1][1] == 0) {
    		EEx = J1x;
    		EEy = J1y;
    	}
    	else if (TC[1][0] == 0){
    		EEx = -72 + coord[1][0];
    		EEy = coord[1][1];
    	}
    	else {
    		EEx = J1x - coord[1][0] - 72;
    		EEy = J1y; // 10 keeps down error
    	}
    	distance = Math.sqrt(((EEx - points[0][0]) * (EEx - points[0][0])) + ((EEy - points[0][1]) * (EEy - points[0][1])));
    	
    	System.out.printf("J1x: %.2f J1y: %.2f", J1x, J1y);
    	System.out.printf("\nEEx : %.0f EEy : %.0f" , EEx, EEy);
		System.out.printf("\nDistance: %.2f", distance);
		
		for(int i=0; i<2; i++){
			motors[i].close();
		}
	}
}