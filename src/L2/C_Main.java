/*
 * Nicholas Mayne & Laura Petrich, University of Alberta © 2017.
 */
package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import L2.C_PIDController;

public class C_Main {

	public static void main(String[] args) {
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.A);
		C_PIDController PIDController = new C_PIDController(M1);
		
		// (Un)comment this section to run iterative testing w/ Matlab output
		
		String filename = "";
		String expData = "";
		String legend = "";
		
		int timeout;
		int prec = 10;			// nicer printout of PID K values in Matlab
		int testTime = 1750;		// how long is one parameter test?
		
		// start values		// end values		// increment
		
		// P Controller
//		double p_ = 1.0;		double p__ = 16.0;	double p___ = 1.0;		// Kp > 14 is unstable
//		double i_ = 0.0;		double i__ = 0.0;	double i___ = 0.1;		// Ki > 0.6 is unstable
//		double d_ = 0.0;		double d__ = 0.0;	double d___ = 0.5;		// Kd > 4 is unstable
		
		// PD Controller
//		double p_ = 1.0;		double p__ = 14.0;	double p___ = 1.0;		// Kp > 14 is unstable
//		double i_ = 0.0;		double i__ = 0.0;	double i___ = 0.1;		// Ki > 0.6 is unstable
//		double d_ = 0.0;		double d__ = 4.0;	double d___ = 0.5;		// Kd > 4 is unstable
		
		// PID Controller
		double p_ = 1.0;		double p__ = 14.0;	double p___ = 1.0;		// Kp > 14 is unstable
		double i_ = 0.0;		double i__ = 0.6;	double i___ = 0.1;		// Ki > 0.6 is unstable
		double d_ = 0.0;		double d__ = 4.0;	double d___ = 0.5;		// Kd > 4 is unstable
		
		
		
		int setPoints[] = {50, 200, 400};	// test these angles (Deg) {50, 200, 400}
		int powers[] = {100};		// test these motor powers {25, 50, 100}
		
		// approximately how long is this test series going to take?
		double PIDLoopCount = (((p__-p_)/p___)+1) * (((i__-i_)/i___)+1) * (((d__-d_)/d___)+1);
		int EstTime = (int) ((PIDLoopCount * setPoints.length * powers.length * testTime));
		System.out.printf("ETA: %.2f min\n\n", (double)((EstTime / 1000) / 60));
		
		// run and log tests
		for (int power : powers) {
			for(int setPoint : setPoints) {
				timeout = 150000 / power; // wait longer for lower powers
				
				filename = "P" + power + "SP" + setPoint;
				
			    FileWriter out = null; 
			    File data = new File(filename + ".m");
			    
			    try	{
			    		out = new FileWriter(data);
			    } 
			    catch(IOException e) {
			    		System.err.println("Failed to create output stream");
			    		Button.waitForAnyPress();
			    		System.exit(1);
			    	}
				System.out.println("Opened: " + filename);

			    try {
			    		out.write(""
							+ "figure(1);\n"
							+ "hold on;\n"
							+ "xlabel('Time (ms)');\n"
							+ "ylabel('Theta (Deg)');\n"
							+ "title('PID Controller | " + filename + "');\n"
							+ "plot([0 " + timeout + "], [" + setPoint + " " + setPoint + "], 'r--');\n");
			    }
			    catch (IOException e) {
			    		System.err.println("Failed to write file header");
			    }

				// run the PID test
				for(double Kp = p_; Kp <= p__; Kp = Kp + p___) {
					for(double Ki = i_; Ki <= i__; Ki = Ki + i___) {
						for(double Kd = d_; Kd <= d__; Kd = Kd + d___) {
							String label = String.format("Kp%di%dd%d", (int) (Kp*prec), (int) (Ki*prec), (int) (Kd*prec));
							legend = legend + "'" + label + "', ";
							System.out.println(label);
							
							expData = PIDController.PID(setPoint, Kp, Ki, Kd, power, timeout);
							
							System.out.print("Time: " + (expData.substring(expData.lastIndexOf(';', expData.length()-3) + 2, expData.lastIndexOf(',')-2)) + " ms\n");							
						    EstTime = EstTime - testTime;
						    System.out.printf("ETA: %.2f min\n\n", (double)((EstTime / 1000) / 60));
							try {
					    			out.write(""
					    					+ label + " = ["
					    					+ expData
					    					+ "];\n"
										+ "plot(" + label + "(:,1)," + label + "(:,2))\n"
										+ "% y = max(" + label + "(:,2));\n"
										+ "% x = " + label + "(find(" + label + "(:,2) == max(y), 1, 'first'));\n"
										+ "% text(x, y, '" + label + "');\n"
										+ "% pause;\n\n");
						    }
						    catch (IOException e) {
						    		System.err.println("Failed to write exp data");
						    }
							Delay.msDelay(100);
						}
					}
				}
				try {
					out.write(""
							+ "caseList = {" + legend.substring(0, legend.length() - 2) + "};\n"
							+ "legend('" + Integer.toString(setPoint) + " Deg', caseList{1,:});\n\n"
							+ "t = {" + legend.substring(0, legend.length() - 2).replace("'", "") + "};\n\n");
					out.close(); // flush the buffer and write the file
				}
				catch (IOException e) {
					System.err.println("Failed to close file");
				}
				System.out.println("Closed: " + filename);
				legend = "";
			}
		}
		
		
//		// (Un)comment this section to run interpolated functions for K gains
//		int timeout = 200000;
//		int setPoints[] = {50, 150, -400, 33, 854, 237, -90, 1000};
//		int powers[] = {10, 25, 75, 100};
//		
//		// Kp poly					//Ki gauss					// Kd poly
//		double p_a = 3.259;			double i_a = 4.347e+15;		double d_a = -3.806;
//		double p_b = 0.1189;			double i_b = -126.8;			double d_b = 0.2893;
//		double p_c = -0.002427;		double i_c = 21.3;			double d_c = -0.004203;
//		double p_d = 1.354e-05;									double d_d = 1.888e-05;
//								
//		for (int power : powers) {
//			for(int setPoint : setPoints) {
//				double Kp = p_a + (p_b * power) + (p_c * Math.pow(power, 2)) + (p_d * Math.pow(power, 3));
//				double Ki = i_a * Math.exp(-Math.pow(((power - i_b) / i_c), 2));
//				double Kd = d_a + (d_b * power) + (d_c * Math.pow(power, 2)) + (d_d * Math.pow(power, 3));
//				
//				PIDController.PID(setPoint, Kp, Ki, Kd, power, timeout);	
//				
//				System.out.printf("P: %.5f\nI: %.5f\nD: %.5f\n",Kp, Ki, Kd);
//				Delay.msDelay(2000);
//			}
//		}
		
		
		
/*
 *		General model Power1 for Kp:
		f(x) = a*x^b
		Coefficients (with 95% confidence bounds):
		a =       18.51  (-8.729, 45.74)
		b =     -0.5707  (-0.9995, -0.1418)
		
		General model Power1 for Kd:
     	f(x) = a*x^b
		Coefficients (with 95% confidence bounds):
       	a =      0.5646  (0.155, 0.9741)
       	b =      0.2721  (0.08683, 0.4573)

 */
		
//		// (Un)comment this section to run interpolated functions for K gains
//		int timeout = 200000;
//		int setPoints[] = {50, 150, -400, 33, 854, 237, -90, 1000};
//		int powers[] = {10, 25, 75, 100};
//		
//		// Kp poly					//Ki gauss				// Kd poly
//		double p_a = 18.51;			double i_a = 0;			double d_a = 0.5646;
//		double p_b = -0.5707;		double i_b = 0;			double d_b = 0.2721;
//								
//		for (int power : powers) {
//			for(int setPoint : setPoints) {
//				double Kp = p_a * Math.pow(power, p_b);
//				double Ki = i_a * Math.pow(power, i_b);
//				double Kd = d_a * Math.pow(power, d_b);
//				
//				PIDController.PID(setPoint, Kp, Ki, Kd, power, timeout);	
//				
//				System.out.printf("P: %.5f\nI: %.5f\nD: %.5f\n",Kp, Ki, Kd);
//				Delay.msDelay(2000);
//			}
//		}
		
		
		
	M1.close();	
	}
}
