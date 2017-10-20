/*
 * Nicholas Mayne & Laura Petrich, University of Alberta © 2017.
 */
package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import L2.C_PIDController;

public class C_Main {

	public static void main(String[] args) {
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.A);
		C_PIDController PIDController = new C_PIDController(M1);
		Stopwatch timer = new Stopwatch();
		
//		// (Un)comment this section to run iterative testing w/ Matlab output
//		
//		String filename = "";
//		String expData = "";
//		String legend = "";
//		
//		int timeout;
//		int prec = 10;			// nicer printout of PID K values in Matlab
//		int testTime = 1750;		// how long is one parameter test?
//		
//		// start values		// end values		// increment
//		
//		// P Controller
////		double p_ = 1.0;		double p__ = 16.0;	double p___ = 1.0;		// Kp > 14 is unstable
////		double i_ = 0.0;		double i__ = 0.0;	double i___ = 0.1;		// Ki > 0.6 is unstable
////		double d_ = 0.0;		double d__ = 0.0;	double d___ = 0.5;		// Kd > 4 is unstable
//		
//		// PD Controller
////		double p_ = 1.0;		double p__ = 14.0;	double p___ = 1.0;		// Kp > 14 is unstable
////		double i_ = 0.0;		double i__ = 0.0;	double i___ = 0.1;		// Ki > 0.6 is unstable
////		double d_ = 0.0;		double d__ = 4.0;	double d___ = 0.5;		// Kd > 4 is unstable
//		
//		// PID Controller
////		double p_ = 1.0;		double p__ = 14.0;	double p___ = 1.0;		// Kp > 14 is unstable
////		double i_ = 0.0;		double i__ = 0.6;	double i___ = 0.1;		// Ki > 0.6 is unstable
////		double d_ = 0.0;		double d__ = 4.0;	double d___ = 0.5;		// Kd > 4 is unstable
//		
//		
//		
//		int setPoints[] = {50, 200, 400};	// test these angles (Deg) {50, 200, 400}
//		int powers[] = {25, 50, 100};		// test these motor powers {25, 50, 100}
//		
//		// approximately how long is this test series going to take?
//		double PIDLoopCount = (((p__-p_)/p___)+1) * (((i__-i_)/i___)+1) * (((d__-d_)/d___)+1);
//		int EstTime = (int) ((PIDLoopCount * setPoints.length * powers.length * testTime));
//		System.out.printf("ETA: %.2f min\n\n", (double)((EstTime / 1000) / 60));
//		
//		// run and log tests
//		for (int power : powers) {
//			for(int setPoint : setPoints) {
//				timeout = 15000 / power; // wait longer for lower powers
//				
//				filename = "P" + power + "SP" + setPoint;
//				
//			    FileWriter out = null; 
//			    File data = new File(filename + ".m");
//			    
//			    try	{
//			    		out = new FileWriter(data);
//			    } 
//			    catch(IOException e) {
//			    		System.err.println("Failed to create output stream");
//			    		Button.waitForAnyPress();
//			    		System.exit(1);
//			    	}
//				System.out.println("Opened: " + filename);
//
//			    try {
//			    		out.write(""
//							+ "figure(1);\n"
//							+ "hold on;\n"
//							+ "xlabel('Time (ms)');\n"
//							+ "ylabel('Theta (Deg)');\n"
//							+ "title('PID Controller | " + filename + "');\n"
//							+ "plot([0 " + timeout + "], [" + setPoint + " " + setPoint + "], 'r--');\n");
//			    }
//			    catch (IOException e) {
//			    		System.err.println("Failed to write file header");
//			    }
//
//				// run the PID test
//				for(double Kp = p_; Kp <= p__; Kp = Kp + p___) {
//					for(double Ki = i_; Ki <= i__; Ki = Ki + i___) {
//						for(double Kd = d_; Kd <= d__; Kd = Kd + d___) {
//							String label = String.format("Kp%di%dd%d", (int) (Kp*prec), (int) (Ki*prec), (int) (Kd*prec));
//							legend = legend + "'" + label + "', ";
//							System.out.println(label);
//							
//							expData = PIDController.PID(setPoint, Kp, Ki, Kd, power, timeout);
//							
//							System.out.print("Time: " + (expData.substring(expData.lastIndexOf(';', expData.length()-3) + 2, expData.lastIndexOf(',')-2)) + " ms\n");							
//						    EstTime = EstTime - testTime;
//						    System.out.printf("ETA: %.2f min\n\n", (double)((EstTime / 1000) / 60));
//							try {
//					    			out.write(""
//					    					+ label + " = ["
//					    					+ expData
//					    					+ "];\n"
//										+ "plot(" + label + "(:,1)," + label + "(:,2))\n"
//										+ "% y = max(" + label + "(:,2));\n"
//										+ "% x = " + label + "(find(" + label + "(:,2) == max(y), 1, 'first'));\n"
//										+ "% text(x, y, '" + label + "');\n"
//										+ "% pause;\n\n");
//						    }
//						    catch (IOException e) {
//						    		System.err.println("Failed to write exp data");
//						    }
//							Delay.msDelay(100);
//						}
//					}
//				}
//				try {
//					out.write(""
//							+ "caseList = {" + legend.substring(0, legend.length() - 2) + "};\n"
//							+ "legend('" + Integer.toString(setPoint) + " Deg', caseList{1,:});\n\n"
//							+ "t = {" + legend.substring(0, legend.length() - 2).replace("'", "") + "};\n\n");
//					out.close(); // flush the buffer and write the file
//				}
//				catch (IOException e) {
//					System.err.println("Failed to close file");
//				}
//				System.out.println("Closed: " + filename);
//				legend = "";
//			}
//		}
		
		
/**
 * PART C1 - TIME = 1.43 min, ERROR = 0
 * Selected fitted model Mean Kp(mp) from Matlab/C1/ because of monotonicty, and near-linearity that matches context expectations
 * Performs reasonable well at low powers, struggles as battery gets lower, compensation for undershooting leads to overshoot.
 * 
 * Linear model Poly2:
 * 		f(x) = p1*x^2 + p2*x + p3
 * Coefficients:
 *		p1 =   0.0002578
 * 		p2 =    -0.08867
 * 		p3 =       8.622    // Corrected by an additional +5.0
 * 							// Tried correcting for debilitating steady state error from overshoot and undershoot, but could not.
 * 							// This is really dependent upon battery power, as the battery power drops the gains needs compensation!
 */
//		// (Un)comment this section to run model for P Controller
//		int timeout = 20000;
//		int totalError = 0;
////		int setPoints[] = {50, 150, -400, 33, 854, 237, -90, 1000};
////		int powers[] = {25, 50, 100};
//		
//		int setPoints[] = {1, 5, 10, 25, 50, 100, 300, 700};
//		int powers[] = {15, 25, 50, 75, 100};
//		
//		double[] p1 = {0.0002578, -0.08867, 8.622};
//		
//		timer.reset();						
//		for (int power : powers) {
//			for(int setPoint : setPoints) {
//				double Kp = (p1[0] * Math.pow(power, 2)) + (p1[1] * power) + p1[2] + 5.0;
//				double Ki = 0;
//				double Kd = 0;
//				System.out.printf("P: %.6f\nI: %.6f\nD: %.6f\n",Kp, Ki, Kd);
//				totalError = totalError + Integer.parseInt(PIDController.PID(setPoint, Kp, Ki, Kd, power, timeout));	
//				Delay.msDelay(1000);
//			}
//		}
//		System.out.printf("TIME: %.2f min\nTOTAL P ERROR:\n%d\n\n\n", ((double) timer.elapsed())/60000, totalError);
//		Button.waitForAnyPress();
		
/**
 * PART C2 TIME = 1.46 min, ERROR = 0
 * Selected fitted model Mean Kp(mp), Mean Kd(mp) from Matlab/C2/ because of monotonicty, and pleasing inverse relationship matching context expectations
 * Had to resample data because power was too low and derivative too high for low powers. Really struggles at low powers when battery power is low
 * Linear model Poly2 for Kp:
 * 		f(x) = p1*x^2 + p2*x + p3
 * Coefficients:
 *		p1 =   0.0004637
 * 		p2 =    -0.08011
 * 		p3 =       8.622  // Corrected by an additional +5.0
 *        
 *        
 * Linear model Poly2 for Kd:
 * 		f(x) = p1*x^2 + p2*x + p3
 * Coefficients:
 *		p1 =  -0.0001348
 * 		p2 =     0.02578
 * 		p3 =      0.6731  // Corrected by an additional +0.1
 */
//		// (Un)comment this section to run model for PD gain
//		int timeout = 20000;
//		int totalError = 0;		
////		int setPoints[] = {50, 150, -400, 33, 854, 237, -90, 1000};
////		int powers[] = {25, 50, 100};
//		
//		int setPoints[] = {1, 5, 10, 25, 50, 100, 300, 700};
//		int powers[] = {10, 25, 50, 75, 100};
//		
//		double[] p2 = {0.0004637, -0.08011, 5.18};
//		double[] d2 = {-0.0001348, 0.02578, 0.6731};
//		
//		timer.reset();
//		for (int power : powers) {
//			for(int setPoint : setPoints) {
//				double Kp = (p2[0] * Math.pow(power, 2)) + (p2[1] * power) + p2[2] + 5.0;
//				double Ki = 0;
//				double Kd = (d2[0] * Math.pow(power, 2)) + (d2[1] * power) + d2[2] + 0.1;
//				System.out.printf("P: %.6f\nI: %.6f\nD: %.6f\n",Kp, Ki, Kd);
//				totalError = totalError + Integer.parseInt(PIDController.PID(setPoint, Kp, Ki, Kd, power, timeout));	
//				Delay.msDelay(1000);
//			}
//		}
//		System.out.printf("TIME: %.2f min\nTOTAL PD ERROR:\n%d\n\n\n", ((double) timer.elapsed())/60000, totalError);
//		Button.waitForAnyPress();
		
/**
 * PART C3 TIME = 1.45 min, ERROR = 0
 * Selected fitted model Mean Kp(mp), Mode Ki(sp) and Mode Kd(mp) from Matlab/C3/ 
 * Difficult to tune! Tried Kx(sp) and this worked okay inside the range of the sp test data but failed outside it.
 * Re-fitted to new models, works well for low powers and small set point and for larger powers and larger set points, struggles a bit in between.
 * Added maxamin chops to Ki because it was misbehaving near 0.
 * 
 * General model Power2 for Kp(mp):
 *      f(x) = a*x^b+c
 * Coefficients:
 *      a =        32.54
 *      b =       -0.782
 *      c =       0.9915 // Corrected by an additional +7.0
 *      
 * General model Power2 for Ki(sp):
 *      f(x) = a*x^b+c
 * Coefficients:
 *      a =     1.179e+14
 *      b =         -8.74
 *      c =    -4.566e-07 // Applied maxamin to bound integral term
 *    
 * General model Power1 for Kd(mp):
 *      f(x) = a*x^b
 * Coefficients:
 *      a =        0.3761
 *      b =        0.3666
 */
//		// (Un)comment this section to run model for Kp gain
//		int timeout = 20000;
//		int totalError = 0;
////		int setPoints[] = {50, 150, -400, 33, 854, 237, -90, 1000};
////		int powers[] = {25, 50, 100};
//		
//		int setPoints[] = {1, 5, 10, 25, 50, 100, 300, 700};
//		int powers[] = {15, 25, 50, 75, 100};
//		
//		double[] p3 = {32.54, -0.782, 0.9915};
//		double[] i3 = {1.179e+14, -8.74, -4.566e-07};
//		double[] d3 = {0.3761, 0.3666};
//		
//		timer.reset();						
//		for (int power : powers) {
//			for(int setPoint : setPoints) {
//				double Kp = (p3[0] * Math.pow(power, (p3[1]))) + p3[2] + 7.0;
//				double Ki = Math.max(0, Math.min(1, (i3[0] * Math.pow(Math.abs(setPoint), (i3[1]))) + i3[2]));
//				double Kd = d3[0] * Math.pow(power, (d3[1]));
//				System.out.printf("P: %.6f\nI: %.6f\nD: %.6f\n",Kp, Ki, Kd);
//				totalError = totalError + Integer.parseInt(PIDController.PID(setPoint, Kp, Ki, Kd, power, timeout));	
//				Delay.msDelay(1000);
//			}
//		}
//		System.out.printf("TIME: %.2f min\nTOTAL PID ERROR:\n%d\n\n\n", ((double) timer.elapsed())/60000, totalError);
//		Button.waitForAnyPress();
		
		
	M1.close();	
	}
}
