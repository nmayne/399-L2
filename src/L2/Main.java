package L2;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import L2.PIDController;


public class Main {

	public static void main(String[] args) {
		UnregulatedMotor M1 = new UnregulatedMotor(MotorPort.A);
		PIDController PIDController1 = new PIDController(M1);
		
		String filename = "";
		String expData = "";
		String legend = "";
		
		int timeout;
		int prec = 10;		// what x-ths are the pid gains, used for nicer printout in Matlab
		
		// start values		// end values		// increment
		double p_ = 4.0;		double p__ = 15.0;	double p___ = 1.0;
		double i_ = 0.0;		double i__ = 0.005;	double i___ = 0.001;
		double d_ = 0.0;		double d__ = 3.0;	double d___ = 0.5;
		
		int setPoints[] = {50, 150, 400};
		int powers[] = {10, 50, 100};
		
//		int setPoints[] = {50, 150, -400, 33, 854, 237, -90, 1000};
//		int powers[] = {10, 25, 75, 100};
//		
//		// Kp poly					//Ki gauss					// Kd poly
//		double p_a = 3.259;			double i_a = 4.347e+15;		double d_a = -3.806;
//		double p_b = 0.1189;			double i_b = -126.8;			double d_b = 0.2893;
//		double p_c = -0.002427;		double i_c = 21.3;			double d_c = -0.004203;
//		double p_d = 1.354e-05;									double d_d = 1.888e-05;
//								
//		
//		for (int power : powers) {
//			for(int setPoint : setPoints) {
//				timeout = 200000;
//				double Kp = p_a + (p_b * power) + (p_c * Math.pow(power, 2)) + (p_d * Math.pow(power, 3));
//				double Ki = i_a * Math.exp(-Math.pow(((power - i_b) / i_c), 2));
//				double Kd = d_a + (d_b * power) + (d_c * Math.pow(power, 2)) + (d_d * Math.pow(power, 3));
//				
////				System.out.printf("P: %.5f\nI: %.5f\nD: %.5f\n",Kp, Ki, Kd);
//				PIDController1.PID(setPoint, Kp, Ki, Kd, power, timeout);	
////				Delay.msDelay(2000);
//			}
//		}
		
		

		
		for (int power : powers) {
			for(int setPoint : setPoints) {
				timeout = 200000/power;
				
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
			    				+ "clear all; clc;\n"
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
							
							expData = PIDController1.PID(setPoint, Kp, Ki, Kd, power, timeout);
							
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
							+ "t = {" + legend.substring(0, legend.length() - 2).replace("'", "") + "};\n\n"
							+ "figure(2);\n"
							+ "hold on;\n"
							+ "xlabel('Time (ms)');\n"
							+ "ylabel('Theta (Deg)');\n"
							+ "title('PID Controller | " + filename + " | Good Options');\n"
							+ "plot([0 " + timeout + "], [" + setPoint + " " + setPoint + "], 'r--');\n"
							+ "legendTwo = {'" + setPoint + " Deg'};\n"
							+ "idx = 1;\n"
							+ "for e = t\n"
							+ "    if max(e{1}(:,2)) <= "+ (setPoint + 5)
							+ " && max(e{1}(:,2)) >= " + (setPoint) + "\n"
							+ "        plot(e{1}(:,1),e{1}(:,2));\n"
							+ "        legendTwo{end+1} = caseList{idx};\n"
							+ "    end\n"
							+ "    idx = idx + 1;\n"
							+ "end\n"
							+ "legend(legendTwo)\n");
					out.close(); // flush the buffer and write the file
				}
				catch (IOException e) {
					System.err.println("Failed to close file");
				}
				System.out.println("Closed: " + filename);
				legend = "";
			}
		}
		
		
		
	}
}
