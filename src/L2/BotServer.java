package L2;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;

public class BotServer {
	public BotServer(int port) {
		// setup TCP
        try (
            ServerSocket serverSocket =new ServerSocket(port);
            Socket clientSocket = serverSocket.accept();
            BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
        	) {
            	String receivedData;
            	System.out.println("Connected");
            	double x = 0;
            	double y = 0;
            	while ((receivedData = in.readLine()) != null) {
            		if (receivedData.startsWith("X:")) {
            			x = Double.valueOf(receivedData.substring(2));
            		} else if (receivedData.startsWith("Y:")) {
            			y = Double.valueOf(receivedData.substring(2));
            		}       		
            		System.out.printf("X: %.2f\nY: %.2f\n", x, y);
            	}
        	} catch (IOException e) {
        		System.out.println("No connection on port " + port);
        		System.out.println(e.getMessage());
        }
	}
}