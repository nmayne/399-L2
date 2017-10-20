package L2;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;

public class BotServer {
	int port;
	int[] coord = new int[2];
	ArrayList<int[]> coordList;

	
	public BotServer(int port) {
		this.port = port;
		this.coordList = new ArrayList<int[]>();
	}
	
	public ArrayList<int[]> getTouchCoords() {
		// setup TCP
        try (
            ServerSocket serverSocket = new ServerSocket(port);
            Socket clientSocket = serverSocket.accept();
            BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
        	) {
            	String receivedData;
            	System.out.println("Connected");

            	while ((receivedData = in.readLine()) != null) {
            		if (receivedData.startsWith("X:")) {
            			coord[0] = Integer.parseInt(receivedData.substring(2));
            		} else if (receivedData.startsWith("Y:")) {
            			coord[1] = Integer.parseInt((receivedData.substring(2)));
            		} else if (receivedData.startsWith("F")) {
            			break;
            		}
            		coordList.add(coord);
            	}
        	} catch (IOException e) {
        		System.out.println("No connection on port " + port);
        		System.out.println(e.getMessage());
        }
		return coordList;
	}
}