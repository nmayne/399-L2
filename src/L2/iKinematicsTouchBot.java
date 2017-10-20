package L2;
import java.util.ArrayList;

import L2.invKine;

public class iKinematicsTouchBot {
	public static void main(String[] args) {
		BotServer TouchBotInput = new BotServer(3000);
		ArrayList<int[]> inputCoords = new ArrayList<int[]>();
		

			inputCoords = TouchBotInput.getTouchCoords();
			
			System.out.println(inputCoords.size());
			System.out.println(inputCoords.get(1)[0]);
			System.out.println(inputCoords.get(2)[1]);


			// method stub
//			for (int i = 0; i < inputCoords.size(); i++) {
//				try {
//					System.out.printf("X: %d \n Y: %d \n\n", inputCoords.get(i)[0], inputCoords.get(i)[1]);
////					TouchBotInvKine.ikine(inputCoords.get(i)[0], (inputCoords.get(i)[1]);
//				} catch (Exception e) {
//					// catch block
//					e.printStackTrace();
//				}
			
			
//		}
	}
}