package L2;

import L2.forwardK;

public class fKinematics {

	public static void main(String[] args) {
		// method stub
		try {
			forwardK.fkine(45, 45); // x = [0, 60] y = [-60, 170]
		} catch (Exception e) {
			// catch block
			e.printStackTrace();
		}
	}
}
