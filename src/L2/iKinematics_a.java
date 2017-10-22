package L2;

import L2.iKinematics_b;

public class iKinematics_a {

	public static void main(String[] args) {
		// method stub
		try {
			iKinematics_b.ikine(102,102); // (102, 102) ~ (-45 deg, -45 deg)
		} catch (Exception e) {
			// catch block
			e.printStackTrace();
		}
	}
}
