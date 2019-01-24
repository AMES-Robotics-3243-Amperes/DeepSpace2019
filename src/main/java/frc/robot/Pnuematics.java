package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Pnuematics {

	Solenoid winch = new Solenoid(2);
	Solenoid lockWinch = new Solenoid(5);
	Solenoid dump1 = new Solenoid(0);
	Solenoid dump2 = new Solenoid(7);
	
	public void lftSwitch(boolean[] climb) {
		if(climb[0]) {
			winch.set(true);
		}else {
			winch.set(false);
		}
		if(climb[1]) {
			lockWinch.set(true);
		}else {
			lockWinch.set(false);
		}
		if(climb[2]) {
			dump1.set(true);
			//dump2.set(true);
		}
		else {
			dump1.set(false);
			//dump2.set(false);
		}
		if(climb[3]) {
			dump2.set(true);
		}else {
			dump2.set(false);
		}
		
	}
}
