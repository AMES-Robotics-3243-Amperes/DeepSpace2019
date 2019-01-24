package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InputManager {
	
	Joystick firstInput = new Joystick(0);
	Joystick secondInput = new Joystick(1);
	Double[] stickData = new Double[3];
	Double liftVal;
	Boolean switchDrv = false;
	Boolean scaleDrv = false;
	boolean climb[] = new boolean[4];
	int scaleHeight = 5;
	boolean suck = false;
	boolean push = false;
	boolean lightStatus = false;
	boolean antiTurbo = false;
	boolean visionButton, ballPosition = false;
	
	boolean getAntiTurbo() {
		antiTurbo = firstInput.getRawButton(9); //change to actual button
		return antiTurbo;
	}
	Double[] getMoveInput(Boolean antiTurbo) {

		//stickData[0] = ((Math.pow(firstInput.getRawAxis(1), 2)) * Math.signum(firstInput.getRawAxis(1)));
		//stickData[1] = ((Math.pow(firstInput.getRawAxis(3), 2)) * Math.signum(firstInput.getRawAxis(3)));
		if(antiTurbo) {
			stickData[0] = .5*firstInput.getRawAxis(1);
			stickData[1] = .5*firstInput.getRawAxis(3);	
		}
		else {
		stickData[0] = firstInput.getRawAxis(1);
		stickData[1] = firstInput.getRawAxis(3);
		}
		return stickData;
	}	
	
	Double[] deadZone(Double[] in) {
		if(-.09 < in[0] && in[0] < .09) {
			in[0] = .0;
		}
		if(-.09 < in[1] && in[1] < .09) {
			in[1] = .0;
		}
		return in;
	}
	
	Boolean getAutoCube() {
		return firstInput.getRawButton(2);
	}
	
	Boolean getSuck() {
		suck = firstInput.getRawButton(8);
		return suck;
	}
	Boolean getPush() {
		push = firstInput.getRawButton(7);
		return push;
	}
	Double getLiftManual() {
		//Divided the lifting variable speed by 2 to so the the chain doesn't come off again
		liftVal = ((Math.pow(secondInput.getRawAxis(3), 3))*0.66666666667);// * Math.signum(secondInput.getRawAxis(3)));
		//liftVal = secondInput.getRawAxis(3);
		return liftVal;
	}
	boolean[] getClimbControl(){
		climb[0] = secondInput.getRawButton(2);
		climb[1] = secondInput.getRawButton(3);
		climb[2] = secondInput.getRawButton(4);
		climb[3] = secondInput.getRawButton(1);
		return climb;
	}	
	Boolean getLit() {
		if(secondInput.getRawButton(10)) {
			lightStatus = false;
		}
		else {
			lightStatus = true;
		}
		return lightStatus;
	}

	public boolean getVision() {
		
		visionButton = firstInput.getRawButton(5);
		
		return visionButton;
	}

	public boolean getBallPosition(){

		ballPosition = firstInput.getRawButton(0);

		return ballPosition;
	}

}
