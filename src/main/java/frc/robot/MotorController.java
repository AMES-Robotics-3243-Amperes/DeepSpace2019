package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;

public class MotorController {

	double switchTime;
	double scaleTime;
	int timeout = 10;
	int PIDIdx = 0;

	Float[] alter = new Float[2];

	WPI_TalonSRX driveM1 = new WPI_TalonSRX(2); // right
	WPI_TalonSRX driveM2 = new WPI_TalonSRX(3); // left
	WPI_TalonSRX driveM3 = new WPI_TalonSRX(1); // right
	WPI_TalonSRX driveM4 = new WPI_TalonSRX(4); // left

	VictorSP greenLight = new VictorSP(4);

	Spark collect1 = new Spark(2);
	Spark collect2 = new Spark(3);

	Spark lift1 = new Spark(0);
	Spark lift2 = new Spark(1);

	DigitalOutput underGlow = new DigitalOutput(0);

	boolean init = false;

	public void PIDinit() {
		alter[0] = 0.0f;
		alter[1] = 0.0f;

		// Init drive 1 PID
		driveM1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
		driveM1.configNominalOutputForward(0, timeout);
		driveM1.configNominalOutputReverse(0, timeout);
		driveM1.configPeakOutputForward(1, timeout);
		driveM1.configPeakOutputReverse(-1, timeout);
		driveM1.config_kF(PIDIdx, .3, timeout);
		driveM1.config_kP(PIDIdx, 1, timeout);
		driveM1.config_kI(PIDIdx, 0, timeout);
		driveM1.config_kD(PIDIdx, 0, timeout);

		// Init drive 3 PID
		driveM2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
		driveM2.configNominalOutputForward(0, timeout);
		driveM2.configNominalOutputReverse(0, timeout);
		driveM2.configPeakOutputForward(1, timeout);
		driveM2.configPeakOutputReverse(-1, timeout);
		driveM2.config_kF(PIDIdx, .3, timeout);
		driveM2.config_kP(PIDIdx, 1, timeout);
		driveM2.config_kI(PIDIdx, 0, timeout);
		driveM2.config_kD(PIDIdx, 0, timeout);

		// get drive 2 and 4 to follow
		driveM3.follow(driveM1);
		driveM4.follow(driveM2);
	}

	public void drive(Double[] val, boolean Vision) {

		if (!init) {
			PIDinit();
			init = true;
		}
		if (!Vision) {
			driveM1.set(ControlMode.PercentOutput, 0.8 * val[1]);
			driveM2.set(ControlMode.PercentOutput, -0.8 * val[0]);
		}
	}

	public void light() {
		greenLight.set(.6);
	}

	public void autoDrive(Double[] value) {
		driveM1.set(ControlMode.PercentOutput, value[0]);
		driveM2.set(ControlMode.PercentOutput, -value[1]);
	}

	public void ballControl(Boolean suck, Boolean push) {
		double speedy = 0.80;
		double spitter = 0.55;
		if (suck && !push) {
			collect1.set(speedy);
			collect2.set(-speedy);
		} else if (!suck && push) {
			collect1.set(-spitter);
			collect2.set(spitter);
		} else {
			collect1.set(0);
			collect2.set(0);
		}
	}

	public void lift(Double val) {
		lift1.set(val);
		lift2.set(-val);
	}

	public void autoDrives(boolean in) {
		if (in) {
			driveM1.set(-.2);
			driveM2.set(.2);
			driveM3.set(-.2);
			driveM4.set(.2);
		}
		if (!in) {
			driveM1.set(0);
			driveM2.set(0);
			driveM3.set(0);
			driveM4.set(0);
		}
	}
	//For Vision 2019
	public void setVision(boolean value, double x, double v, double area) {
		float KpSteering = 0.00001f;
		float KpSteering2 = 0.015f;
		float KpDistance = 0.25f;
		//float KpDistance2 = 0.8f;
		float min_command = 0.05f;
		float refArea = 1.15f;

		if (value == true)
		{
		        double heading_error = (float) x;
				double steering_adjust = 0.0d;
				double distance_error = (float) area;
				double distance_adjust = 0.0d;
				double maxDistAdjust = 0.35d;
				double maxAngAdjust = 1.0d;

				if (v == 0.0)
				// We don't see the target, seek for the target by spinning in place at a safe speed.
					{
					steering_adjust = 0.25f;
					}
			
				else if (x > 1.0 && v != 0)
				// We do see the target, execute aiming code
		        {
					steering_adjust = (KpSteering*Math.pow(heading_error, 3) + KpSteering2*heading_error) - min_command;
		        }
		        else if (x < 1.0 && v != 0)
		        {
					steering_adjust = (KpSteering*Math.pow(heading_error, 3) + KpSteering2*heading_error) + min_command;
				}
				
				if (v == 0.0)
				// We don't see the target, seek for the target by spinning in place at a safe speed.
					{
					distance_adjust = 0.0f;
					}
			if(area != 0){
				if (area > refArea)	//when tape closer
				// We don't see the vision target at the right distance, so we go closer/farther away from it
		        {
						distance_error = refArea - distance_error;
						distance_adjust = distance_error*KpDistance;
						//distance_error = 0.05/distance_error; //f;lakdfjlds
						//distance_error = -0.1*Math.pow(distance_error, 1.25);
						//distance_adjust = -(KpDistance*Math.pow(distance_error, 3.75) + KpDistance2*distance_error) - min_command;
				}
		        else if (area < refArea)	//when tape farther away
		        {
						distance_error = refArea - distance_error;
						//distance_adjust = distance_error*KpDistance;
						distance_adjust = (0.09/Math.pow(KpDistance*distance_error, 2));
						//distance_error = 0.05/distance_error; //flkwjefoiwj
						//distance_error = -0.5*Math.pow(distance_error, 1.25);
						//distance_adjust = -(KpDistance*Math.pow(distance_error, 3.75) + KpDistance2*distance_error) + min_command;
					}
			}

			/*if(distance_adjust > 0){
				distance_adjust = 1.5*Math.pow(distance_adjust, 2);
			}*/
			
				distance_adjust = Math.tanh(distance_adjust) * maxDistAdjust;
				steering_adjust = Math.tanh(steering_adjust) * maxAngAdjust;
				//System.out.println("steering " + steering_adjust);
				System.out.println("distance " + distance_adjust);
				//distance_adjust = 0;
				driveM1.set(alter[0] + steering_adjust + distance_adjust);
				driveM2.set(alter[1] + steering_adjust - distance_adjust);
				driveM3.follow(driveM1);
				driveM4.follow(driveM2);

			

		}

		

	}

	public void rave(boolean input) {
		underGlow.set(input);
	}
}
