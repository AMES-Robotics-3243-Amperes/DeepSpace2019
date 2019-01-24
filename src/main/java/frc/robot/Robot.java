package frc.robot;

import javax.sound.sampled.BooleanControl;

//import com.sun.tools.javac.resources.version;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.j
 */
public class Robot extends TimedRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	//Compressor C = new Compressor();
	//Pnuematics P = new Pnuematics();
	InputManager IM = new InputManager();
	MotorController MC = new MotorController();

	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry camMode;
	NetworkTableEntry pipeline;
	NetworkTableEntry tx;
	NetworkTableEntry ty;
	NetworkTableEntry ta;
	NetworkTableEntry tv;

	boolean autoRun = false;
	boolean wtfAuto = false;
	boolean autoFrwd = false;
	String gameData;
	Double[] forward = new Double[2];
	Double[] left = new Double[2]; 
	Double[] right = new Double[2]; 
	//Auto A = new Auto();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		chooser.addDefault("Just Forward", defaultAuto);
		chooser.addObject("Switch drop", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		forward[0] = -.4; forward[1] = -.4;
		left[0] = -.5; left[1] = .5;
		right[0] = .5; right[1] = -.5;
		//MC.light();
		//MC.PIDinit();
		//A.stopVelocity();
		//C.setClosedLoopControl(true);

		camMode = table.getEntry("camMode");
		pipeline = table.getEntry("pipeline");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		tv = table.getEntry("tv");
		ta = table.getEntry("ta");

		camMode.setFlags(1);
		pipeline.setNumber(0);

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		if(chooser.getSelected() == defaultAuto) {
			autoFrwd = true;
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);

		//post to smart dashboard periodically
		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
		

		if(!autoRun) {
			if(autoFrwd){
				if(!wtfAuto) {
					Timer.delay(11);
					wtfAuto = true;
				}
				if(wtfAuto) MC.autoDrive(forward);
			}else {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
			if(gameData.length() > 0) {	
				if(gameData.charAt(0) == 'L'){
					MC.autoDrive(left);
					Timer.delay(.5);
					MC.autoDrive(forward);
					Timer.delay(6);
					MC.ballControl(false, true);
					autoRun = true;
				}else if(gameData.charAt(0) == 'R') {
					MC.autoDrive(right);
					Timer.delay(.5);
					MC.autoDrive(forward);
					Timer.delay(6);
					MC.ballControl(false, true);
					autoRun = true;
				}
			}
			}
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopInit() {
		
		//test
	}

	public void teleopPeriodic() {
		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double v = tv.getDouble(0.0);
		double area = ta.getDouble(0.0);

		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
		SmartDashboard.putNumber("Limelight Pipeline", (double) pipeline.getNumber(-1));
		SmartDashboard.putNumber("See Light?", v);
		//System.out.println(v);
		MC.setVision(IM.getVision(), x,v, area);
		

		//if(IM.getAutoCube()) {
		//	MC.autoDrive(SmartDashboard.getNumber("cubeDegrees", 0), SmartDashboard.getNumber("cubeDistance", 0));
		//}
		MC.drive(IM.getMoveInput(IM.getAntiTurbo()), IM.getVision());
		//P.lftSwitch(IM.getClimbControl());
		MC.lift(IM.getLiftManual());
		MC.ballControl(IM.getSuck(), IM.getPush());
		//MC.light();
		MC.rave(IM.getLit());
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		//MC.light();
	}
}

