
package org.usfirst.frc.team2635.robot;

import com.kauailabs.navx.frc.AHRS;
import com.lakemonsters2635.sensor.interfaces.BaseSensor;
import com.lakemonsters2635.sensor.interfaces.IOutput;
import com.lakemonsters2635.sensor.modules.OutputAngleFromImage;
import com.lakemonsters2635.sensor.modules.SensorAngleFromCameraColorSense;
import com.lakemonsters2635.sensor.modules.SensorUnwrapper;
import com.lakemonsters2635.util.ImageGrabber;
import com.ni.vision.NIVision;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	NIVision.Range RETRO_HUE_RANGE = new NIVision.Range(0, 5);	//Default hue range for yellow tote
	NIVision.Range RETRO_SAT_RANGE = new NIVision.Range(0, 10);	//Default saturation range for yellow tote
	NIVision.Range RETRO_VAL_RANGE = new NIVision.Range(250, 255);	//Default value range for yellow tote

	int CAMERA_RESOLUTION_X = 640;
	double VIEW_ANGLE = 64.0; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
	double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
	IOutput<Double, NIVision.Image> angleSensor;
	ImageGrabber camera;
	final int REAR_RIGHT_CHANNEL = 3;
	final int FRONT_RIGHT_CHANNEL = 4;
	final int REAR_LEFT_CHANNEL = 1;
	final int FRONT_LEFT_CHANNEL = 2;
	String KEY_P = "P";
	String KEY_I = "I";
	String KEY_D = "D";

	CANTalon rearRightMotor;
	CANTalon frontRightMotor;
	CANTalon rearLeftMotor;
	CANTalon frontLeftMotor;
	
	RobotDrive drive;
    
	Joystick joystick;
    
	PIDController pid;
    
	AHRS navx;
    SensorUnwrapper unwrapper;

	public void robotInit()
	{
		SmartDashboard.putNumber(KEY_P, 0.0);
		SmartDashboard.putNumber(KEY_I, 0.0);
		SmartDashboard.putNumber(KEY_D, 0.0);
        int session = NIVision.IMAQdxOpenCamera("cam0",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        //By default visionSensor will grab the angle every 20ms
		angleSensor = new OutputAngleFromImage(CAMERA_RESOLUTION_X,  VIEW_ANGLE, RETRO_HUE_RANGE, RETRO_SAT_RANGE, RETRO_VAL_RANGE, AREA_MINIMUM);
		
		camera = new ImageGrabber(session, true);
		rearRightMotor = new CANTalon(REAR_RIGHT_CHANNEL);
		frontRightMotor = new CANTalon(FRONT_RIGHT_CHANNEL);
		rearLeftMotor = new CANTalon(REAR_LEFT_CHANNEL);
		frontLeftMotor = new CANTalon(FRONT_LEFT_CHANNEL);
		drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		
		joystick = new Joystick(0);
		
		navx = new AHRS(SerialPort.Port.kMXP);
		unwrapper = new SensorUnwrapper(180.0, new SensorNavxAngle(navx));
		
		pid = new PIDController(0.0, 0.0, 0.0, unwrapper, new PIDOutputDrive(drive));
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
	public void autonomousInit()
	{
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic()
	{

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic()
	{
		SmartDashboard.putNumber("Current Angle", unwrapper.sense());
		if(joystick.getRawButton(1))
		{
			if(!pid.isEnabled())
			{
				double angleToTarget = angleSensor.getOutput(camera.getImage());
				SmartDashboard.putNumber("Angle to target", angleToTarget);
				double setPoint = (unwrapper.sense() + angleToTarget);
				SmartDashboard.putNumber("Setpoint", setPoint);
				pid.setPID(SmartDashboard.getNumber(KEY_P), SmartDashboard.getNumber(KEY_I), SmartDashboard.getNumber(KEY_D));
				System.out.println(SmartDashboard.getNumber(KEY_I));
				pid.setSetpoint(setPoint);
				pid.enable();
			}
			
		}
		else
		{
			if(pid.isEnabled())
			{
				pid.disable();
			}
			drive.arcadeDrive(-joystick.getRawAxis(1), -joystick.getRawAxis(0));
		}
			
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic()
	{

	}

}
