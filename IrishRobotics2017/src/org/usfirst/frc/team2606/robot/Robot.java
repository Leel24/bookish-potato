package org.usfirst.frc.team2606.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends IterativeRobot {

	// Autonomous Routine Command Chooser and Counters
	public static int				autonomousLoopCounter		= 0;

	// Drive
	private static double			controllerDeadZone			= 0.06;

	// Default Shooter Speed 0 - 100%
	private static double			DEFAULT_MIN_SHOOTER_SPEED	= 0.55;

	// Size of camera image
	private static int				IMG_WIDTH					= 640;
	private static int				IMG_HEIGHT					= 480;

	// Thread Lock object to used when obtaining the image & Toggle for enabling vision
	private final Object			imgLock						= new Object();
	public static final double		VISION_CAMERA_HEIGHT		= 2.0; // Ft
	public static final int			HORIZON_LINE_OFSET			= -100; // Pixels
	public static final double		HIGHGOAL_HEIGHT				= 8.08333 ; // ft
	private static final boolean	ENABLE_VISION				= false;

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Vision Code
		if (ENABLE_VISION) {
			// Regular camera
			UsbCamera cameraRaw = CameraServer.getInstance().startAutomaticCapture("Normal Camera", 0);
			cameraRaw.setResolution(IMG_WIDTH, IMG_HEIGHT);

			// Reflective detection camera
			UsbCamera cameraReflective = CameraServer.getInstance().startAutomaticCapture("Reflective Camera Raw", 1);
			cameraReflective.setResolution(IMG_WIDTH, IMG_HEIGHT);
			cameraReflective.setBrightness(0);

			// Put the Reflection images to the dashboard
			CvSource cvOutput = CameraServer.getInstance().putVideo("Reflective Detection Output", IMG_WIDTH, IMG_HEIGHT);
			CvSink cvSink = CameraServer.getInstance().getVideo(cameraReflective);

			// Create the Mat object to work with the output of the detection
			Mat mat = new Mat();

			// Create the vision detection thread and get the output of each frame
			VisionThread visionThread = new VisionThread(cameraReflective, new GripPipeline(), pipeline -> {

				// Bounding Rectangle
					Rect r = new Rect();
					double averageX = 0.0;
					double averageY = 0.0;
					double maxY = Integer.MIN_VALUE;
					double minY = Integer.MAX_VALUE;
					double leftX = Integer.MAX_VALUE;
					double rightX = Integer.MIN_VALUE;
					// NetworkTable contoures = NetworkTable.getTable("GRIP/ContoursReport");

					// Draw a rectangle around the detection and get the center
					if (pipeline.convexHullsOutput().size() >= 2) {
						synchronized (imgLock) {
							for (MatOfPoint p : pipeline.convexHullsOutput()) {
								r = Imgproc.boundingRect(p);
								
								// if the Contours are in the top half of the image, include it in target average
								if (r.y + r.height / 2 <= pipeline.cvResizeOutput().height() / 2) {
									averageX += r.x + r.width / 2;
									averageY += r.y + r.height / 2;
									if (r.y < maxY) maxY = r.y;
									if (r.y + r.height > minY) minY = r.y + r.height;
									if (r.x < leftX) leftX = r.x;
									if (r.x + r.width > rightX) rightX = r.x + r.width;
								}
							}
							
							averageX /= pipeline.convexHullsOutput().size();
							averageY /= pipeline.convexHullsOutput().size();  
							
						}
					}

					// Get the processed frame and put it to the dashboard
					cvSink.grabFrame(mat);
					pipeline.process(mat);

					// Draw the rectangle and center of detection on the image
					Mat processedImage = pipeline.maskOutput();
					Imgproc.rectangle(processedImage, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height), new Scalar(255, 0, 0), 1);
					Imgproc.drawContours(processedImage, pipeline.filterContoursOutput(), 0, new Scalar(0, 0, 255), 1);
					cvOutput.putFrame(processedImage);
				});
			visionThread.start();
		}
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example) or additional
	 * comparisons to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousLoopCounter = 0;
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run(); // Run any called commands

		// 50 Autonomous Loops = 1 Second
		autonomousLoopCounter++;
	}

	@Override
	public void teleopInit() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run(); // Run any called commands

		// ----- Drive Code -----

		/*// B button pressed & Right Trigger is beyond Halfway pressed drive in reverse
		if (RobotMap.xboxController.getRawButton(2) && RobotMap.xboxController.getRawAxis(3) > 0.5)
			RobotMap.robotDrive.tankDrive(-1.0, -1.0);

		// B button pressed, Drive full Speed Strait Forward
		else if (RobotMap.xboxController.getRawButton(2))
			RobotMap.robotDrive.tankDrive(1.0, 1.0);
		
		// Y Axis of Left joystick controls the Left wheels, Y Axis of Right joystick controls the Right wheels
		else if (RobotMap.xboxController.getRawAxis(5) > controllerDeadZone || RobotMap.xboxController.getRawAxis(1) > controllerDeadZone
				|| RobotMap.xboxController.getRawAxis(5) < -controllerDeadZone || RobotMap.xboxController.getRawAxis(1) < -controllerDeadZone) {
			RobotMap.robotDrive.tankDrive(-RobotMap.xboxController.getRawAxis(1), -RobotMap.xboxController.getRawAxis(5));

			// If the right trigger is pressed past half, reverse the whole drive
			if (RobotMap.xboxController.getRawAxis(3) > 0.5)
				RobotMap.robotDriveReverse.tankDrive(-RobotMap.xboxController.getRawAxis(1), -RobotMap.xboxController.getRawAxis(5));
		}
		else RobotMap.robotDrive.tankDrive(0,  0);
*/
		
		if(RobotMap.xboxController.getRawButton(2))
    		RobotMap.robotDrive.drive(1.0, 0);
    	//if right trigger is being held past halfway, the drive-train will reverse
    	else if(RobotMap.xboxController.getRawAxis(3)>0.5)
    		RobotMap.robotDrive.tankDrive(RobotMap.xboxController.getRawAxis(5), RobotMap.xboxController.getRawAxis(1));
    	//left stick's Y axis controls left wheel speed, right stick's Y axis controls right wheel speed 
    	else
    		RobotMap.robotDrive.tankDrive(RobotMap.xboxController.getRawAxis(1)*-1, RobotMap.xboxController.getRawAxis(5)*-1);
		
		
		
		
		
		// ----- Climber Code -----
		// Climb Up
		if (RobotMap.xboxController.getRawButton(5))
			RobotMap.climberMotor.set(-1);

		// Climb down only if A Button is pressed (one-way)
		else if (RobotMap.xboxController.getRawButton(6) && RobotMap.xboxController.getRawButton(8))
			RobotMap.climberMotor.set(1);

		// Stop Climber if no bumper is pressed
		else
			RobotMap.climberMotor.stopMotor();

		// ----- Feeder Code ----- (flight stick)
		// If button 3 pressed, feed ball in
		if (RobotMap.flightStick.getRawButton(3))
			RobotMap.feederMotor.set(1.0);
		// If button 2 is pressed go in reverse
		if (RobotMap.flightStick.getRawButton(2))
			RobotMap.feederMotor.set(-1.0);
		// otherwise stop motor
		else
			RobotMap.feederMotor.stopMotor();
		
		// ----- Shooter Code -----
		// Set the speed of the Shooter Motor to flight stick lever
		if (RobotMap.flightStick.getRawAxis(2) != 0) {
			// If button 7 is pressed, go in reverse
			if (RobotMap.flightStick.getRawButton(7))
				RobotMap.shooterMotor.set(-(RobotMap.flightStick.getRawAxis(2) - 1) * 0.5);
			// otherwise go forward
			else
				RobotMap.shooterMotor.set((RobotMap.flightStick.getRawAxis(2) - 1) * 0.5);
		}
		// Stop the motor otherwise
		else
			RobotMap.shooterMotor.stopMotor();

		// Put the shooter motor % of power to the Dashboard variables tab
		SmartDashboard.putNumber("Shooter Speed %", RobotMap.shooterMotor.get());
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
