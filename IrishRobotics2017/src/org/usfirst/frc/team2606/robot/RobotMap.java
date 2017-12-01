package org.usfirst.frc.team2606.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	// Xbox One Controller
	public static Joystick xboxController = new Joystick(0); // Xbox One Controller Joystick Object
	public static Joystick flightStick = new Joystick(1); // Logitech Flight Stick
	
	// Drive Motor Speed Controllers
	public static SpeedController frontLeftMotor = new Victor(0); // Front Left Drive Motor
	public static SpeedController frontRightMotor = new Victor(2); // Front Right Drive Motor
	public static SpeedController rearLeftMotor = new Victor(1); // Rear Left Drive Motor
	public static SpeedController rearRightMotor = new Victor(3); // Rear Right Drive Motor
	
	// Special System Motor Speed Controllers
	public static SpeedController shooterMotor = new Victor(4); // Shooter Wheel Motor Controller
	public static SpeedController feederMotor = new Jaguar(8); // Shooter Ball Feeding Motor Controller
	public static SpeedController climberMotor = new Spark(6); // Robot Climbing Motor Controller
	
	// Shooter RPM Detection Switch
	public static DigitalInput shooterRPMSwitch = new DigitalInput(0); // Detects and Fires A Signal When A Ball Is Shot
	public static Counter shooterRPMCounter = new Counter(shooterRPMSwitch);
	
	public static RobotDrive robotDrive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor); // Robot Drive Object
	public static RobotDrive robotDriveReverse = new RobotDrive(frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor); // Robot Drive Object
}
