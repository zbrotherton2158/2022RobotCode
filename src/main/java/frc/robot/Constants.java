// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;


// The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
// constants. This class should not be used for any other purpose. All constants should be declared
// globally (i.e. public static). Do not put anything functional in this class.

// <p>It is advised to statically import this class (or one of its inner classes) wherever the
// constants are needed, to reduce verbosity.

public final class Constants {
        
    // Constants for wheel motors

    // Actual IDs on robot, used to activate the right motors
    // TODO: Values to be changed, these are placeholder values for now
    public static final int kDriveRightFront = 0;
    public static final int kDriveRightRear = 0;
    public static final int kDriveLeftFront = 0;
    public static final int kDriveLeftRear = 0;
    
    // This is used for organizational purposes (Note numbers 0-3 to distinguish between the 4 motors)
    public static final int kDriveLeftFrontIndex = 0;
    public static final int kDriveLeftRearIndex = 1;
    public static final int kDriveRightFrontIndex = 2;
    public static final int kDriveRightRearIndex = 3;

    public static final int kDriveBaseCurrentLimit = 40;
    //Pathweaver constants
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    //TODO: Calibrate robot with correct values - These are just placeholers
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(0.5); // Replace 0.5 with track width in meters
    //Controller constants
    public static final int kDBJoystickPort = 0;

    public static final int kDBLeftJoystickAxisX = 0;
    public static final int kDBLeftJoystickAxisY = 1;
    public static final int kDBRightJoystickAxisX = 2;
    public static final int kDBRightJoystickAxisY = 3;

    // Intake Contstants 
    public static final int kIntakeMotorOneID = 11;
    public static final int kIntakeMotorTwoID = 12;
    public static final double kIntakeMotorSpeed = 0.25;

    //Hopper Constants
    public static final int kHopperMotorThreeID = 13;
    public static final double kHopperMotorSpeed = 0.25;
    
    //Joystick Constants
    public static final int kPortNumber = 0;
    public static final int kJoystickButtonNumberOne = 1;
    public static final int kJoystickButtonNumberTwo = 2;
    public static final int kAButton = 1;           // Button for hopper
    public static final int kRightBumperButton = 5; // Button for intake 
    public static final int kLeftBumperButton = 6;  // Button to reverse intake
}
