// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.subsystems.DriveBaseSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.HopperCommand;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.CDSForwardCommand;
import frc.robot.commands.CDSReverseCommand;


 // This class is where the bulk of the robot should be declared. Since Command-based is a
 // "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 // perieodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 // subsystems, commands, and button mappings) should be declared here.

public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private final Joystick mDriverJoystick = new Joystick(Constants.kPortNumber);
  private JoystickButton[] mButtons = new JoystickButton[11];


  // subsystems
  private final DriveBaseSubsystem mDriveBaseSubsystem = new DriveBaseSubsystem(mDriverJoystick);
  private final CDSSubsystem mCDSSubsystem = new CDSSubsystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();


  // commands
  private final DriveBaseTeleopCommand mDriveBaseTeleopCommand = new DriveBaseTeleopCommand(mDriveBaseSubsystem);
  private IntakeForwardCommand mIntakeForwardCommand = new IntakeForwardCommand(mIntakeSubsystem);
  private IntakeReverseCommand mIntakeReverseCommand = new IntakeReverseCommand(mIntakeSubsystem);
  private HopperCommand mHopperCommand = new HopperCommand(mHopperSubsystem);

  // auton
  private ArrayList<Trajectory> mTrajectories;  // multiple trajectories
  
  private CDSForwardCommand mCDSForwardCommand = new CDSForwardCommand(mCDSSubsystem);
  private CDSReverseCommand mCDSReverseCommand = new CDSReverseCommand(mCDSSubsystem);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    // Configure the button bindings
    for (int i = 1; i < mButtons.length; i++) {
      mButtons[i] = new JoystickButton(mDriverJoystick, i);
    }

    configureButtonBindings();
    
    try {
      initializeTrajectories();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    mDriveBaseSubsystem.setDefaultCommand(mDriveBaseTeleopCommand);
  }

  // Use this method to define your button->command mappings. Buttons can be created by
  // instantiating a {@link GenericHID} or one of its subclasses ({@link
  // edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
  // edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  private void configureButtonBindings() {
    mButtons[Constants.kLeftBumperButton].whileHeld(mIntakeForwardCommand);
    mButtons[Constants.kRightBumperButton].whileHeld(mIntakeReverseCommand);
    mButtons[Constants.kXButton].whileHeld(mCDSForwardCommand);
    mButtons[Constants.kBButton].whileHeld(mCDSReverseCommand);
  }

  public void initializeTrajectories() throws IOException {
    String trajectoryJSON = "deploy/One.wpilib.json";
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    mTrajectories.add(trajectory);

    trajectoryJSON = "deploy/Two.wpilib.json";
    trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    mTrajectories.add(trajectory);

    trajectoryJSON = "deploy/Three.wpilib.json";
    trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    mTrajectories.add(trajectory);

    trajectoryJSON = "deploy/Four.wpilib.json";
    trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    mTrajectories.add(trajectory);
  }

  private int trajectoryIndex = 0;
            
  // Use this to pass the autonomous command to the main {@link Robot} class.
  // @return the command to run in autonomous
  public Command getAutonomousCommand() {

    //Ramsete Command for Pathweaver
    RamseteCommand ramseteCommand =
    new RamseteCommand(
        mTrajectories.get(trajectoryIndex++),
        mDriveBaseSubsystem::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), //Fix these constants by
        //characterizing the robot
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        
        mDriveBaseSubsystem::getWheelSpeeds,
        new SparkMaxPIDController(Constants.kPDriveVel, 0, 0),
        new SparkMaxPIDController(Constants.kPDriveVel, 0, 0),
        //RamseteCommand passes volts to the callback
        mDriveBaseSubsystem::setAutonVolts,
        mDriveBaseSubsystem);
      return null;

    // An ExampleCommand will run in autonomous
    
  }


  // TODO: create get methods for other subsystems to pass into TabContainer, or find a more efficient way
  public DriveBaseSubsystem getDriveBase() {
    return mDriveBaseSubsystem;
  }

  public DriveBaseTeleopCommand getDefaulDriveCommand() {
    return mDriveBaseTeleopCommand;
  }
}
