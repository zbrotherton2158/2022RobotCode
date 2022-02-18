// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.subsystems.DriveBaseSubsystem;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.math.controller.RamseteController;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.ShooterPrime;
import frc.robot.commands.CDSForwardCommand;
import frc.robot.commands.CDSReverseCommand;
import frc.robot.commands.ClimbDOWNCommand;
import frc.robot.commands.ClimbUPCommand;

 // This class is where the bulk of the robot should be declared. Since Command-based is a
 // "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 // perieodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 // subsystems, commands, and button mappings) should be declared here. 
 

public class RobotContainer {
  public static ShuffleboardTab debugTab;

  // The robot's subsystems and commands are defined here...
  private static final Joystick driverJoystick = new Joystick(Constants.portNumber0);
  private JoystickButton[] buttons = new JoystickButton[13];


  // subsystems

  private static ClimbSubsystem climbSubsystem;
  private static DriveBaseSubsystem driveBaseSubsystem;
  private static CDSSubsystem CDSSubsystem;
  private static IntakeSubsystem intakeSubsystem; 
  private static ShooterSubsystem shooterSubsystem;
  private static LimelightSubsystem limelightSubsystem;

  // commands
  private DriveBaseTeleopCommand driveBaseTeleopCommand;
  private IntakeForwardCommand intakeForwardCommand;
  private IntakeReverseCommand intakeReverseCommand;
  private ClimbUPCommand climbUPCommand;
  private ClimbDOWNCommand climbDOWNCommand;

  // private BeamBreakCommand beamBreakCommand = new BeamBreakCommand(intakeSubsystem);
  private ShooterPrime shooterPrime;
  private CDSForwardCommand CDSForwardCommand;
  private CDSReverseCommand CDSReverseCommand;
  private LimelightAlign limelightAlign;

  //auton
  private List<Trajectory> mTrajectories;  // multiple trajectories
  private int trajectoryIndex = 0;
  private Trajectory trajectory;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    debugTab = Shuffleboard.getTab("debug");

    mTrajectories = new ArrayList<Trajectory>();

    initSubsystems();
    initCommands();

    // initialize the button bindings
    for (int i = 1; i < buttons.length; i++) {
      buttons[i] = new JoystickButton(driverJoystick, i);
    }
    configureButtonBindings();
    
    initializeTrajectories();
  }

  private void initSubsystems() {
    // subsystems
    for (Constants.Subsystems sub : Constants.Subsystems.values()) {
      if (sub.isEnabled()) {

        //System.out.println((String) k + " " + subSysEnables.get((String) k));
        switch (sub.toString()) {
          case "DriveBaseSubsystem": 
          {
            System.out.println("Drivebase enabled");
            driveBaseSubsystem = new DriveBaseSubsystem(driverJoystick, false);
            break;
          }
          case "CDSSubsystem": 
          {
            System.out.println("CDS enabled");
            CDSSubsystem = new CDSSubsystem();
            //CDSForwardCommand = new CDSForwardCommand(CDSSubsystem, shooterSubsystem);
            //CDSReverseCommand = new CDSReverseCommand(CDSSubsystem, shooterSubsystem);
            //CDSSubsystem.getAllianceColor();
            break;
          }
          case "IntakeSubsystem":
          {
            System.out.println("Intake enabled");
            intakeSubsystem = new IntakeSubsystem(); 
            break;
          }
          case "ShooterSubsystem":
          {
            System.out.println("Shooter enabled");
            shooterSubsystem = new ShooterSubsystem();
            break;
          }
          case "LimelightSubsystem":
          {
            System.out.println("Limelight enabled");
            limelightSubsystem = new LimelightSubsystem();
            break;
          }
          case "ClimbSubsystem":
          {
            System.out.println("Climb enabled");
            
            climbSubsystem = new ClimbSubsystem();
            climbUPCommand = new ClimbUPCommand(climbSubsystem);
            climbDOWNCommand = new ClimbDOWNCommand(climbSubsystem);
            break;
          }

        }
      }
    }
  }

  private void initCommands() {
    //Initializes commands based on enabled subsystems
    if(driveBaseSubsystem != null){
      driveBaseTeleopCommand = new DriveBaseTeleopCommand(driveBaseSubsystem);
      driveBaseSubsystem.setDefaultCommand(driveBaseTeleopCommand);
    }
    if(CDSSubsystem != null && shooterSubsystem != null){
      CDSForwardCommand = new CDSForwardCommand(CDSSubsystem, shooterSubsystem);
      CDSReverseCommand = new CDSReverseCommand(CDSSubsystem, shooterSubsystem);
    }
    if(intakeSubsystem != null){
      intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem);
      intakeReverseCommand = new IntakeReverseCommand(intakeSubsystem);
    }
    if(shooterSubsystem != null && limelightSubsystem != null && CDSSubsystem != null){
      shooterPrime = new ShooterPrime(shooterSubsystem, limelightSubsystem, CDSSubsystem);
    }
    if(limelightSubsystem != null && driveBaseSubsystem != null){
      limelightAlign = new LimelightAlign(limelightSubsystem, driveBaseSubsystem);
    }
  }

  // Use this method to define your button->command mappings. Buttons can be
  // created by
  // instantiating a {@link GenericHID} or one of its subclasses ({@link
  // edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
  // it to a {@link
  // edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  private void configureButtonBindings() {

    // Intake
    if(intakeForwardCommand != null && intakeReverseCommand != null) {
      buttons[Constants.LBumper].whileHeld(intakeForwardCommand);
      buttons[Constants.RBumper].whileHeld(intakeReverseCommand);
    }

    // Shooter
    if (shooterSubsystem != null && shooterPrime != null) {
      buttons[Constants.backButton].whenPressed(shooterPrime);
      buttons[Constants.LJoystickButton].whenPressed(new InstantCommand(shooterSubsystem::cycleAimModeUp, shooterSubsystem));
      buttons[Constants.RJoystickButton].whenPressed(new InstantCommand(shooterSubsystem::cycleAimModeDown, shooterSubsystem));
    }

    //CDS
    if (CDSSubsystem != null && shooterSubsystem != null) {
      CDSForwardCommand = new CDSForwardCommand(CDSSubsystem, shooterSubsystem);
      CDSReverseCommand = new CDSReverseCommand(CDSSubsystem, shooterSubsystem);
      CDSSubsystem.senseColor();
    }
    
    if (CDSForwardCommand != null && CDSReverseCommand != null) {
      buttons[Constants.LTriggerButton].whileHeld(CDSForwardCommand);
      buttons[Constants.RTriggerButton].whileHeld(CDSReverseCommand);
    }

	  // Limelight
	  if (limelightAlign != null) {
      buttons[Constants.startButton].whenPressed(limelightAlign);
    }

    if (climbSubsystem != null)
    {
      buttons[Constants.AButton].whileHeld(climbUPCommand);
      buttons[Constants.XButton].whileHeld(climbDOWNCommand);
    }
  } 

  private void initializeTrajectories() {
    // auton with just a one straight path
    String trajectoryJSON = "paths/RightCurve.wpilib.json";
    try { 
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON); // goes to scr/main/deploy/paths
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    // method to intialize multiple trajectories in one auton route
    // TODO: some sort of sendable chooser on shuffle board get the desire route
    String routeName = "RouteA";

    File f = new File("Autos/" + routeName);  // get file with all the names of the path for the auton routine
    Scanner fileScanner;
    List<String> trajectoryJSONList = new ArrayList<String>();

    try {
      fileScanner = new Scanner(f);
      while(fileScanner.hasNextLine()) {
        trajectoryJSONList.add(fileScanner.nextLine().split("\\.")[0]);  // get name of file not the (.path)
      }

      if(trajectoryJSONList != null) {
        for(String name : trajectoryJSONList) {
          String pathName = "paths/" + name + ".wpilib.json";
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathName); // goes to scr/main/deploy/paths
          Trajectory trajectory;
          try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            mTrajectories.add(trajectory);
          } catch (IOException e) {
            System.out.println("***Unable to create individual trajectories.");
          }
        }
      }  

      fileScanner.close();
    } catch (FileNotFoundException e) {
      System.out.println("***Trajectory List unable to be created.");
    }
    
  }
            
  // Use this to pass the autonomous command to the main {@link Robot} class.
  // @return the command to run in autonomous
  public Command getAutonomousCommand() {
    if (driveBaseSubsystem != null && trajectory != null) {
      //Ramsete Command for Pathweaver
      RamseteCommand ramseteCommand =
      new RamseteCommand(
        trajectory,
        driveBaseSubsystem::getPose,
        new RamseteController(Constants.ramseteB, Constants.ramseteZeta), // ramsete follower to follow trajectory
        Constants.driveKinematics,
        driveBaseSubsystem::acceptWheelSpeeds,
        driveBaseSubsystem);
          
      driveBaseSubsystem.resetOdometry(trajectory.getInitialPose());

      return ramseteCommand.andThen(() -> driveBaseSubsystem.acceptWheelSpeeds(0,0));
    }
    return null;
  }
  

  // TODO: create get methods for other subsystems to pass into TabContainer, or find a more efficient way
  public static DriveBaseSubsystem getDriveBase() {
    if (driveBaseSubsystem == null) {
      return null;
    }
    return driveBaseSubsystem;
  }
}
