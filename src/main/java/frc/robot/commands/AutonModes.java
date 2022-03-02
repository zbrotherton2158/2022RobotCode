package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.io.IOException;
import java.nio.file.Path;

public class AutonModes {

  // subsystems
  private DriveBaseSubsystem driveBaseSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private CDSSubsystem cdsSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private boolean allSubsystemsEnabled;

  // Trajectories
  // private Trajectory[] taxiTrajectories;
  private Trajectory taxiTrajectory;
  private Trajectory oneBallTrajectory;
  private Trajectory twoBallTrajectory;

  private Trajectory[] threeBallTrajectories;
  private Trajectory[] fourBallTrajectories;

  // Ramsete Commands, commands for following paths from pathweaver
  private RamseteCommand taxiRamseteCommand;
  private RamseteCommand oneBallRamseteCommand;
  private RamseteCommand twoBallRamseteCommand;

  private RamseteCommand[] threeRamseteCommands;
  private RamseteCommand[] fourRamseteCommands;

  // command groups
  private Command taxiCommand;
  private Command oneBallCommand;
  private Command twoBallCommand;
  private Command twoBallParallel;
  private Command threeBallCommand;
  private Command fourBallCommand;

  // this constructor is the default, only needs driveBaseSubsystem, useful when only wanting to
  // test taxi without worrying about other subsystems
  public AutonModes(DriveBaseSubsystem d) {
    this.driveBaseSubsystem = d;

    allSubsystemsEnabled = false;

    // parameters are route names (names based on pathweaver, they are located under Auton file)
    initializeTrajectories();

    // create ramsete commands using the trajectories
    initializeRamseteCommands();

    // initialize the command groups
    initializeCommandGroups();
  }

  public AutonModes(
      DriveBaseSubsystem d,
      ShooterSubsystem s,
      LimelightSubsystem l,
      CDSSubsystem c,
      IntakeSubsystem i) {

    this.driveBaseSubsystem = d;
    this.shooterSubsystem = s;
    this.limelightSubsystem = l;
    this.cdsSubsystem = c;
    this.intakeSubsystem = i;

    allSubsystemsEnabled = true;

    // parameters are route names (names based on pathweaver, they are located under Auton file)
    initializeTrajectories();

    // create ramsete commands using the trajectories
    initializeRamseteCommands();

    // initialize the command groups
    initializeCommandGroups();
  }

  private Trajectory getTrajectory(String pathName) {
    Trajectory t = null;
    Path path =
        Filesystem.getDeployDirectory().toPath().resolve(pathName); // goes to scr/main/deploy/paths
    try {
      t = TrajectoryUtil.fromPathweaverJson(path);
      System.out.println("Success: " + pathName + " created.");
    } catch (IOException e) {
      System.out.println("Trajectory: " + pathName + " not created.");
    }
    return t;
  }

  private RamseteCommand getRamseteCommand(Trajectory trajectory) {
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            driveBaseSubsystem::getPose,
            new RamseteController(
                Constants.ramseteB, Constants.ramseteZeta), // ramsete follower to follow trajectory
            Constants.driveKinematics,
            driveBaseSubsystem::acceptWheelSpeeds,
            driveBaseSubsystem);

    return ramseteCommand;
  }

  private void initializeTrajectories() {
    taxiTrajectory = getTrajectory("paths/TaxiOut.wpilib.json");

    if (allSubsystemsEnabled) {
      oneBallTrajectory = getTrajectory("paths/TaxiOutFromFender.wpilib.json");

      twoBallTrajectory = getTrajectory("paths/TaxiOutToGrabBall.wpilib.json");

      // threeBallTrajectories
      // fourBallTrajectories
    }
  }

  private void initializeRamseteCommands() {
    taxiRamseteCommand = getRamseteCommand(taxiTrajectory);

    if (allSubsystemsEnabled) {
      oneBallRamseteCommand = getRamseteCommand(oneBallTrajectory);

      twoBallRamseteCommand = getRamseteCommand(twoBallTrajectory);

      // threeBallRamseteCommand
      // fourBallRamseteCommand
    }
  }

  private void initializeCommandGroups() {
    // TODO: the wait time should not be a constant, should be configurable

    taxiCommand =
        new SequentialCommandGroup(
            new WaitCommand(Constants.delaytaxi),
            // taxiRamseteCommands[0]);
            taxiRamseteCommand.beforeStarting(
                () -> driveBaseSubsystem.resetOdometry(taxiTrajectory.getInitialPose())));

    if (allSubsystemsEnabled) {
      oneBallCommand =
          new SequentialCommandGroup(
              new WaitCommand(Constants.delayshot),
              // new ShooterPrime(shooterSubsystem, limelightSubsystem, cdsSubsystem),
              new ShooterPressed(
                  shooterSubsystem,
                  limelightSubsystem,
                  cdsSubsystem,
                  true), // also has a time delay of 2-3 seconds
              new WaitCommand(Constants.delaytaxi),
              oneBallRamseteCommand.beforeStarting(
                  () -> driveBaseSubsystem.resetOdometry(oneBallTrajectory.getInitialPose())));

      twoBallParallel =
          new ParallelDeadlineGroup(
              twoBallRamseteCommand.beforeStarting(
                  () ->
                      driveBaseSubsystem.resetOdometry(
                          twoBallTrajectory.getInitialPose())), // go out to get ball
              new IntakeForwardCommand(intakeSubsystem));

      twoBallCommand =
          new SequentialCommandGroup(
              new WaitCommand(Constants.delaytaxi),
              twoBallParallel,
              new WaitCommand(Constants.delayshot),
              // new ShooterPrime(shooterSubsystem, limelightSubsystem, cdsSubsystem)
              new ShooterPressed(shooterSubsystem, limelightSubsystem, cdsSubsystem, true));

      threeBallCommand = null;
      fourBallCommand = null;
    }
  }

  public Command getChosenCommand(String commandName) {
    switch (commandName) {
      case "taxi":
        System.out.println("Taxi command chosen.");
        return taxiCommand;
      case "one ball":
        System.out.println("One ball command chosen.");
        return oneBallCommand;
      case "two ball":
        System.out.println("Two ball command chosen.");
        return twoBallCommand;
      case "three ball":
        System.out.println("Three ball command chosen.");
        return threeBallCommand;
      case "four ball":
        System.out.println("Four ball command chosen.");
        return fourBallCommand;
      default:
        System.out.println("No command chosen.");
        return null;
    }
  }
}