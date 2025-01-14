// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Auton;
import frc.robot.commands.AutonModes;
import java.util.Map;

// The VM is configured to automatically run this class, and to call the functions corresponding to
// each mode, as described in the TimedRobot documentation. If you change the name of this class or
// the package after creating this project, you must also update the build.gradle file in the
// project.

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  // Shuffleboard for auton config
  private ShuffleboardTab configTab =
      Shuffleboard.getTab("Config"); // all auton settings located here
  private NetworkTableEntry waitTimeSlider =
      configTab
          .add("Wait Time", Constants.defaultInitialWaitTime)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("Min", 0, "Max", 10))
          .getEntry();
  private SendableChooser<Auton> chooser = new SendableChooser<>();

  private RobotContainer robotContainer;
  public UsbCamera usbCamera;

  // This function is run when the robot is first started up and should be used
  // for any
  // initialization code.

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // TODO: Put commands here

    if (isReal()) {
      usbCamera = CameraServer.startAutomaticCapture();
      usbCamera.setResolution(240, 320);
    }

    robotContainer = new RobotContainer();

    chooser.setDefaultOption(
        Auton.FOURBALL.getName(), Auton.FOURBALL); // default is four ball mode for now

    chooser.addOption(Auton.INTAKETAXI.getName(), Auton.INTAKETAXI);
    chooser.addOption(Auton.PUSHTAXI.getName(), Auton.PUSHTAXI);
    chooser.addOption(Auton.ONEBALL.getName(), Auton.ONEBALL);
    chooser.addOption(Auton.TWOBALL.getName(), Auton.TWOBALL);
    chooser.addOption(Auton.TWOBALLSTEAL1.getName(), Auton.TWOBALLSTEAL1);
    // chooser.addOption(Auton.TWOBALLSTEAL2.getName(), Auton.TWOBALLSTEAL2); // not ready
    chooser.addOption(Auton.THREEBALL.getName(), Auton.THREEBALL);
    chooser.addOption(Auton.FOURBALL.getName(), Auton.FOURBALL);
    // chooser.addOption(Auton.FIVEBALL.getName(), Auton.FIVEBALL);
    // chooser.addOption(Auton.TEST.getName(), Auton.TEST);      // don't need to show during
    // competition

    configTab.add("Auton mode", chooser).withPosition(0, 1).withSize(2, 2);
  }

  // This function is called every robot packet, no matter the mode. Use this for items like
  // diagnostics that you want ran during disabled, autonomous, teleoperated and test.

  // <p>This runs after the mode specific periodic functions, but before LiveWindow and
  // SmartDashboard integrated updating

  @Override
  public void robotPeriodic() {

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    robotContainer.pushSmartDashData();

    CommandScheduler.getInstance().run();
  }

  // This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // This would put the command that the auto mode is using on the smart dashboard, for debugging
    // SmartDashboard.putString("Auto Command", chooser.getSelected().getName());
  }

  // This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    AutonModes.setWaitTime(waitTimeSlider.getDouble(Constants.defaultInitialWaitTime));
    autonomousCommand = robotContainer.getAutonomousCommand(chooser.getSelected());

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  // This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  // This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  // This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
