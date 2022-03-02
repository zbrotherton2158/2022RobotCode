// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class CDSBallManagementCommand extends CommandBase {
  /** Creates a new CDSBallManagementCommand. */
  private final CDSSubsystem CDSSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private boolean runningCDS = false;
  private boolean ejectRunning = false;
  private int setpointIndex;

  private int cycleCount = 0;
  private int msDelay = 2000;

  public CDSBallManagementCommand(CDSSubsystem mCDSSubsystem, IntakeSubsystem mIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mCDSSubsystem);
    addRequirements(mIntakeSubsystem);

    CDSSubsystem = mCDSSubsystem;
    intakeSubsystem = mIntakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run eject before auto advance
    int ballCount = (int) SmartDashboard.getNumber("Ball Count", 0);
    boolean[] sensorStatus = CDSSubsystem.getSensorStatus();

    SmartDashboard.putBoolean("Front sensor status", sensorStatus[2]);
    SmartDashboard.putBoolean("Middle Sensor Status", sensorStatus[1]);
    SmartDashboard.putBoolean("Back Sensor Status", sensorStatus[0]);


    if (!ejectRunning) {
      if ((ballCount > 2) || (sensorStatus[2] && CDSSubsystem.getAllianceColor() != CDSSubsystem.senseColor() && !Constants.testMode)) {
        CDSSubsystem.CDSWheelToggle(true);
        intakeSubsystem.toggleIntake(true);
        ejectRunning = true;
      } 
    } else {
        if (cycleCount >= msDelay) {
          CDSSubsystem.stopCDS();
          intakeSubsystem.stopIntake();
          ejectRunning = false;
          cycleCount = 0;
        } else {
          cycleCount+=60;
        }
    }

    // Only run auto advance if auto eject is not running
    if (!ejectRunning) {

      if (!runningCDS) {
        // Send ball to setpoint
        if (sensorStatus[2]) {  
          int nextOpenSensor = CDSSubsystem.getNextOpenSensor(sensorStatus);
          SmartDashboard.putNumber("Setpoint", nextOpenSensor);
          if (nextOpenSensor != -1) {
            // There is an open setpoint avaliable, run CDS
            runningCDS = true;
            setpointIndex = nextOpenSensor;
            CDSSubsystem.CDSToggleAll(false);
            CDSSubsystem.setReady(false);
          }
        } 
      } else {
        // Check if ball has reached setpoint, stop if it has
        if (sensorStatus[setpointIndex]) {
          CDSSubsystem.stopCDS();
          runningCDS = false;
          setpointIndex = -1;
          CDSSubsystem.setReady(true);
        } 
      }    
    }   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
