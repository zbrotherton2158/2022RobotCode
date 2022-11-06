// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CDSConstants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.StopperSubsystem;

public class CDSForwardCommand extends CommandBase {
  /** Creates a new IntakeForwardCommand. */
  private CDSSubsystem mCDSSubsystem;
  private StopperSubsystem stopperSubsystem;

  public CDSForwardCommand(CDSSubsystem CDSSubsystem, StopperSubsystem stopperSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CDSSubsystem);
    addRequirements(stopperSubsystem);
    mCDSSubsystem = CDSSubsystem;
    this.stopperSubsystem = stopperSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mCDSSubsystem.CDSWheelToggle(false);
    mCDSSubsystem.CDSBeltToggle(false, CDSConstants.CDSBeltSpeed);
    stopperSubsystem.reverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mCDSSubsystem.stopCDS();
    stopperSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
