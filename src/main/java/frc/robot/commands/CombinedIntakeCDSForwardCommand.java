// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StopperSubsystem;

public class CombinedIntakeCDSForwardCommand extends CommandBase {
  /** Creates a new OuttakeCommand. */
  private CDSSubsystem CDSSubsystem;
  private StopperSubsystem stopperSubsystem;
  private IntakeSubsystem intakeSubsystem;

  public CombinedIntakeCDSForwardCommand(
      IntakeSubsystem mIntakeSubsystem,
      CDSSubsystem mCDSSubsystem,
      StopperSubsystem stopperSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeSubsystem);
    addRequirements(mCDSSubsystem);
    addRequirements(stopperSubsystem);
    this.stopperSubsystem = stopperSubsystem;
    intakeSubsystem = mIntakeSubsystem;
    CDSSubsystem = mCDSSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CDSSubsystem.CDSBeltToggle(false, Constants.CDSBeltSpeed);
    CDSSubsystem.CDSWheelToggle(false);
    intakeSubsystem.toggleIntake(false);
    stopperSubsystem.reverse();
    intakeSubsystem.deployIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CDSSubsystem.stopCDS();
    intakeSubsystem.stopIntake();
    stopperSubsystem.stop();
    intakeSubsystem.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
