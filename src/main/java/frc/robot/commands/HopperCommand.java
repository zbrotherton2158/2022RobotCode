// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem; 

public class HopperCommand extends CommandBase {
  /** Creates a new IntakeForwardCommand. */
  private final HopperSubsystem m_hopperSubsystem;
  public IntakeForwardCommand(HopperSubsystem hopperSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem); 
    m_hopperSubsystem = hopperSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hopperSubsystem.HopperSwitch(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.HopperSwitch(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
© 2022
