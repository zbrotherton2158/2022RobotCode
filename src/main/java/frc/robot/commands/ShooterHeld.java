// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StopperSubsystem;

public class ShooterHeld extends CommandBase {
  private ShooterSubsystem m_ShooterSubsystem;
  private CDSSubsystem m_CDSSubsystem;
  private StopperSubsystem stopperSubsystem;
  private int i;

  /** Creates a new ShooterHeld. */
  public ShooterHeld(
      ShooterSubsystem shooterSubsystem,
      CDSSubsystem CDSSubsystem,
      StopperSubsystem stopperSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CDSSubsystem);
    addRequirements(shooterSubsystem);
    addRequirements(stopperSubsystem);
    m_ShooterSubsystem = shooterSubsystem;
    m_CDSSubsystem = CDSSubsystem;
    this.stopperSubsystem = stopperSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    m_ShooterSubsystem.resetIAccum();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.windShooter();
    if (m_ShooterSubsystem.wheelReady()) {
      // If below will bypass the LL check if the stopper is already running, or the LL is disabled.
      // Otherwise, alignment is checked.
      if (i > 0) {
        // if (i > 1) {
        m_CDSSubsystem.runBelt(false);
        stopperSubsystem.forward();
        // }
        // i++;
        /*if (i >= 50) { // 1000 miliseconds delay TODO: Use a CDS method for this when possible
          i = 0;
          m_CDSSubsystem.stopCDS();
          m_ShooterSubsystem.runCargo(0);
          m_ShooterSubsystem.setCargoBoolean(false);
        }*/
      }
    } else {
      /* m_ShooterSubsystem.setCargoBoolean(false);
      m_CDSSubsystem.stopCDS();
      m_ShooterSubsystem.runCargo(Constants.Shooter.cargoReverse);
      i = 0;*/
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stopperSubsystem.stop();
    m_ShooterSubsystem.stopShooter();
    m_CDSSubsystem.stopCDS();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
