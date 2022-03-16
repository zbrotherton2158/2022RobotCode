// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CDSSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CDSBallManagementCommand extends CommandBase {
  /** Creates a new CDSBallManagementCommand. */
  private final CDSSubsystem CDSSubsystem;

  private final IntakeSubsystem intakeSubsystem;

  private boolean runningCDS = false;
  private boolean ejectRunning = false;
  private int sensorIndex;

  private int msCurrent = 0;
  private int runInterval = 40; // how often to call color sensors (in ms)
  private int msDelay = 750;

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
    // Only run if all sensors are online. If not, don't run ball management
    if (CDSSubsystem.sensorsOnline()) {

      if (msCurrent >= runInterval) {
        int lastBallCount = CDSSubsystem.getBallCount();
        boolean[] sensorStatus = CDSSubsystem.getSensorStatus();

        SmartDashboard.putBoolean("Front sensor status", sensorStatus[2]);
        SmartDashboard.putBoolean("Middle Sensor Status", sensorStatus[1]);
        SmartDashboard.putBoolean("Back Sensor Status", sensorStatus[0]);

        // Calls Auto Eject
        if (!ejectRunning) {
          // Checks if conditions for ejection are met:
          // A ball count of over 2 OR ball color is wrong and test mode is off (meaning ball color
          // shouldn't be disregarded)
          // TODO This is somehow ejecting any ball regardless of color

          // Uncomment Once Done

          if ((lastBallCount > 2)
              || (sensorStatus[2]
                  && CDSSubsystem.getAllianceColor() != CDSSubsystem.senseColor())) {
            // System.out.printf("Eject: %d %b %s %s\n", lastBallCount, sensorStatus[2],
            // CDSSubsystem.getAllianceColor(), CDSSubsystem.senseColor());
            intakeSubsystem.toggleIntake(true);
            CDSSubsystem.CDSWheelToggle(true);
            ejectRunning = true;
            SmartDashboard.putBoolean("Eject Running", ejectRunning);
          }
        } else {
          if (msCurrent >= msDelay) {
            CDSSubsystem.stopCDS();
            intakeSubsystem.stopIntake();
            ejectRunning = false;
            msCurrent = 0;
          } else {
            msCurrent += 20;
          }
        }

        // Only run auto advance if auto ject is not running
        if (!ejectRunning) {
          SmartDashboard.putBoolean("CDS Running", runningCDS);
          if (!runningCDS) {
            // Send ball to sensor
            SmartDashboard.putBooleanArray("Sensor Statuses", sensorStatus);
            if (sensorStatus[2]) {
              int nextOpenSensor = CDSSubsystem.getNextOpenSensor(sensorStatus);
              SmartDashboard.putNumber("Open Sensor Index", nextOpenSensor);
              if (nextOpenSensor != -1) {
                // There is an open sensor avaliable, run CDS
                runningCDS = true;
                sensorIndex = nextOpenSensor;
                CDSSubsystem.CDSBeltToggle(false);
                CDSSubsystem.setReady(false);
              }
            }
          } else {
            // Check if ball has reached sensor, stop if it has
            SmartDashboard.putNumber("Open Sensor Index", sensorIndex);
            if (sensorStatus[sensorIndex]) {
              CDSSubsystem.stopCDS();
              runningCDS = false;
              sensorIndex = -1;
              CDSSubsystem.setReady(true);
            }
          }
        }
      } else {
        msCurrent += 20;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CDSSubsystem.stopCDS();
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (CDSSubsystem.getSensorDown() >= Constants.sensorsDownLimit) {
      return true;
    } else {
      return false;
    }
  }
}
