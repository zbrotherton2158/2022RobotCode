// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController CDSBeltController;
  private MotorController CDSMotorController2;

  private DigitalInput initialBallSensor;
  private DigitalInput middleBallSensor;
  private DigitalInput finalBallSensor;

  
  private int ballCount = 0;
 
  //TODO: Figure out what CDS motor is for
  public CDSSubsystem() {
    CDSBeltController = new MotorController("CDS Motor", Constants.CDSBeltID);
    CDSMotorController2 = new MotorController("CDS Motor 2", Constants.CDSMotorID);

    CDSMotorController2.getSparkMax().follow(CDSBeltController.getSparkMax());
   
    initialBallSensor = new DigitalInput(Constants.initialBallSensorChannel);
    middleBallSensor = new DigitalInput(Constants.middleBallSensorChannel);
    finalBallSensor = new DigitalInput(Constants.finalBallSensorChannel); 

    DigitalInput[] sensorArray = {initialBallSensor, middleBallSensor, finalBallSensor};
  }
  
  public void stopCDS() {
    CDSBeltController.getSparkMax().set(0.0);
    SmartDashboard.putNumber("CDS Belt Speed", 0.0);
  }

  public void toggleCDS (boolean reverse) {
    if (reverse) {
      CDSBeltController.getSparkMax().set(-Constants.CDSBeltSpeed);
      SmartDashboard.putString("CDS Belt Direction", "Forward");
      SmartDashboard.putNumber("CDS Belt Speed", -Constants.CDSBeltSpeed);
    } else {
      CDSBeltController.getSparkMax().set(Constants.CDSBeltSpeed);
      SmartDashboard.putString("CDS Belt Direction", "Reverse");
      SmartDashboard.putNumber("CDS Belt Speed", Constants.CDSBeltSpeed);
    }
  }

  public boolean getDirection() {
    // true = inverted, false = forward
    if (CDSBeltController.getSparkMax().get() > 0) {
      return false;
    } else {
    return true;
    }
  }

  public boolean[] getBeamBreakStatus() {
    boolean initialStatus = initialBallSensor.get();
    boolean middleStatus = middleBallSensor.get();
    boolean finalStatus = finalBallSensor.get();
    boolean[] beamBreakArray = {initialStatus, middleStatus, finalStatus};
    return beamBreakArray;
  }

  public int getBallCount() {
    boolean[] beamBreakStatuses = this.getBeamBreakStatus();
    if (!beamBreakStatuses[0]) {
      if (this.getDirection()){
        ballCount--;
      } else {
        ballCount++;
      }
    }
    return ballCount;
  }

  public void expelBalls() {
    if (ballCount > 2) {
      this.toggleCDS(true);
      while (this.getBallCount() > 2) {
      }
      this.toggleCDS(false);
    }
  }

} //dont delete, for main method 