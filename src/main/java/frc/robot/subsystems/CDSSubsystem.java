// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.hardware.MotorController;

/** Add your docs here. */
public class CDSSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private MotorController mCDSBeltController;
  private MotorController mCDSWheelControllerOne;
  private MotorController mCDSWheelControllerTwo;
  
  public CDSSubsystem() {
    mCDSBeltController = new MotorController("CDS Motor", Constants.kCDSMotorThreeID);
    mCDSWheelControllerOne =  new MotorController("Main CDS Wheel Controller", Constants.kCDSMotorFourID);
    mCDSWheelControllerTwo = new MotorController("Follows CDS Wheel Controller", Constants.kCDSMotorFiveID);
    
    mCDSWheelControllerTwo.getSparkMax().follow(mCDSWheelControllerOne.getSparkMax());
  }

  public void HopperSwitch(boolean on) {
    if (on) {
      double beltSmartSpeed = SmartDashboard.getNumber("Belt Speed", Constants.kCDSBeltSpeed);
      double wheelSmartSpeed = SmartDashboard.getNumber("Wheel Speed", Constants.kCDSWheelSpeed);

      mCDSBeltController.getSparkMax().set(beltSmartSpeed);
      mCDSWheelControllerOne.getSparkMax().set(wheelSmartSpeed);
      SmartDashboard.putNumber("CDS Belt Speed", Constants.kCDSBeltSpeed);
      SmartDashboard.putNumber("CDS Wheel Speed", Constants.kCDSWheelSpeed);
    } else {
      mCDSBeltController.getSparkMax().set(0.0);
      mCDSWheelControllerOne.getSparkMax().set(0.0);
      SmartDashboard.putNumber("CDS Motor Speed", 0);
    }
  }

  public void ForwardCDS() {
    mCDSBeltController.getSparkMax().setInverted(false);
    mCDSWheelControllerOne.getSparkMax().setInverted(false);
    SmartDashboard.putString("CDS Belt Direction", "Forward");
    SmartDashboard.putString("CDS Wheel Direction", "Forward");
  }

  public void ReverseCDS() {
    mCDSBeltController.getSparkMax().setInverted(true);
    mCDSWheelControllerOne.getSparkMax().setInverted(true);
    SmartDashboard.putString("CDS Belt Direction", "Reverse");
    SmartDashboard.putString("CDS Wheel Direction", "Reverse");
  }

}