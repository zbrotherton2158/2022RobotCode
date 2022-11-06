// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CDSConstants;
import frc.robot.common.hardware.MotorController.MotorConfig;
import frc.robot.common.hardware.MotorController;

public class CDSSubsystem extends SubsystemBase {

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax CDSBeltMotor;
  private CANSparkMax singulatorOneMotor;
  private CANSparkMax singulatorTwoMotor;

  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DCDSSpeed =
      operatorTab
          .add("CDS Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(2, 1)
          .withPosition(3, 1)
          .getEntry();


  public CDSSubsystem() {
    CDSBeltMotor = MotorController.constructMotor(MotorConfig.CDSBelt);
    singulatorOneMotor = MotorController.constructMotor(MotorConfig.singulatorOne);
    singulatorTwoMotor = MotorController.constructMotor(MotorConfig.singulatorTwo);

    singulatorTwoMotor.follow(singulatorOneMotor, true);

    CDSBeltMotor.setIdleMode(IdleMode.kBrake);
    singulatorOneMotor.setIdleMode(IdleMode.kCoast);
  }

  public void CDSToggleAll(boolean reverse) {
    if (reverse) {
      singulatorOneMotor.set(-CDSConstants.CDSWheelControllerSpeed);
      DCDSSpeed.setDouble(-1);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Reverse");
        SmartDashboard.putNumber("CDS Wheel Speed 2", -CDSConstants.CDSWheelControllerSpeed);
      }

      CDSBeltMotor.set(-CDSConstants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 3", "Reverse");
        SmartDashboard.putNumber("CDS Belt Speed 2", -CDSConstants.CDSBeltSpeed);
      }
    } else {
      DCDSSpeed.setDouble(1);
      singulatorOneMotor.set(CDSConstants.CDSWheelControllerSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Forward");
        SmartDashboard.putNumber("CDS Wheel Speed 3", CDSConstants.CDSWheelControllerSpeed);
      }

      CDSBeltMotor.set(CDSConstants.CDSBeltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 4", "Forward");
        SmartDashboard.putNumber("CDS Belt Speed 3", CDSConstants.CDSBeltSpeed);
      }
    }
  }

  public void CDSWheelToggle(boolean reverse) {
    if (reverse) {
      singulatorOneMotor.set(-CDSConstants.CDSWheelControllerSpeed);
      SmartDashboard.putString("CDS Wheel Direction", "Reverse");
    } else {
      singulatorOneMotor.set(CDSConstants.CDSWheelControllerSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Wheel Direction", "Forward");
        SmartDashboard.putNumber("CDS Wheel Speed 4", CDSConstants.CDSWheelControllerSpeed);
      }
    }
  }

  public void CDSBeltToggle(boolean reverse, double beltSpeed) {
    DCDSSpeed.setDouble(-1);
    if (reverse) {
      CDSBeltMotor.set(-beltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 5", "Reverse");
        SmartDashboard.putNumber("CDS Belt Speed 4", -beltSpeed);
      }
    } else {
      DCDSSpeed.setDouble(1);
      CDSBeltMotor.set(beltSpeed);
      if (Constants.DebugMode) {
        SmartDashboard.putString("CDS Belt Direction 6", "Forward");
        SmartDashboard.putNumber("CDS Belt Speed 5", beltSpeed);
      }
    }
  }

  public double getBeltSpeed() {
    return CDSBeltMotor.get();
  }

  public double getWheelSpeed() {
    return singulatorOneMotor.get();
  }

  public void stopCDS() {
    DCDSSpeed.setDouble(0);
    // stops all motors in the CDS
    singulatorOneMotor.set(0.0);
    CDSBeltMotor.set(0.0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("CDS Wheel Speed 5", 0.0);
      SmartDashboard.putNumber("CDS Belt Speed 6", 0.0);
    }
  }

  public void stopCDSWheel() {
    // Stops only the centering wheels
    singulatorOneMotor.set(0.0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("CDS Wheel Speed 6", 0.0);
    }
  }

  public void stopCDSBelt() {
    // Stops only the belt
    CDSBeltMotor.set(0.0);
    if (Constants.DebugMode) {
      SmartDashboard.putNumber("CDS Belt Speed", 0.0);
    }
  }
}