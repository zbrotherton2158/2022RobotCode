// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.common.hardware.MotorController;
import frc.robot.common.hardware.MotorController.MotorConfig;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotor;
  private CANSparkMax shooterTwoMotor;
  private SimpleMotorFeedforward shooterFF;
  private SparkMaxPIDController shooterPIDController;
  private RelativeEncoder shooterEncoder;

  private double targetRPM;
  private double currentRPM;
  private double smoothRPM;

  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DTRPM = operatorTab.add("T-RPM", 0).withPosition(0, 2).getEntry();
  private NetworkTableEntry DRPM =
      operatorTab.add("RPM", 0).withWidget(BuiltInWidgets.kDial).withSize(2, 2).getEntry();
  private NetworkTableEntry SAimMode =
      operatorTab.add("Aim Mode", "TEST").withPosition(2, 1).getEntry();
  private NetworkTableEntry BOverride =
      operatorTab
          .add("Override", false)
          .withPosition(1, 2)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .getEntry();

  public ShooterSubsystem() {
    smoothRPM = 0;
    // Initializes the SparkMAX for the flywheel motors
    shooterMotor = MotorController.constructMotor(MotorConfig.shooterOne);
    shooterMotor = MotorController.constructMotor(MotorConfig.shooterTwo);
    shooterPIDController = MotorController.constructPIDController(shooterMotor, ShooterConstants.kPIDArray);
    shooterPIDController.setIMaxAccum(ShooterConstants.kMaxIAccum, ShooterConstants.kMaxISlot);
    shooterPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    shooterFF =
        new SimpleMotorFeedforward(
          ShooterConstants.kSg, ShooterConstants.kVg, ShooterConstants.kAg);
    shooterEncoder = shooterMotor.getEncoder();
    shooterMotor.enableVoltageCompensation(11);
    shooterTwoMotor.enableVoltageCompensation(11);
    shooterTwoMotor.follow(shooterMotor, true);
  }

  private double getTargetRPM() {
    // Check if override is active, return corresponding RPM
    if (BOverride.getBoolean(false)) {
      SAimMode.setString("OVERRIDE");
      return DTRPM.getDouble(0);
    } else {
      SAimMode.setString("NORMAL");
      return ShooterConstants.fenderRPM;
    }
  }

  public void windShooter() {
    // Wind shooter using PID control to target RPM
    targetRPM = getTargetRPM();
    DTRPM.setDouble(targetRPM);
    shooterPIDController.setReference(
        targetRPM,
        CANSparkMax.ControlType.kVelocity,
        ShooterConstants.kMaxISlot,
        shooterFF.calculate(targetRPM / 60.0));
  }
  
  public void stopShooter(){
    shooterPIDController.setReference(0, CANSparkMax.ControlType.kVoltage);
  }

  public void resetIAccum() {
    if (shooterEncoder.getVelocity() < 500) {
      shooterPIDController.setIAccum(0);
    }
  }

  public boolean wheelReady(int low, int high) {
    return (smoothRPM > targetRPM - low && smoothRPM < targetRPM + high);
  }

  public boolean wheelReady() {
    return (smoothRPM > targetRPM - 56 && smoothRPM < targetRPM + 56);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentRPM = shooterEncoder.getVelocity();
    smoothRPM = ShooterConstants.kA * currentRPM + smoothRPM * (1 - ShooterConstants.kA);
    DRPM.setDouble(currentRPM);
  }
}
