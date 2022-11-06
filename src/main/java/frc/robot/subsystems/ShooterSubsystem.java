// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.common.hardware.MotorController;
import frc.robot.common.hardware.MotorController.MotorConfig;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotor;
  private CANSparkMax shooterTwoMotor;
  private SimpleMotorFeedforward shooterFF;
  private SparkMaxPIDController shooterPIDController;
  private RelativeEncoder shooterEncoder;
  private CANSparkMax stopperWheelMotor;

  private double targetRPM;
  private double currentRPM;
  private double smoothRPM;

  private ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator View");
  private NetworkTableEntry DTRPM = operatorTab.add("T-RPM", 0).withPosition(0, 2).getEntry();
  private NetworkTableEntry DRPM =
      operatorTab.add("RPM", 0).withWidget(BuiltInWidgets.kDial).withSize(2, 2).getEntry();
  private NetworkTableEntry BCargoRunning =
      operatorTab.add("Flywheel Ready", false).withPosition(2, 0).getEntry();
  private NetworkTableEntry SAimMode =
      operatorTab.add("Aim Mode", "TEST").withPosition(2, 1).getEntry();
  private NetworkTableEntry BOverride =
      operatorTab
          .add("Override", false)
          .withPosition(1, 2)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .getEntry();

  private ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter Tab");
  private NetworkTableEntry PID_P;
  private NetworkTableEntry PID_I;
  private NetworkTableEntry PID_D;
  private NetworkTableEntry DSmoothRPM;

  public ShooterSubsystem() {
    if (Constants.DebugMode) {
      instantiateDebugTab();
    }
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
    stopperWheelMotor = MotorController.constructMotor(MotorConfig.stopperWheel);

    // flywheelPID.setFF(Constants.Shooter.kF);

    DSmoothRPM = shooterTab.add("Smooth RPM", 0.0).getEntry();
  }

  private void instantiateDebugTab() {
    shooterTab = Shuffleboard.getTab("Shooter Tab");
    PID_P = shooterTab.add("PID P", ShooterConstants.kPIDArray[0]).withPosition(0, 1).getEntry();
    PID_I = shooterTab.add("PID I", ShooterConstants.kPIDArray[1]).withPosition(0, 2).getEntry();
    PID_D = shooterTab.add("PID D", ShooterConstants.kPIDArray[2]).withPosition(0, 3).getEntry();
  }

  public void updatePID() {
    if (Constants.DebugMode) {
      shooterPIDController.setP(PID_P.getDouble(0));
      shooterPIDController.setI(PID_I.getDouble(0));
      shooterPIDController.setD(PID_D.getDouble(0));
    }
  }

  public void windFlywheel(double rpm) {

    // Winds Flywheel using PID control to passed rpm
    if (rpm == 0) {
      shooterPIDController.setReference(0, CANSparkMax.ControlType.kVoltage);
    } else {
      DTRPM.setDouble(rpm);
      targetRPM = rpm;
      shooterPIDController.setReference(
          rpm,
          CANSparkMax.ControlType.kVelocity,
          ShooterConstants.kMaxISlot,
          shooterFF.calculate(rpm / 60.0));
    }
  }

  public void resetIAccum() {
    if (shooterEncoder.getVelocity() < 500) {
      shooterPIDController.setIAccum(0);
    }
  }

  public void setCargoBoolean(boolean a) {
    BCargoRunning.setBoolean(a);
  }

  public void runCargo(double speed) {
    stopperWheelMotor.set(speed);
  }

  public boolean wheelReady(int low, int high) {
    return (smoothRPM > targetRPM - low && smoothRPM < targetRPM + high);
  }

  public boolean wheelReady() {
    return (smoothRPM > targetRPM - 56 && smoothRPM < targetRPM + 56);
  }

  public double getTY() {
    // Returns TY, the vertical angle of the target from the limelight
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getDistance() {
    // Uses Limelight to find distance to High Goal
    return (ShooterConstants.highHeight - ShooterConstants.LLHeight)
        / Math.tan(Math.toRadians((getTY() + ShooterConstants.LLAngle))); // Return distance in ft
  }

  public void prime() {
    // Check what aimMode is active, winds flywheel
    if (BOverride.getBoolean(false)) {
      windFlywheel(DTRPM.getDouble(0));
      SAimMode.setString("OVERRIDE");
    } else {
      windFlywheel(ShooterConstants.fenderRPM);
      SAimMode.setString("NORMAL");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentRPM = shooterEncoder.getVelocity();
    smoothRPM = ShooterConstants.kA * currentRPM + smoothRPM * (1 - ShooterConstants.kA);

    DSmoothRPM.setDouble(smoothRPM);
    DRPM.setDouble(currentRPM);

    if (Constants.DebugMode) {
      if ((shooterPIDController.getP() != PID_P.getDouble(0))
          || (shooterPIDController.getI() != PID_I.getDouble(0))
          || (shooterPIDController.getD() != PID_D.getDouble(0))) {
        updatePID();
      }
    }
  }
}
