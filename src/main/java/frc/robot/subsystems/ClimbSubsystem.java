// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;

/** Add your docs here. */
public class ClimbSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private NetworkTableEntry sbclimbHeightOne;
  private NetworkTableEntry sbclimbHeightTwo;
  private NetworkTableEntry sbClimbEnabbled;
  private Joystick m_climbJoystick;
  private MotorController m_climbMotorControllerOne;
  private MotorController m_climbMotorControllerTwo;
  private DigitalInput m_limitSwitch;
  private boolean climbEnabbled;
  private double climbHeightOne;
  private double climbHeightTwo;
  private double joystickAxis;
  private ShuffleboardTab climberTab;
  private NetworkTableEntry sbclimberpositionOne;
  private NetworkTableEntry sbclimberspeedOne;
  private NetworkTableEntry sbclimberheightOne;
  private NetworkTableEntry sbclimberpositionTwo;
  private NetworkTableEntry sbclimberspeedTwo;
  private NetworkTableEntry sbclimberheightTwo;
  private NetworkTableEntry sbclimbHeightImputOne;
  private NetworkTableEntry sbclimbHeightImputTwo;
  private NetworkTableEntry sbClimberSpeedInput;

  public ClimbSubsystem(Joystick joystick) {
    m_climbJoystick = joystick;
    climbEnabbled = false;
    climbHeightOne = 0;
    climbHeightTwo = 0;

    // One is left, two is right
    m_climbMotorControllerOne =
        new MotorController("Climb Motor One", Constants.ClimbMotorOne, 60, true);
    m_climbMotorControllerTwo =
        new MotorController("Climb Motor Two", Constants.ClimbMotorTwo, 60, true);
    m_climbMotorControllerTwo.setInverted(true);
    // m_climbMotorControllerTwo.getPID().setOutputRange(-.4, .4);
    // m_climbMotorControllerOne.getPID().setOutputRange(-.4, .4);
    m_climbMotorControllerOne.getEncoder().setPosition(0);
    m_climbMotorControllerTwo.getEncoder().setPosition(0);

    m_climbMotorControllerOne.setIdleMode(IdleMode.kBrake);
    m_climbMotorControllerTwo.setIdleMode(IdleMode.kBrake);

    // m_limitSwitch = new DigitalInput(Constants.LimitSwitchChannel);

    climberTab = Shuffleboard.getTab("ClimbBase");

    sbClimberSpeedInput =
        climberTab.add("Climber Speed input", 0).withSize(2, 2).withPosition(5, 0).getEntry();

    sbClimbEnabbled =
        climberTab.add("Climb Eanbled", false).withSize(2, 2).withPosition(0, 0).getEntry();
    sbclimberpositionOne =
        climberTab.add("Climber position", 0).withSize(2, 2).withPosition(2, 0).getEntry();
    sbclimberspeedOne =
        climberTab.add("Climber Current Speed 1", 0).withSize(2, 2).withPosition(4, 0).getEntry();
    sbclimberheightOne =
        climberTab
            .add("Climber targetted height 1", 0)
            .withSize(2, 2)
            .withPosition(6, 0)
            .getEntry();
    sbclimbHeightOne =
        climberTab.add("Climb Hight 1", 0).withSize(2, 2).withPosition(8, 0).getEntry();
    sbclimbHeightTwo =
        climberTab.add("Climb Hight 2", 0).withSize(2, 2).withPosition(8, 0).getEntry();

    sbclimberheightTwo =
        climberTab
            .add("Climber targetted height 2", 0)
            .withSize(2, 2)
            .withPosition(8, 0)
            .getEntry();
    sbclimberspeedTwo =
        climberTab.add("Climber Current Speed 2", 0).withSize(2, 2).withPosition(0, 2).getEntry();

    sbclimbHeightImputOne =
        climberTab.add("Climber Imput One", 0).withSize(2, 2).withPosition(2, 2).getEntry();
    sbclimbHeightImputTwo =
        climberTab.add("Climber Imput Two", 0).withSize(2, 2).withPosition(4, 2).getEntry();
    sbclimberpositionTwo =
        climberTab.add("Climber position 2", 0).withSize(2, 2).withPosition(2, 4).getEntry();

    m_climbMotorControllerOne.getPID().setP(Constants.climbRightPID[0]);
    m_climbMotorControllerOne.getPID().setI(Constants.climbRightPID[1]);
    m_climbMotorControllerOne.getPID().setD(Constants.climbRightPID[2]);

    m_climbMotorControllerTwo.getPID().setP(Constants.climbLeftPID[0]);
    m_climbMotorControllerTwo.getPID().setI(Constants.climbLeftPID[1]);
    m_climbMotorControllerTwo.getPID().setD(Constants.climbLeftPID[2]);
  }

  public void climbEnabbledEnable() {
    climbEnabbled = !climbEnabbled;
    sbClimbEnabbled.setBoolean(climbEnabbled);
  }

  public void RunManual() {
    if (climbEnabbled
    /** && !m_limitSwitch.get() */
    ) {

      joystickAxis = -m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (joystickAxis > 0.1 || joystickAxis < -0.1) {
        if (joystickAxis > 0) {
          if (climbHeightOne <= 20.0) {
            m_climbMotorControllerOne.getSparkMax().set(sbClimberSpeedInput.getDouble(0));
            ;
          }
          if (climbHeightTwo <= 20.0) {
            m_climbMotorControllerTwo.getSparkMax().set(sbClimberSpeedInput.getDouble(0));
            ;
          }
        }
        if (joystickAxis < 0) {
          if (climbHeightOne >= 0) {
            m_climbMotorControllerOne.getSparkMax().set(-sbClimberSpeedInput.getDouble(0));
            ;
          }
          if (climbHeightTwo >= 0) {
            m_climbMotorControllerTwo.getSparkMax().set(-sbClimberSpeedInput.getDouble(0));
            ;
          }
        }
      } else {
        m_climbMotorControllerOne.getSparkMax().set(0);
        m_climbMotorControllerTwo.getSparkMax().set(0);
      }
    }
  }

  public void enableClimb() {
    if (climbEnabbled
    /** && !m_limitSwitch.get() */
    ) {

      joystickAxis = -m_climbJoystick.getRawAxis(Constants.leftJoystickY);
      if (joystickAxis > 0.1 || joystickAxis < -0.1) {
        if (joystickAxis > 0) {
          if (climbHeightOne <= 20.0) {
            climbHeightOne = climbHeightOne + (joystickAxis / 10);
          }
          if (climbHeightTwo <= 20.0) {
            climbHeightTwo = climbHeightTwo + (joystickAxis / 10);
          }
        }
        if (joystickAxis < 0) {
          if (climbHeightOne >= 0) {
            climbHeightOne = climbHeightOne + (joystickAxis / 10);
          }
          if (climbHeightTwo >= 0) {
            climbHeightTwo = climbHeightTwo + (joystickAxis / 10);
          }
        }
      }
      m_climbMotorControllerOne
          .getPID()
          .setReference(climbHeightOne, CANSparkMax.ControlType.kPosition);
      sbclimbHeightOne.setNumber(climbHeightOne);

      m_climbMotorControllerTwo
          .getPID()
          .setReference(climbHeightTwo, CANSparkMax.ControlType.kPosition);
      sbclimbHeightTwo.setNumber(climbHeightTwo);
    } else {
      // m_climbMotorControllerOne.getPID().setReference(0, CANSparkMax.ControlType.kVoltage);
    }
  }

  public void periodic() {

    SmartDashboard.putNumber(
        "Climb motor 1 Applied Output", m_climbMotorControllerOne.getSparkMax().getAppliedOutput());
    SmartDashboard.putNumber(
        "Climb motor 2 Applied Output", m_climbMotorControllerTwo.getSparkMax().getAppliedOutput());
    SmartDashboard.putNumber(
        "Climb Hight One", m_climbMotorControllerOne.getEncoder().getPosition());
    if (sbclimbHeightOne.getDouble(0) != climbHeightOne) {
      climbHeightOne = sbclimbHeightOne.getDouble(0);
    } else {
      sbclimberheightOne.setDouble(climbHeightOne);
    }
    sbclimberspeedOne.setDouble(m_climbMotorControllerOne.getEncoder().getVelocity());
    sbclimberpositionOne.setDouble(m_climbMotorControllerOne.getEncoder().getPosition());
    m_climbMotorControllerOne.updateSmartDashboard();
    SmartDashboard.putNumber("Climb IAccum One", m_climbMotorControllerOne.getPID().getIAccum());

    SmartDashboard.putNumber(
        "Climb Hight Two", m_climbMotorControllerTwo.getEncoder().getPosition());
    if (sbclimbHeightTwo.getDouble(0) != climbHeightTwo) {
      climbHeightTwo = sbclimbHeightTwo.getDouble(0);
    } else {
      sbclimberheightTwo.setDouble(climbHeightTwo);
    }
    sbclimberspeedTwo.setDouble(m_climbMotorControllerTwo.getEncoder().getVelocity());
    sbclimberpositionTwo.setDouble(m_climbMotorControllerTwo.getEncoder().getPosition());
    m_climbMotorControllerTwo.updateSmartDashboard();
    SmartDashboard.putNumber("Climb IAccum Two", m_climbMotorControllerTwo.getPID().getIAccum());
  }

  public boolean getLimitSwitchVal() {
    return m_limitSwitch.get();
  }
  // TODO: might add other getter methods depending on how many limit switches
}
