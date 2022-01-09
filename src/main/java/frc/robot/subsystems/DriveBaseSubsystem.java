// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.common.hardware.MotorController;


public class DriveBaseSubsystem extends SubsystemBase {

  private final Joystick m_driverJoystick;
  private final MotorController[] m_motorControllers = new MotorController[6];
  private final DifferentialDrive m_differentialDrive;
  
  // Creates a new ExampleSubsystem.
  public DriveBaseSubsystem(Joystick joystick) {  
    m_driverJoystick = joystick;
    
    m_differentialDrive = new DifferentialDrive(m_motorControllers[Constants.kDriveLeftFrontIndex].getSparkMax(), mMotorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    m_motorControllers[Constants.kDriveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.kDriveLeftFront);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
