// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.hardware.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class ClimbSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private MotorController m_climbMotorControllerOne;
  private MotorController m_climbMotorControllerTwo;

  private DigitalInput m_limitSwitch;

  // TODO: maybe add servos

  public ClimbSubsystem() {
    //One is left, two is right
    m_climbMotorControllerOne = new MotorController("Climb Motor One", Constants.kClimbMotorOneIndex);
    m_climbMotorControllerTwo = new MotorController("Climb Motor Two", Constants.kClimbMotorTwoIndex);
    m_climbMotorControllerTwo.setInverted(true);

    m_limitSwitch = new DigitalInput(Constants.kLimitSwitchChannel);
  }

  public void toggleClimb(boolean on){    
    if (on){
      m_climbMotorControllerOne.getSparkMax().set(1);
      
    } else {
      m_climbMotorControllerOne.getSparkMax().set(0);
      
      // TODO: Add a better method of holding the robot
    }
  }

  public boolean getLimitSwitchVal() {
      return m_limitSwitch.get();
  }
  // TODO: might add other getter methods depending on how many limit switches

  
}
