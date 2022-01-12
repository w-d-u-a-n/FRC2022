// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbing extends SubsystemBase {
  /** Creates a new Climbing. */
  private PWMTalonSRX m_Climbing = new PWMTalonSRX(0); //set port
  
  public Climbing() {}

  public void climbUp(){
  m_Climbing.set (0.5);
  }
  public void climbDown(){
  m_Climbing.set (-0.5);
    }

  public void stop(){
    m_Climbing.set (0);
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
