// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private PWMTalonSRX m_Elevator = new PWMTalonSRX(0);//change 0
  public Elevator() {}
  //RR- methods 1/11/22
  public void moveUp(){
    m_Elevator.set(.5);
  }

  public void moveDown(){
    m_Elevator.set(-.5);
  }

  public void stop(){
    m_Elevator.set(0);
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
