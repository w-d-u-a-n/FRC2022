// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

public class ShootingRotate extends SubsystemBase {
  /** Creates a new ShootingRotate. */
  private PWMTalonSRX m_Rotator = new PWMTalonSRX(AutoConstants.shootRotate);//Change for port- RR 1/11/2022

  public ShootingRotate() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //Methods written by RR 1/11/2022
  public void move(double angle){
    m_Rotator.set(angle);
  }

  public void stop(){
    m_Rotator.set(0);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
