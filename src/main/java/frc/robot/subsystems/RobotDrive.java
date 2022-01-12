// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotDrive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private PWMVictorSPX m_rearLeft = new PWMVictorSPX(0); //change
  private PWMVictorSPX m_frontLeft = new PWMVictorSPX(1);
  private MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  private PWMVictorSPX m_rearRight = new PWMVictorSPX(2);
  private PWMVictorSPX m_frontRight = new PWMVictorSPX(3);
  private MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  public RobotDrive() {}

  public void arcadeDriveSimple(double y, double x){
    DifferentialDrive.arcadeDriveIK(y, x, false);
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
