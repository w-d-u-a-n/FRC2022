// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotDrive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private PWMVictorSPX m_rearLeft = new PWMVictorSPX(AutoConstants.rearLeftDrive); //change
  private PWMVictorSPX m_frontLeft = new PWMVictorSPX(AutoConstants.frontLeftDrive);
  private MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  private PWMVictorSPX m_rearRight = new PWMVictorSPX(AutoConstants.rearRightDrive);
  private PWMVictorSPX m_frontRight = new PWMVictorSPX(AutoConstants.frontRightDrive);
  private MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  public RobotDrive() {}

  /**
   * 
   * @param y 
   *      The robot's speed along the X axis [-1.0, 1.0]. Forward is positive.
   * @param x
   *      The robot's rotation rate around the Z axis [-1.0, 1.0]. Clockwise is positive.
   */
  public void arcadeDriveSimple(double y, double x){
    //third param = If true, decreases the input sensitivity at low speeds.
    DifferentialDrive.arcadeDriveIK(y, x, false);
  }

  public void takeJoystickInput(Joystick joystick){
    double sensitivity = ((joystick.getThrottle()*-1)/8) + .875;
    m_drive.arcadeDrive(joystick.getY()*sensitivity, joystick.getX()*sensitivity);
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
