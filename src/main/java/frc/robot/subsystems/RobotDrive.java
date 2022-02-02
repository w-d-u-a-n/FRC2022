// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
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



  public RobotDrive() {}

  public void arcadeDriveSimple(double leftStickPos, double rightStickPos, double maxSpeed){

    double drivePower = Math.pow(leftStickPos, 3);
    double leftDrive = drivePower + rightStickPos*.5;
    double rightDrive  = drivePower - rightStickPos*.5;

    leftDrive = leftDrive*maxSpeed;
    rightDrive = rightDrive*maxSpeed;

    m_left.set(leftDrive);
    m_right.set(-rightDrive);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.arcadeDriveSimple(RobotContainer.getLeftStick(), RobotContainer.getRightStickXAxis(), .3);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
