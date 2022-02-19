// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class RobotDrive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_rearLeft = new CANSparkMax(AutoConstants.rearLeftDrive, MotorType.kBrushless);
  private CANSparkMax m_frontLeft = new CANSparkMax(AutoConstants.frontLeftDrive, MotorType.kBrushless);
  private MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  private CANSparkMax m_rearRight = new CANSparkMax(AutoConstants.rearRightDrive, MotorType.kBrushless);
  private CANSparkMax m_frontRight = new CANSparkMax(AutoConstants.frontRightDrive, MotorType.kBrushless);
  private MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

  private RelativeEncoder m_RLencoder = m_rearLeft.getEncoder();
  private RelativeEncoder m_FLencoder = m_frontLeft.getEncoder();
  private RelativeEncoder m_RRencoder = m_rearRight.getEncoder();
  private RelativeEncoder m_FRencoder = m_frontRight.getEncoder();

  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private double error, derivative, adjust;
  private int integral, previousError, setpoint = 0;


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

  public double PID(){
    error = setpoint - gyro.getAngle();
    integral += (error*.02);
    derivative = (error-this.previousError)/.02;
    adjust = AutoConstants.kP * error + AutoConstants.kI*integral + AutoConstants.kD*derivative;
    return adjust;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.arcadeDriveSimple(RobotContainer.getLeftStickX(), RobotContainer.getLeftStickY()-PID(), .3);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
