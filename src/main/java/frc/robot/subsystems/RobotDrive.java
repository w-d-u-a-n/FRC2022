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
  private static CANSparkMax m_rearLeft = new CANSparkMax(AutoConstants.rearLeftDrive, MotorType.kBrushless);
  private static CANSparkMax m_frontLeft = new CANSparkMax(AutoConstants.frontLeftDrive, MotorType.kBrushless);
  private MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  private static CANSparkMax m_rearRight = new CANSparkMax(AutoConstants.rearRightDrive, MotorType.kBrushless);
  private static CANSparkMax m_frontRight = new CANSparkMax(AutoConstants.frontRightDrive, MotorType.kBrushless);
  private MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

  public static RelativeEncoder m_RLencoder = m_rearLeft.getEncoder();
  public static RelativeEncoder m_FLencoder = m_frontLeft.getEncoder();
  public static RelativeEncoder m_RRencoder = m_rearRight.getEncoder();
  public static RelativeEncoder m_FRencoder = m_frontRight.getEncoder();

  private static AHRS gyro = new AHRS(SPI.Port.kMXP);
  private static double error, derivative, adjust;
  private static int integral, previousError, setpoint = 0;


  public RobotDrive() {}

  public void arcadeDriveSimple(double leftStickPos, double rightStickPos, double maxSpeed){

    double drivePower = -1*Math.pow(leftStickPos, 3);
    double leftDrive = drivePower + rightStickPos*0.5; //*0.5
    double rightDrive  = drivePower - rightStickPos*0.5; //*0.5


    leftDrive = leftDrive*maxSpeed;
    rightDrive = rightDrive*maxSpeed;

    m_left.set(leftDrive);
    m_right.set(-rightDrive);

  }

  public static double gyroAngle(){
    return gyro.getAngle();
  }


  public static double PID(){
    error = RobotContainer.getLeftStickX() - gyro.getAngle();
    integral += (error*.02);
    derivative = (error-previousError)/.02;
    adjust = AutoConstants.kP * error + AutoConstants.kI*integral + AutoConstants.kD*derivative;
    return adjust;
  }

  public static double getDistanceStraight(){
    return (m_FLencoder.getPosition() + m_FRencoder.getPosition()+m_RLencoder.getPosition()+m_RRencoder.getPosition())/4;
  }

  public static double getTurnRight(){
    return (m_FRencoder.getPosition()+m_RRencoder.getPosition())/2;
  }

  public static double getTurnLeft(){
    return (m_FLencoder.getPosition()+m_RLencoder.getPosition())/2;
  }

  public static double getSpeed(){
    return (m_FLencoder.getVelocity() + m_FRencoder.getVelocity()+m_RLencoder.getVelocity()+m_RRencoder.getVelocity())/4;
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
