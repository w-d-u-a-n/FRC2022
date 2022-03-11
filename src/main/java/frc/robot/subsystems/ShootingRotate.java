// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

import frc.robot.subsystems.RobotDrive;

public class ShootingRotate extends SubsystemBase {
  /** Creates a new ShootingRotate. */
  private static CANSparkMax m_Rotator = new CANSparkMax(AutoConstants.shootRotate, MotorType.kBrushless);
  private static CANSparkMax m_angleRotator = new CANSparkMax(AutoConstants.shootAngleRotate, MotorType.kBrushless);

  public static RelativeEncoder m_AngleRotateEncoder = m_angleRotator.getEncoder();

  public double currentPos = 0.0;
  public double newPos = 0.0;

  public ShootingRotate() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*System.out.println(RobotContainer.limelightTrackingX());
    SmartDashboard.putNumber("LimelightX", RobotContainer.limelightTrackingX());
    RobotContainer.switchRotateStatus();*/
    move(RobotContainer.getJoystickXAxis(), RobotContainer.getJoystickYAxis());
    System.out.println("Angle Measurement: " + getZEncoder());
    
  }
  //Methods written by RR 1/11/2022
  public void move(double aimAngle, double trajectoryAngle){
   if(!(RobotContainer.getHoodLimitSwitch() /*&& trajectoryAngle < 0.5*/)){
      m_angleRotator.set(trajectoryAngle);
    } else if(trajectoryAngle < 0 ){
      m_angleRotator.set(trajectoryAngle);
    } else {
    ShootingRotate.m_AngleRotateEncoder.setPosition(0);
      }
    /*if(Math.abs(trajectoryAngle) < .1){
      m_angleRotator.set(0);
    }
    else{
      m_angleRotator.set(trajectoryAngle);
    }*/
    m_Rotator.set(aimAngle);
    
    /*if(Math.abs(aimAngle) < .1){
      m_Rotator.set(0);
    }
    else{
      m_Rotator.set(aimAngle*1.1);
    }*/
    
    
    
  }

  public double getZEncoder(){
    return m_AngleRotateEncoder.getPosition();
  }

  public void adjustZ(){
    currentPos = m_AngleRotateEncoder.getPosition();
    newPos = RobotContainer.limelightAdjustY();
    while(Math.abs(newPos - currentPos) > 1){
      if(newPos > currentPos){
        m_angleRotator.set(.4);
      }
      if(newPos < currentPos){
        m_angleRotator.set(.4);
      }
    }
    currentPos = newPos;

    if(RobotContainer.getHoodLimitSwitch()){
      (ShootingRotate.m_AngleRotateEncoder).setPosition(0);
      }
  }

  public void stop(){
    m_Rotator.set(0);

  }

  public void adjustX(){
    while(Math.abs(RobotContainer.limelightTrackingX()) > 3){
        if(RobotContainer.limelightTrackingX() > 0 ){
          m_Rotator.set(.3);
        }
        if(RobotContainer.limelightTrackingX() < 0){
          m_Rotator.set(-.3);

      }
    }
    m_Rotator.set(0);
  }

  /**
   *  method to adjust the hood to a given angle, assuming the relativee ecnoder is 0 at the limit switch.
   * @param angle - angle we want to adjust to
   */
  public void adjustHood(){
    double angle = RobotContainer.calcHoodAngle();
    final double speed = 0.3;
    while (Math.abs(ShootingRotate.m_AngleRotateEncoder.getPosition()-angle) > 10){
      //I'm too lazy to write a PID loop so let's hope that by the time the motor stops 10 degrees before, when it stops
      // spinning it will reach 
      if(ShootingRotate.m_AngleRotateEncoder.getPosition() > angle){
        m_angleRotator.set(-1*speed);
      } else{
        m_angleRotator.set(speed);
      }
    }
    m_angleRotator.set(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}
