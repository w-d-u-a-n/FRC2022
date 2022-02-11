// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

public class ShootingRotate extends SubsystemBase {
  /** Creates a new ShootingRotate. */
  private CANSparkMax m_Rotator = new CANSparkMax(AutoConstants.shootRotate, MotorType.kBrushless);
  private CANSparkMax m_angleRotator = new CANSparkMax(AutoConstants.shootAngleRotate, MotorType.kBrushless);

  public ShootingRotate() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(RobotContainer.limelightTrackingX());
    SmartDashboard.putNumber("LimelightX", RobotContainer.limelightTrackingX());
    adjustX();
  }
  //Methods written by RR 1/11/2022
  public void move(double aimAngle, double trajectoryAngle){
    m_Rotator.set(aimAngle);
    //m_angleRotator.set(trajectoryAngle);
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

  


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}
