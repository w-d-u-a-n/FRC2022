// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.Constants.AutoConstants;

public class Shooting extends SubsystemBase {
  /** Creates a new Shooting. */
  private PWMTalonFX m_ShooterTopRight = new PWMTalonFX(AutoConstants.shooter_right);
  private PWMTalonFX m_ShooterTopLeft = new PWMTalonFX(AutoConstants.shooter_left);
  private double error, derivative, adjust;
  private int integral, previousError = 0;
  private double setpoint = 0.0;

  public Shooting() {}
  //methods written by RR on 1/11/22
  public void shootTop(double strength) {
    m_ShooterTopRight.set(-strength);
    m_ShooterTopLeft.set(strength);
    /*double tS  = strength;
    double current = 0;
    while(current < tS){
      current += 0.005;
      m_ShooterTopRight.set(-current);
      m_ShooterTopLeft.set(current);
    }*/
    
    
  }

  /*public double PID(double q){
    setpoint = q;
    error = setpoint - m_ShooterTop.getSelectedSensorVelocity();
    integral += (error*.02);
    derivative = (error-this.previousError)/.02;
    adjust = AutoConstants.kP * error + AutoConstants.kI*integral + AutoConstants.kD*derivative;
    return adjust;
  }*/

  public void stop() {
    m_ShooterTopRight.set(0);
    m_ShooterTopLeft.set(0);


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
