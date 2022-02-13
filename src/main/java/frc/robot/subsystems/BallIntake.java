// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

public class BallIntake extends SubsystemBase {
  /** Creates a new BallIntake. */
  private PWMSparkMax m_Intake = new PWMSparkMax(AutoConstants.intakePort);
  private PWMSparkMax m_FixIntake = new PWMSparkMax(11); //what is this constant - HC 02/05/22

  public BallIntake() {}

  public void ballTake(){//intake- RR
    m_Intake.set(.34);
  }

  public void ballTakeReverse(){//move wheels out- RR
    m_Intake.set(-.7);
  }
  
  public void stop(){//stop- RR
    m_Intake.set(0);
    m_FixIntake.set(0);
  }

  public void pushIntake(){
    m_FixIntake.set(.1);
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
