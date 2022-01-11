// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallIntake extends SubsystemBase {
  /** Creates a new BallIntake. */
  private PWMTalonSRX m_Intake = new PWMTalonSRX(0);
  public BallIntake() {}

  public void ballTake(){//intake- RR
    m_Intake.set(.7);
  }

  public void ballTakeReverse(){//move wheels out- RR
    m_Intake.set(-.7);
  }
  
  public void stop(){//stop- RR
    m_Intake.set(0);
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
