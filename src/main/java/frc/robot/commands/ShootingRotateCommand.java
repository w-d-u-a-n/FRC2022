// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShootingRotate;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

/** An example command that uses an example subsystem. */
public class ShootingRotateCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShootingRotate m_subsystem;
  public static boolean rotateStatus = false;
  public static double encoderAngle = 0;

  /**
   * Creates a new ShootingRotateCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootingRotateCommand(ShootingRotate subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotateStatus = true;
    m_subsystem.adjustX();
    /*
    if(RobotContainer.limelightTrackTarget() == false){
      m_subsystem.move(0, 0);
      System.out.println ("ShootingRotateCommand execute - target = false.");
    } else {
      SmartDashboard.putNumber("Turret Rotate X", RobotContainer.limelightAdjustX());
      SmartDashboard.putNumber("Turret Rotate Y", RobotContainer.limelightAdjustY());
      m_subsystem.move(RobotContainer.limelightAdjustX(), RobotContainer.limelightAdjustY());
      System.out.println ("ShootingRotateCommand execute - target = true.");
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
    rotateStatus = false;
    if(RobotContainer.getHoodLimitSwitch()){
    (ShootingRotate.m_AngleRotateEncoder).setPosition(0);
    }
    System.out.println ("ShootingRotateCommand end.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
