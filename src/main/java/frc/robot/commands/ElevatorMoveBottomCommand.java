// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorMoveBottomCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_subsystem;

  /**
   * Creates a new ElevatorMoveBottomCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorMoveBottomCommand(Elevator subsystem) {
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
    m_subsystem.moveDown();//FZ
    System.out.println ("ElevatorMoveBottomCommand execute.");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop1();//FZ
    m_subsystem.stop3();
    System.out.println ("ElevatorMoveBottomCommand end.");

  }
    
    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
