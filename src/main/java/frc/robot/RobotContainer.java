// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.AutoConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //RR 1/11/2022
  public static final XboxController m_driverController = new XboxController(0);//change
  public static final PS4Controller m_controller = new PS4Controller(3);
  public static final Joystick m_joystick = new Joystick(2);

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Auto m_auto = new Auto();
  private final BallIntake m_BallIntake = new BallIntake();
  private final Climbing m_Climbing = new Climbing();
  private final Elevator m_Elevator = new Elevator();
  private final RobotDrive m_RobotDrive = new RobotDrive();
  private final Shooting m_Shooting = new Shooting();
  private final ShootingRotate m_ShootingRotate = new ShootingRotate();
  
  private int[] timing = new int[]{0, 6, 12};
  private final ExampleCommand m_exampleCommand = new ExampleCommand(m_exampleSubsystem);
  private final AutoCommand m_AutoCommand = new AutoCommand(m_RobotDrive, m_BallIntake, m_Shooting, m_ShootingRotate, m_Elevator, .5, timing);
  private final BallIntakeCommand m_BallIntakeCommand = new BallIntakeCommand(m_BallIntake);
  private final BallShootTopCommand m_BallShootTopCommand = new BallShootTopCommand(m_Shooting);
  private final ClimbingHangCommand m_ClimbingHangCommand = new ClimbingHangCommand(m_Climbing);
  private final ClimbingTraverseCommand m_ClimbingTraverseCommand = new ClimbingTraverseCommand(m_Climbing);
  private final DriveCommand m_DriveCommand = new DriveCommand(m_RobotDrive);
  private final ElevatorMoveBottomCommand m_ElevatorMoveBottomCommand = new ElevatorMoveBottomCommand(m_Elevator);
  private final ShootingRotateCommand m_ShootingRotateCommand = new ShootingRotateCommand(m_ShootingRotate);
  private final ElevatorMoveTopCommand m_ElevatorMoveTopCommand = new ElevatorMoveTopCommand(m_Elevator);
  private final IndexTwo m_IndexTwo = new IndexTwo(m_Elevator, m_BallIntake);
  private final MoveIndexThree m_moveIndexThreeCommand = new MoveIndexThree(m_Elevator);
  private final BallOutCommand m_BallOutCommand = new BallOutCommand(m_Elevator);
  private static boolean adjustRotateOn = true;

  public static double startingAngle = 0.0;
  public static Encoder leftEncoder = new Encoder(0,1);
  public static Encoder rightEncoder = new Encoder(2, 3);
  //public static Encoder limelightRotateEncoder = new Encoder(4, 0);

  private static AHRS m_gyro = new AHRS(SPI.Port.kMXP); //HC - 01/13/22
  private static PIDController turnController = new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD);

  // private static DigitalInput ballLimitSwitch = new DigitalInput(AutoConstants.ballLimitSwitchPort);
  private static DigitalInput hoodLimitSwitch = new DigitalInput(AutoConstants.hoodLimitSwitchPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_DriveCommand.execute();
    configureButtonBindings();
    (ShootingRotate.m_AngleRotateEncoder).setPosition(0);
    (RobotDrive.m_RLencoder).setPosition(0);
    (RobotDrive.m_RRencoder).setPosition(0);
    (RobotDrive.m_FLencoder).setPosition(0);
    (RobotDrive.m_FRencoder).setPosition(0);
    startingAngle = limelightTrackingX();
    m_gyro.calibrate();

  }

  public double additionalX(){
    double current_angle = startingAngle + m_gyro.getAngle();
    double xVelocity = Math.cos(current_angle)*m_RobotDrive.getSpeed();
    return xVelocity * .02; //change - idk how to do this
  }

  public double additionalY(){
    double current_angle = startingAngle + m_gyro.getAngle();
    double yVelocity = Math.sin(current_angle)*m_RobotDrive.getSpeed();
    return yVelocity * .02; //change - idk how to do this
  }


  // public static boolean getBallLimitSwitch(){
  //   return ballLimitSwitch.get();
  // }

  public static boolean getHoodLimitSwitch(){
    return hoodLimitSwitch.get();
  }

  // public static double getJoystickXAxis () {
  //   return m_controller.getRightX();
  // }
  // public static double getJoystickYAxis () {
  //   return m_controller.getRightY();
  // }
  public static double getLeftStickY(){
    return m_driverController.getRawAxis(0);
  }
  public static double getLeftStickX(){
    return m_driverController.getRawAxis(1);
  }
  public static double getJoystickXAxis(){
    return m_driverController.getRawAxis(2);
  }
  public static double  getJoystickYAxis(){
    return m_driverController.getRawAxis(3);
  }
  
  


// public static double getLeftStickY(){
//    return m_controller.getLeftY();
//  }

//  public static double getLeftStickX(){
//    return m_controller.getLeftX();
//  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  /* private void configureButtonBindings() {
  //   new JoystickButton(m_driverController, XboxController.Button.kY.value).whileHeld(m_ElevatorMoveBottomCommand);
  //   new JoystickButton(m_driverController, XboxController.Button.kX.value).whileHeld(m_BallShootTopCommand); //og : m_BallIntakeCommand
  //   new JoystickButton(m_driverController, XboxController.Button.kB.value).whileHeld(m_ElevatorMoveTopCommand);
  //   new JoystickButton(m_driverController, XboxController.Button.kStart.value).whileHeld(m_BallIntakeCommand);
  //   new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileHeld(m_ElevatorMoveBottomCommand);
  //   new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileHeld(m_IndexTwo);
   }*/

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kB.value).whileHeld(m_BallShootTopCommand); //og : m_BallIntakeCommand
    new JoystickButton(m_driverController, XboxController.Button.kY.value).whileHeld(m_ElevatorMoveTopCommand);
    new JoystickButton(m_driverController, XboxController.Button.kX.value).whileHeld(m_BallIntakeCommand);
    new JoystickButton(m_driverController, XboxController.Button.kA.value).whileHeld(m_ShootingRotateCommand);
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileHeld(m_moveIndexThreeCommand);
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileHeld(m_BallOutCommand);
  }


  //  private void configureButtonBindings() {
  //    //new JoystickButton(m_driverController, PS4Controller.Button.kTriangle.value).whileHeld(m_ElevatorMoveBottomCommand);
  //    new JoystickButton(m_driverController, PS4Controller.Button.kCircle.value).whileHeld(m_BallShootTopCommand); //og : m_BallIntakeCommand
  //    new JoystickButton(m_driverController, PS4Controller.Button.kR2.value).whileHeld(m_ElevatorMoveTopCommand);
  //    new JoystickButton(m_driverController, PS4Controller.Button.kL2.value).whileHeld(m_BallIntakeCommand);
  //    new JoystickButton(m_driverController, PS4Controller.Button.kL1.value).whileHeld(m_moveIndexThreeCommand);
  //    new JoystickButton(m_driverController, PS4Controller.Button.kR1.value).whileHeld(m_BallOutCommand);


  // }

  // public static XboxController getXboxController(){
  //   return m_driverController;
  // }

  public static XboxController getDriveJoystick(){
    return m_driverController;
    
  }

  /**
   * Called in the ShootingRotate subsystem to pass in the boolean adjustRotateOn. - HC
   */
  public static boolean getRotateStatus(){
    return adjustRotateOn;
  }
  
  /**
   * Called in the ShootingRotate subsystem under periodic() to check if the rotateOn boolean should be negated - HC
   * Change this button.
   */
  

  public static double limelightTrackingX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(1);
  }

  public static double limelightTrackingY() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public static double limelightTrackingA() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  public static boolean limelightTrackTarget() {
    if(limelightTrackingX() == 0.0){//change
      /*if(limelightTrackingY() == 0.0){//change
        return true;
      }*/
      
      return true;
    }
    return false;
  }

  public static double getShootSpeedValue(){//implement vision here later- RR 1/11/2022
    double velocity = limelightTrackingA() * 3.0 + 3.234324;//change this is a function
    return velocity; 
  }


  public static double getDistance() { 
    leftEncoder.reset();
    rightEncoder.reset();
    return (leftEncoder.getDistance() + rightEncoder.getDistance())/2;
  }

  /**
   * We need to test this and figure out anything logistical errors as well as see if this even works.
   * Takes in tx, ty, and ta from the limelight and outputs aim_adjust for the shooting rotator.
   * HC 02/05/2022
   * @return the constant to adjust the turret left or right
   */
  public static double limelightAdjustX() {
    /**
     * Once this works we can take in the current robot speed and factor that into our calculations same with limelightAdjustY()
     */
    double KpAim = -0.1;
    double min_aim_command = 0.05;
    double tx = RobotContainer.limelightTrackingX();
    double aim_adjust = 0;
    
    //TO-DO: Make sure to bind this to an XBox Controller button
    if (RobotContainer.limelightTrackTarget()) {
      double heading_error = -1 * tx;

    
      //AIM_ADJUST
      if (tx > 1.0) {
        aim_adjust = KpAim*heading_error - min_aim_command;
      } else if (tx < 1.0) {
      //otherwise the robot won’t adjust to these tiny increments due to friction
        aim_adjust = KpAim*heading_error + min_aim_command;
      }
    }
    return aim_adjust;
  }

  /**
   * We need to test this and figure out anything logistical errors as well as see if this even works.
   * Takes in ty, and ta from the limelight and outputs an angle_adjust for the shooting rotator.
   * HC 02/05/2022
   * @return the constant to adjust the angle the turret is aimed
   */


  public static double limelightAdjustY() {
    double KpTrajectory = -0.05; //not sure what to set this at
    double min_aimAngle_command = 0.05; //what should this be set at
    final double baseArea = 0.5; //change
    double angle_adjust = 0;
    
    double ty = RobotContainer.limelightTrackingY();
    double ta = RobotContainer.limelightTrackingA();
    
    //TO-DO: Make sure to bind this to an XBox Controller button
    if (RobotContainer.limelightTrackTarget()) {
      double trajectory_error = -1 * ty;    
      //ANGLE_ADJUST - accounts for both difference in target area and y adjustments
      if (ty > 1.0 ) {
        angle_adjust = KpTrajectory*trajectory_error - min_aimAngle_command;
      } else if (ty < 1.0) {
        angle_adjust = KpTrajectory*trajectory_error + min_aimAngle_command;
      }
      if (ta > baseArea ) {
        angle_adjust += 50 * (ta-baseArea)/baseArea; //arbitrary function 50 might be too much of an angle adjustment
      } else if (ta < baseArea) {
        angle_adjust -= 50 * (ta-baseArea)/baseArea;
      }
    }
    return angle_adjust;
  }

/** HC - 01/12/2022
 * Pseudocode from https://frc-pdr.readthedocs.io/en/latest/control/gyro.html
 * function rotateToAngle(targetAngle):
    error = targetAngle - gyroAngle # check out wpilib documentation for getting the angle from the gyro
    if error > threshold
        this.rotation =  error*kP
        return False
    else:
        this.rotation = 0
        return True
 * 
 */
  // public static boolean rotateToAngle(double targetAngle){
  //   //threshold is subject to change, but represents the accpetable margin of error
  //   double threshold = 5;
  //   double error = targetAngle - m_gyro.getAngle();
  //   if(error > threshold){
  //      /**
  //       * Add code to adjust motor so that the error is reduced
  //       */
      
  //      return false;
  //   } else {
  //     return true;
  //   }
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_AutoCommand;
  }


}
