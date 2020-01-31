/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  public static DriveBase driveBase;
  public static LimelightBase limelightBase;
  
  public static MoveForward moveForward;
  public static DriveWithJoystick driveWithJoystick;
  
  public static WPI_TalonSRX rightTalon = new WPI_TalonSRX(2);
  public static WPI_VictorSPX rightVictor = new WPI_VictorSPX(3);
  public static WPI_TalonSRX leftTalon = new WPI_TalonSRX(4);
  public static WPI_VictorSPX leftVictor = new WPI_VictorSPX(5);

  public static Joystick rightJoystick; 
  public static Joystick leftJoystick;
  public static Joystick logitech;

  
  




  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    
    rightJoystick = new Joystick(0);
    leftJoystick = new Joystick(1);
    logitech = new Joystick(2);

    driveBase = new DriveBase();
    limelightBase = new LimelightBase();
    moveForward = new MoveForward();
    driveWithJoystick = new DriveWithJoystick();
    CommandScheduler.getInstance().setDefaultCommand(driveBase, driveWithJoystick);

  
    // Configure the button bindings
    configureButtonBindings();
    


  }

  public static Joystick getRightJoystick(){
    return rightJoystick;
  }

  public static Joystick getLeftJoystick(){
    return leftJoystick;
  }

  public static Joystick getLogitech(){
    return logitech;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return moveForward;
    //just do moveForward for now, but eventually will be the chosen autonomous command 

  }
}