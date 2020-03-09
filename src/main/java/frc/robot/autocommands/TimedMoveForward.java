/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

//@author MARIA CRISTOFORO
public class TimedMoveForward extends CommandBase {
  /**
   * Creates a new TimedMoveForward.
   */

  public WPI_TalonSRX rightTalon = RobotContainer.rightTalon;
  public WPI_VictorSPX rightVictor = RobotContainer.rightVictor;
  public WPI_TalonSRX leftTalon = RobotContainer.leftTalon;
  public WPI_VictorSPX leftVictor = RobotContainer.leftVictor;

  public WPI_TalonSRX intakeTalon = RobotContainer.intakeTalon;

   private double timeToRun;

   private boolean reachedHere = false;
  
   private Timer timer = RobotContainer.moverTimer;


  public TimedMoveForward(double t) {
    // Use addRequirements() here to declare subsystem dependencies.

    timeToRun = t;
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    timer.reset();
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   RobotContainer.driveBase.driveForwardAutonomous();
  //the method driveForwardAutonomous in the driveBase subsystem
  //just sets all the motor controllers to 0.8 - exactly like what is 
  //commented out below 

   
   //tried writing these here - didn't work either 
   //rightTalon.set(0.8);
   //leftTalon.set(0.8);
   //rightVictor.set(0.8);
   //leftVictor.set(0.8);

   //intakeTalon.set(0.8);
   //when we ran the command with the intakeTalon here instead 
   //of the drivetrain motor controllers, the command worked perfectly
    
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  RobotContainer.driveBase.stopAutonomous();
  //the method stopAutonomous in the driveBase subsystem
  //just sets all the motor controllers to zero - exactly like what is 
  //commented out below 

   
   //tried writing these here - didn't work either 
   //rightTalon.set(0.0);
   //leftTalon.set(0.0);
   //rightVictor.set(0.0);
   //leftVictor.set(0.0);

   //intakeTalon.set(0.0);
   //when we ran the command with the intakeTalon here instead 
   //of the drivetrain motor controllers, the command worked perfectly
    
   
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    reachedHere = true;
    SmartDashboard.putBoolean("Reached here", reachedHere);
    SmartDashboard.putBoolean("isFinished", (timer.get() >= timeToRun));
    return (timer.get() >= timeToRun);
  
    
    
  }
}
