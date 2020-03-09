/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


import frc.robot.RobotContainer;
import frc.robot.Robot;
import frc.robot.commands.*;

//@author MARIA CRISTOFORO, ELLA WARNOCK
public class DriveBase extends SubsystemBase {

  public WPI_TalonSRX rightTalon = RobotContainer.rightTalon;
  public WPI_VictorSPX rightVictor = RobotContainer.rightVictor;
  public WPI_TalonSRX leftTalon = RobotContainer.leftTalon;
  public WPI_VictorSPX leftVictor = RobotContainer.leftVictor;

  public JoystickSensitivity joystickSensitivity = new JoystickSensitivity();

  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

  
  double leftMotorSpeed = 0; //create variables to store joystick information 
  double rightMotorSpeed = 0;

  public static double scale = 1.0;

  
  /**
   * Creates a new DriveBase.
   */
  public DriveBase() {
   
  }

  public double EstimateDistance(){
    double distance = 0; //set to zero initially
    double robotHeight = 2.25;
    double openingHeight = 7.5;
    double mountingAngle = 30;
    double shootingAngle = -ty;
    //these values need to be measured and changed

    double totalDegrees = mountingAngle + shootingAngle;

    double totalRadians = Math.toRadians(totalDegrees);

    double tanValue = Math.tan(totalRadians);

    distance = (openingHeight - robotHeight) / tanValue;

    SmartDashboard.putNumber("EstimatedDistance", distance);

    return distance;

  }

  public void JoystickInputs(Joystick rightJoystick, Joystick leftJoystick, Joystick logitech){

      leftMotorSpeed = leftJoystick.getY() * -1; //get values from joysticks 
      rightMotorSpeed = rightJoystick.getY();

      leftMotorSpeed = joystickSensitivity.GetOutput(leftMotorSpeed);
      rightMotorSpeed = joystickSensitivity.GetOutput(rightMotorSpeed);
      
      SmartDashboard.putNumber("leftMotorSpeed", leftMotorSpeed);
      SmartDashboard.putNumber("rightMotorSpeed", rightMotorSpeed);

      
      leftTalon.set(leftMotorSpeed * scale); // finally assign the stored values to talons  
      leftVictor.set(leftMotorSpeed * scale);
      rightTalon.set(rightMotorSpeed * scale);
      rightVictor.set(rightMotorSpeed * scale);

     
    
  }


  public void driveForwardAutonomous(){
    leftTalon.set(ControlMode.PercentOutput, -0.5);
    rightTalon.set(ControlMode.PercentOutput, 0.5);
    leftVictor.set(ControlMode.PercentOutput, -0.5);
    rightVictor.set(ControlMode.PercentOutput, 0.5);
    SmartDashboard.putNumber("leftTalonCurrent", leftTalon.getSupplyCurrent());
    SmartDashboard.putNumber("rightTalonCurrent", rightTalon.getSupplyCurrent());
  

  }

  public void stopAutonomous(){
    leftTalon.set(ControlMode.PercentOutput, 0.0);
    leftTalon.set(ControlMode.PercentOutput, 0);
    leftVictor.set(ControlMode.PercentOutput, 0);
    rightTalon.set(ControlMode.PercentOutput, 0);
    rightVictor.set(ControlMode.PercentOutput, 0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

   
  }
}
