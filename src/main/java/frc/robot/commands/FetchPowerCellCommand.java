/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.models.VisionObject;


public class FetchPowerCellCommand extends Command {

  PIDController angleController;

  public double totalRotation = 0;
  public FetchPowerCellCommand() {
    requires(Robot.drivetrainSubsystem);
    //PidConstants PID_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
    angleController = new PIDController(0.01, 0.015, 0);
  }

  public FetchPowerCellCommand(double timeout) {
    super(timeout);
    requires(Robot.drivetrainSubsystem);
    //PidConstants PID_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
    angleController = new PIDController(0.01, 0.015, 0);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.objectTrackerSubsystem.data();
    double forward = 0;
    double strafe = 0;
    double rotation = 0;

    VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject("powerCell");
    if (closestObject == null) 
      return; // no object found
    double angle =  Math.atan2(closestObject.x, closestObject.z);
    
    angleController.setSetpoint(angle);

    rotation = angleController.calculate(0);

    if(rotation > 1){
      rotation = 1;
    }else if(rotation < -1){
      rotation = -1;
    }

    totalRotation += rotation;
    SmartDashboard.putNumber("driveRotation", rotation);

    final boolean robotOriented = false;

    final Vector2 translation = new Vector2(forward, strafe);

    Robot.drivetrainSubsystem.holonomicDrive(translation, rotation, !robotOriented);
  }


@Override
protected boolean isFinished() {
  // boolean isFinished = super.isTimedOut();
  // if (isFinished) {
  //   SmartDashboard.putNumber("totalRotation", totalRotation);
  // }
  //  return isFinished;
  return false;   // TODO: add the actual completion test code
}

@Override
  protected void end() {
    Robot.vision.ledOff();
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}
