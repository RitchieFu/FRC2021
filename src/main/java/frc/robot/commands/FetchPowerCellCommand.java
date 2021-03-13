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
import frc.robot.RobotMap;
import frc.robot.models.VisionObject;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class FetchPowerCellCommand extends Command {
  PIDController angleController;
  PIDController strafeController;
  PIDController forwardController; 
  double gyroAngle;
  double angle;
  double desiredAngle;
  double setPointAngle=6;
  AHRS navX;

  public double totalRotation = 0;
  public FetchPowerCellCommand() {
    requires(Robot.drivetrainSubsystem);
    //PidConstants PID_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
    //angleController = new PIDController(0.3, 0.01, 0.0);
    angleController = new PIDController(0.03, 0.01, 0.0);
    strafeController = new PIDController(0, 0.0, 0.0); // TODO update constants
    forwardController = new PIDController(0.05, 0.01, 0.0); // TODO update constants
    // navX = new AHRS(SPI.Port.kMXP, (byte) 200);
  }

  public FetchPowerCellCommand(double timeout) {
    super(timeout);
    requires(Robot.drivetrainSubsystem);
    //PidConstants PID_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
    //angleController = new PIDController(0.3, 0.01, 0.0);
    angleController = new PIDController(0.03, 0.01, 0.0);
    strafeController = new PIDController(0.0, 0.0, 0.0); // TODO update constants
    forwardController = new PIDController(0.05, 0.01, 0.0); // TODO update constants
    // navX = new AHRS(SPI.Port.kMXP, (byte) 200);
    navX = null;
  }

  @Override
  protected void initialize() {
    
    //Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Robot.drivetrainSubsystem.getGyroscope().getAdjustmentAngle()); 
    //Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(new Rotation2(0, 1, false)); //TODO for testing only
    // VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject("powerCell");
    // if (closestObject == null) {
    //   return; // no object found
    // }
    navX.reset();
    gyroAngle = navX.getFusedHeading()*Math.PI/180;

    //gyroAngle = Robot.drivetrainSubsystem.getGyroscope().getAngle().toRadians();
    // angle =  Math.atan2(closestObject.x, closestObject.z);
    // desiredAngle = angle + gyroAngle;
    
    // if (gyroAngle<=Math.PI/2 && desiredAngle>0) {
    //   setPointAngle = desiredAngle;
    // }
    // else if(gyroAngle<=Math.PI/2 && desiredAngle<0) {
    //   setPointAngle = 0;
    // }
    // else if(gyroAngle>=Math.PI/2 && desiredAngle<2*Math.PI && desiredAngle>Math.PI/2) {
    //   setPointAngle = desiredAngle;
    // }
    // else if(gyroAngle>=Math.PI/2 && desiredAngle<Math.PI/2) {
    //   setPointAngle = Math.PI*2;
    // }
    
    SmartDashboard.putNumber("Vision angle", angle);
    SmartDashboard.putNumber("Desired angle", desiredAngle);
    SmartDashboard.putNumber("initial angle", gyroAngle);
    SmartDashboard.putNumber("SetPoint angle", setPointAngle);
    Vector2 position = new Vector2(0, 0);
    Robot.drivetrainSubsystem.resetKinematics(position, 0);
  }

  @Override
  protected void execute() {
    
    Robot.objectTrackerSubsystem.data();
    double forward = 0;
    double strafe = 0;
    double rotation = 0;

    // VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject("powerCell");
    // if (closestObject == null) {
    //   SmartDashboard.putNumber("driveRotation", 99);
    //   Robot.drivetrainSubsystem.holonomicDrive(new Vector2(0,0), 0, false);
    //   return; // no object found
    // }
    
    //gyroAngle = Robot.drivetrainSubsystem.getGyroscope().getAngle().toRadians();
    
    gyroAngle = navX.getFusedHeading()*Math.PI/180;

    SmartDashboard.putNumber("Process angle", gyroAngle);
    //double angle =  Math.atan2(closestObject.x, closestObject.z);
    //SmartDashboard.putNumber("Updated Vision angle", Math.atan2(closestObject.x, closestObject.z));
    
    //angleController.setSetpoint(desiredAngle);
    angleController.setSetpoint(setPointAngle);
    rotation = angleController.calculate(gyroAngle);
    
    if(rotation > 1){
      rotation = 1;
    }else if(rotation < -1){
      rotation = -1;
    }

    totalRotation += rotation;
    SmartDashboard.putNumber("driveRotation", rotation);
    
    // strafe
    //strafeController.setSetpoint(closestObject.x);
    strafe = strafeController.calculate(0);

    if(strafe > 1){
      strafe = 1;
    }else if (strafe < -1){
      strafe = -1;
    }

    SmartDashboard.putNumber("driveStrafe", strafe);

    // forward
    //forwardController.setSetpoint(closestObject.z-RobotMap.TARGET_TRIGGER_DISTANCE); // TODO figure out how to implement code that begins intake process 
    forward = forwardController.calculate(0);

    if(forward > 1){
      forward = 1;
    }else if (forward < -1){
      forward = -1;
    }

    SmartDashboard.putNumber("driveForward", forward);
    
    final boolean robotOriented = false;

    final Vector2 translation = new Vector2(-forward * 0, -strafe * 0);
    // final Vector2 translation = new Vector2(0, 0);


    Robot.drivetrainSubsystem.holonomicDrive(translation, rotation*0, !robotOriented);
  }


@Override
protected boolean isFinished() {
  double tolerance = 2;
  //VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject("powerCell");
  // if(closestObject == null) {
  //   return true;
  // }//TODO could lose sight for small amount of time causing command to finish early

  //return Math.abs(closestObject.z-RobotMap.TARGET_TRIGGER_DISTANCE) <= tolerance;
  return false;
  // boolean isFinished = super.isTimedOut();
  // if (isFinished) {
  //   SmartDashboard.putNumber("totalRotation", totalRotation);
  // }
  //  return isFinished;
     // TODO: add the actual completion test code
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
