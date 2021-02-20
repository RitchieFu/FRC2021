/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.models.VisionObject;
import frc.robot.Robot;

public class AreWeThereYetCommand extends Command {
    private double TargetTriggerDistance = 26; // inches
    
    public AreWeThereYetCommand() {
    
    }

  // Called just before this Command runs the first time
    @Override
    protected void initialize() {

    }

  // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

  // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
      Robot.objectTrackerSubsystem.data();
      VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject("powerCell");
      if (closestObject == null) 
        return false; // no object found

      return closestObject.z < this.TargetTriggerDistance;
    }

  // Called once after isFinished returns true
    @Override
    protected void end() {
        super.end();
    }


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}