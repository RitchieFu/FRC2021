package frc.robot.models;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.DrivetrainSubsystem;

public class VisionObject {
    public String objectLabel; 
    public double x;
    public double y;
    public double z;
    public double confidence;
    

    public VisionObject(String objectType, double x, double y, double z)
    {
        this.objectLabel = objectType;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void motionCompensate(DrivetrainSubsystem drivetrainSubsystem, boolean compensateTranslation)
    {
        if (compensateTranslation) {
        // Normally, we'd subtract the distance travelled.  However, the camera points off the back
        // of the robot.  Therefore, motion in the direction the camera is aiming is returned by
        // getVelocityX() as negative.
            
            z += drivetrainSubsystem.getKinematicVelocity().x * RobotMap.OBJECT_DETECTION_LATENCY;
            x -= drivetrainSubsystem.getKinematicVelocity().y * RobotMap.OBJECT_DETECTION_LATENCY;
            SmartDashboard.putNumber("X Velocity", drivetrainSubsystem.getKinematicVelocity().x);
            SmartDashboard.putNumber("Z Velocity", drivetrainSubsystem.getKinematicPosition().y);
        }
        
        double omega = drivetrainSubsystem.getGyroscope().getRate();
        double theta = omega * RobotMap.OBJECT_DETECTION_LATENCY;
        //System.out.println("theta: " + theta); 

        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);

        double newZ = z * cosTheta - x * sinTheta;
        double newX = z * sinTheta + x * cosTheta;

        z = newZ;
        x = newX;
    }
};




/*
from marshmallow import Schema, fields


class VisionObject(Schema):
};


/*
from marshmallow import Schema, fields


class VisionObject(Schema):
    objectType = fields.Str(required=True)
    x = fields.Float(required=True)
    y = fields.Float(required=True)
    z = fields.Float(required=True)

class Something(Schema):
    items = fields.List(fields.Nested(VisionObject))
*/