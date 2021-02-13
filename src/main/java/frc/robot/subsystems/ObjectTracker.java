package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;


class VisionObject {
	String objectLabel;
	int x;
	int y;
	int z;
	int confidence;

	// TODO - figure out ~magic~ to pass data from camera/pi to robot
}


/**
 *
 */
public class ObjectTracker extends Subsystem {
	NetworkTable monsterVision; 
    ArrayList<VisionObject> foundObjects; 

	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	public ObjectTracker(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
        monsterVision = inst.getTable("MonsterVison");	
        	
        monsterVision.addEntryListener(
            "ObjectTracker",
            (monsterVision, key, entry, value, flags) -> {
           System.out.println("ObjectTracker changed value: " + value.getValue());
        }, 
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
                 
	}
    
    // private NetworkTableEntry getEntry(Integer index, String subkey) {
    //     try {
    //         NetworkTable table = monsterVision.getSubTable(index.toString());
    //         NetworkTableEntry entry = table.getEntry(subkey);
    //         return entry;
    //     }
    //     catch (Exception e){
    //         return null;
    //     } 
    // }
	
	public String getLabel(int index) { //TODO add overload for objectId
        if (foundObjects.size() <= index) {
            return null; 
        }
        return foundObjects.get(index).objectLabel;
	}

    public int numberOfObjects() {
        return foundObjects.size(); 
    }
    
    public Integer getX(int index) {
        if (foundObjects.size() <= index) {
            return null; 
        }
        return foundObjects.get(index).x;
    }
    
    public Integer getY(int index) {
        if (foundObjects.size() <= index) {
            return null;
        }
        return foundObjects.get(index).y;
    }

    public Integer getZ(int index) {
        if (foundObjects.size() <= index) {
            return null;
        }
        return foundObjects.get(index).z;
    }

    public Integer getConfidence(int index) {
        if (foundObjects.size() <= index) {
            return null;
        }
        return foundObjects.get(index).confidence;
    }


	public Double getXAngle(int index) {
        Integer x = getX(index);
        Integer z = getZ(index);
        if (x == null || z == null) {
            return null; 
        }
        double angle = Math.atan2(x, z);
        return angle;
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}