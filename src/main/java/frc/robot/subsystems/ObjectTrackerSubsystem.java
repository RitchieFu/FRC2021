package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import com.google.gson.Gson;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.models.VisionObject;





/**
 *
 */
public class ObjectTrackerSubsystem extends Subsystem {
	NetworkTable monsterVision; 
    VisionObject[] foundObjects; 

	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	public ObjectTrackerSubsystem(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
        monsterVision = inst.getTable("MonsterVision");	
        
        // monsterVision.addEntryListener(
        //     "ObjectTracker",
        //     (monsterVision, key, entry, value, flags) -> {
        //    System.out.println("ObjectTracker changed value: " + value.getValue());
        // }, 
        // EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

                 
    }
    

    
    public void data() {
        Gson gson = new Gson();
        NetworkTableEntry entry = monsterVision.getEntry("ObjectTracker");
        if(entry==null) {
            return;
        }
        String json = entry.getString("ObjectTracker");
        foundObjects = gson.fromJson(json, VisionObject[].class);
        
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
	
	public VisionObject getClosestObject(String objectLabel) {
        if (foundObjects == null || foundObjects.length == 0) {
            return null; 
        }
        VisionObject[] objects = getObjectsOfType(objectLabel);
        return foundObjects[0];
	}

    public int numberOfObjects() {
        return foundObjects.length; 
    }

    public VisionObject[] getObjectsOfType(String objectLabel) {

        if (foundObjects == null || foundObjects.length == 0)
            return null;
            List<VisionObject> filteredResult = Arrays.asList(foundObjects)
            .stream()
            .filter(vo -> vo.objectLabel.equals(objectLabel))
            .collect(Collectors.toList());

            VisionObject filteredArray[] = new VisionObject[filteredResult.size()];
            return filteredResult.toArray(filteredArray);

    }



    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

