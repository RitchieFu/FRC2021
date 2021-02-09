package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 *
 */
public class ObjectTracker extends Subsystem {
	NetworkTable monsterVision; 
     
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	public ObjectTracker(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
        monsterVision = inst.getTable("MonsterVison");			   
	}
    
    private NetworkTableEntry getEntry(Integer index, String subkey) {
        try {
            NetworkTable table = monsterVision.getSubTable(index.toString());
            NetworkTableEntry entry = table.getEntry(subkey);
            return entry;
        }
        catch (Exception e){
            return null;
        } 
    }
	
	public boolean powerCellExists(int index) { //TODO add overload for objectId
        NetworkTableEntry entry = getEntry(index,"Label");
        if (entry == null) {
            return false;
        }
        String label = entry.getString("");
        return label.equals("powerCell");
	}

    public int numberOfObjects() {
        NetworkTableEntry entry = monsterVision.getEntry("numberOfObjects");
        if (entry == null) {
            return 0; 
        }
        return (int) entry.getDouble(0); // truncates decimal, AKA rounds down
    }
    
    public Double getX(int index) {
        NetworkTableEntry entry = getEntry(index,"x");
        if (entry == null) {
            return null; 
        }
        Double x = entry.getDouble(0);
        return x;
    }
    
    public Double getY(int index) {
        NetworkTableEntry entry = getEntry(index,"y");
        if (entry == null) {
            return null; 
        }
        Double y = entry.getDouble(0);
        return y;
    }

    public Double getZ(int index) {
        NetworkTableEntry entry = getEntry(index,"z");
        if (entry == null) {
            return null; 
        }
        Double z = entry.getDouble(0);
        return z; 
    }


	public Double getXAngle(int index) {
        double x = getX(index);
        double z = getZ(index);
        double angle = Math.atan2(x, z);
        return angle;
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}