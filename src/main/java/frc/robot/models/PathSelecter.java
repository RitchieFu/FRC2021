package frc.robot.models;
import java.io.IOException;
import java.util.HashMap;

import frc.robot.Robot;
import frc.robot.subsystems.Vision;

public class PathSelecter {
    HashMap<String, VisionObject[]> pathDictionary;
   
    public void loadPathDictionary() {
        pathDictionary = new HashMap<String, VisionObject[]>();
        try {
            VisionObject[] pathArray =Robot.objectTrackerSubsystem.loadVisionSnapshot("/home/lvuser/PathBlueB.json");
            pathDictionary.put("PathBlueB", pathArray);
        }
        catch (IOException e) {
            System.out.println("error loading path dictionary");
        } 

    }


    public static String choosePath(){
        Robot.objectTrackerSubsystem.data();

        VisionObject[] objects = Robot.objectTrackerSubsystem.getObjectsOfType("powerCell");

        if (objects == null || objects.length<2) {
            return null;
        }

        VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject("powerCell");
        VisionObject secondObject = Robot.objectTrackerSubsystem.getSecondClosestObject("powerCell");

        if (closestObject.z<=85) {
            if(Math.abs(secondObject.x)<=15) {
                return "PathRedB";
            }
            else{
                return "PathRedA";
            }
        }
        else {
            if(objects.length == 3) {
                return "PathBlueB";
            }
            else {
                return "PathBlueA";
            }
        }
    }
    
}
