package frc.robot.models;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.math.Vector2;

public class AutoNavMath {
    private static final double RADIUS = 28; // in inches

    /*
    largerDeltaY is looking at the displacement from the center of the circle to either of the two tangent points. 
    If the robot has to move to a higher Y-coord up, the displacement is positive, so largerDeltaY should be set as "true"
    */

    public static PathLineSegment circleCircleExternalTangent(String circleString, String pointString, boolean largerDeltaY) {
        Vector2 start = convertPoint(circleString); 
        Vector2 end = convertPoint(pointString); 

        System.out.println("start x, start y | end x, end y");
		System.out.println(start.x + ", " + start.y + " | " + end.x + ", " + end.y);

		double deltaY = end.y - start.y;
		double deltaX = end.x - start.x;
		
		double cosTheta;
		double sinTheta;

		if (Math.abs(deltaY) > Math.abs(deltaX)) {
			double cotTheta = deltaX / deltaY; 
			
			cosTheta = cotTheta / Math.sqrt(1 + cotTheta * cotTheta);
			sinTheta = 1 / Math.sqrt(1 + cotTheta * cotTheta);

		} else {
			double tanTheta = deltaY / deltaX;

			cosTheta = Math.sqrt(1 / (tanTheta * tanTheta + 1));
			sinTheta = (tanTheta / Math.sqrt(tanTheta * tanTheta + 1));
		}

		// construct reference triangle to find the x,y displacement from center to tangent point
		// x displacement = rsin(theta)
		// y displacement = rcos(theta)
		double displacementX = RADIUS * sinTheta; 
		double displacementY = RADIUS * cosTheta; 
		System.out.println("displacementX, displacementY = " + displacementX + ", " + displacementY);

		// add reference triangle values to circle centers
        Vector2 startTan;
        Vector2 endTan; 

        if (largerDeltaY) {
            startTan = new Vector2((start.x + displacementX), (start.y + displacementY)); 
		    endTan = new Vector2((end.x + displacementX), (end.y + displacementY)); 
        } else {
            startTan = new Vector2((start.x - displacementX), (start.y - displacementY)); 
		    endTan = new Vector2((end.x - displacementX), (end.y - displacementY));
        }
		
		return new PathLineSegment(startTan, endTan); 
    }

    public static PathLineSegment circleCircleInternalTangent(String startString, String endString, boolean largerDeltaY) {
        Vector2 start = convertPoint(startString);
        Vector2 end = convertPoint(endString); 

        double AA = start.x;
        double BB = start.y;
        double CC = end.x;
        double DD = end.y; 
        double rr = RADIUS;

        double Xp = (CC + AA) / 2; 
        double Yp = (DD + BB) / 2;
        
        // two sets of points on the start circle
        double XDisc12 = rr * (Yp - BB) * Math.sqrt((Xp - AA) * (Xp - AA) + (Yp - BB) * (Yp - BB) - rr * rr);
        double YDisc12 = rr * (Xp - AA) * Math.sqrt((Xp - AA) * (Xp - AA) + (Yp - BB) * (Yp - BB) - rr * rr);

        double Divisor12 = (Xp-AA) * (Xp-AA) + (Yp-BB) * (Yp-BB);
        
        double Xt1 = ((rr * rr * (Xp - AA) + XDisc12) / Divisor12) + AA; 
        double Yt1 = ((rr * rr * (Yp - BB) - YDisc12) / Divisor12) + BB; 
        double Xt2 = ((rr * rr * (Xp - AA) - XDisc12) / Divisor12) + AA;
        double Yt2 = ((rr * rr * (Yp - BB) + YDisc12) / Divisor12) + BB;

        // two sets of points on the end circle
        double XDisc34 = rr * (Yp - DD) * Math.sqrt((Xp - CC) * (Xp - CC) + (Yp - DD) * (Yp - DD) - rr * rr); 
        double YDisc34 = rr * (Xp - CC) * Math.sqrt((Xp - CC) * (Xp - CC) + (Yp - DD) * (Yp - DD) - rr * rr);

        double Divisor34 = (Xp - CC) * (Xp - CC) + (Yp - DD) * (Yp - DD); 

        double Xt3 = ((rr * rr * (Xp - CC) + XDisc34) / Divisor34) + CC;
        double Yt3 = ((rr * rr * (Yp - DD) - YDisc34) / Divisor34) + DD; 
        double Xt4 = ((rr * rr * (Xp - CC) - XDisc34) / Divisor34) + CC;
        double Yt4 = ((rr * rr * (Yp - DD) + YDisc34) / Divisor34) + DD;

        if (largerDeltaY) {
            return new PathLineSegment(new Vector2(Xt1, Yt1), new Vector2(Xt3, Yt3)); 
        } else {
            return new PathLineSegment(new Vector2(Xt2, Yt2), new Vector2(Xt4, Yt4));
        }
    }

    public static PathLineSegment circlePointTangent(String circleString, String pointString, boolean largerDeltaY, boolean circleToPoint) {
        /*
        circleToPoint should be TRUE if the robot is STARTING from the CIRCLE
        should be FALSE if the robot is ENDING on the CIRCLE
        */
        Vector2 circle = convertPoint(circleString); 
        Vector2 point = convertPoint(pointString); 
        
        double a = point.x - circle.x;
		double b = point.y - circle.y; 
		double a2 = a * a;
		double b2 = b * b;
		double r2 = RADIUS * RADIUS; 
		double t1X;
		double t1Y;
		double t2X;
		double t2Y;

        
		if (b == 0) { // case where start and end point have same y value
			t1X = t2X = r2 / a;
			t1Y = RADIUS * Math.sqrt(1 - (r2 / a2));
			t2Y = -RADIUS * Math.sqrt(1 - (r2 / a2));
		} else {
			t1X = (a * r2 - Math.sqrt(b2 * r2 * (a2 + b2 - r2))) / (a2 + b2);
			t1Y = (b2 * r2 + a * Math.sqrt(b2 * r2 * (a2 + b2 - r2))) / (b * (a2 + b2));

			t2X = (a * r2 + Math.sqrt(b2 * r2 * (a2 + b2 - r2))) / (a2 + b2);
			t2Y = (b2 * r2 - a * Math.sqrt(b2 * r2 * (a2 + b2 - r2))) / (b * (a2 + b2));
		}
        
        Vector2 tanLine;

        if (largerDeltaY) {
            tanLine = new Vector2(t1X, t1Y); 
        } else {
            tanLine = new Vector2(t2X, t2Y);            
        }

        if (circleToPoint) {
            return new PathLineSegment(tanLine, point); 
        } else {
            return new PathLineSegment(point, tanLine); 
        }        		
        
    }

    public static Vector2 convertPoint(String p) {
        int y = 150 - (p.charAt(0) - 'A') * 30; 
        int x = Integer.parseInt(p.substring(1, p.length() - 1)) * 30; 
        return new Vector2(x, y); 
    } 
}
