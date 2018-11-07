package org.firstinspires.ftc.teamcode;

public class Segment {
	
	public double dt;
	public double x;
	public double y;
	public double position;
	public double velocity;
	public double acceleration;
	public double jerk;
	public double heading;
	
	public Segment(String[] segment) {
		
		try {
			
			dt = Double.parseDouble(segment[0]);
			x = Double.parseDouble(segment[1]);
			y = Double.parseDouble(segment[2]);
			position = Double.parseDouble(segment[3]);
			velocity = Double.parseDouble(segment[4]);
			acceleration = Double.parseDouble(segment[5]);
			jerk = Double.parseDouble(segment[6]);
			heading = Double.parseDouble(segment[7]);
			
		}catch(Exception ex) {
			
		}
		
	}

}
