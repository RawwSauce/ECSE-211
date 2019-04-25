package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Avoid implements Runnable{
	private EV3UltrasonicSensor usSensor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor centerMotor;
	private static int sampleSize;
	
	private double x;
	private double y;
	private double currentX;
	private double currentY;
	private double dx;
	private double dy;
	private double distance;
	private Odometer odometer;
	private double t;
	private double turnAngle;
	
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double track = 12.7;
	private static final double WHEEL_RADIUS = 2.2;
	
	private boolean avoid = false;
	private boolean finish = false;
	
	public Avoid(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions{
		this.odometer = Odometer.getOdometer();
		this.usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
		this.centerMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("A"));
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	

	public void run(){
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
	                                                              // this instance
	    float[] usDistance = new float[3];
	    //	while(true) {	  
	    		sampleProvider.fetchSample(usDistance, 0);
		
	    		//if(usDistance[0] * 100 < 15){
	    			//leftMotor.stop();
	    			//rightMotor.stop();

	    			leftMotor.setSpeed(ROTATE_SPEED);
	    			rightMotor.setSpeed(ROTATE_SPEED);
	    		
	    			leftMotor.rotate(convertAngle(WHEEL_RADIUS, track, 90.0), true);
	    			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, track, 90.0), false);
	    		
	    			centerMotor.setSpeed(150);
	    			centerMotor.rotateTo(-90);
	    		
	    			sampleProvider.fetchSample(usDistance , 0);
	    			while(usDistance[0] * 100 < 20){
	    				leftMotor.rotate(convertDistance(WHEEL_RADIUS, 10 ), true);     
	    				rightMotor.rotate(convertDistance(WHEEL_RADIUS, 10 ), false);		
	    				sampleProvider.fetchSample(usDistance , 0);
	    			}
	    			centerMotor.rotateTo(0);
	    			leftMotor.stop();
	    			rightMotor.stop();
	    		//}
	    		//else{
	    			//int x[] = {1};
	    			//int y[] = {1};
	    			//travelTo(x[0]*Lab3.TILE_SIZE, y[0]*Lab3.TILE_SIZE);
	    		//}
	    //	}
	}
	public void travelTo(double x, double y) {
		this.x = x;
		this.y = y;
		
		currentX = odometer.getX();
		currentY = odometer.getY();
		dx = x - currentX;
		dy = y - currentY;
		
		distance = Math.sqrt(Math.pow(dx,2)+Math.pow(dy, 2));
		t = Math.toDegrees(Math.atan2(dx, dy));
		
		turnTo(t);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RADIUS, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RADIUS, distance), false);
		
		
	}
	
	public void turnTo(double theta) {
		turnAngle = theta - odometer.getT();
		//calculate the minimal angle
		if(turnAngle < -180) {
			turnAngle = turnAngle + 360;
		}
		else if(turnAngle > 180) {
			turnAngle = turnAngle - 360;
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, track, turnAngle), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, track, turnAngle), false);
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
