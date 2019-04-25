package ca.mcgill.ecse211.lab3;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class NavigationA implements Runnable{
	private double x;
	private double y;
	private double currentX;
	private double currentY;
	private double dx;
	private double dy;
	private double distance;
	private Odometer odometer;
	private double t;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor centerMotor;
	private double turnAngle;
	private int counter;
	
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	private EV3UltrasonicSensor usSensor;
	
	/**
	 * constructor of NavigationA
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	public NavigationA(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = Odometer.getOdometer();
		this.usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
		this.centerMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("A"));
	}
	
	/**
	 * run method for thread
	 */
	public void run() {
		while(counter < 5) {
    	  	try {
				travelTo(Lab3_v2.x[counter] * Lab3_v2.TILE_SIZE, Lab3_v2.y[counter] * Lab3_v2.TILE_SIZE);
			} catch (OdometerExceptions e) {
				e.printStackTrace();
			}
      }
	}
	/**
	 * this method calculates the direction the robot needs to travel and check if there are blocks 
	 * in the path the robot is about to move to, if there is, then call the avoidWall method
	 * @param x
	 * @param y
	 * @throws OdometerExceptions 
	 */
	public void travelTo(double x, double y) throws OdometerExceptions {
		this.x = x;
		this.y = y;
		
		currentX = odometer.getX();
		currentY = odometer.getY();
		dx = x - currentX;
		dy = y - currentY;
		
		distance = Math.sqrt(Math.pow(dx,2)+Math.pow(dy, 2));
		t = Math.toDegrees(Math.atan2(dx, dy));
		
		turnTo(t);
		if(needAvoid()) {
			avoidWall();
		}
		else {
			leftMotor.setAcceleration(500);
			rightMotor.setAcceleration(500);
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, distance/2), true);
			rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, distance/2), false);
			if(needAvoid()) {
				avoidWall();
			}
			else {
				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);
				leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, distance/2), true);
				rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, distance/2), false);
				counter++;
			}
		}				
	}
	/**
	 * this method is called if there is a block in front of the robot. 
	 */
	public void avoidWall() {
		if(odometer.getX()>=53.0 && odometer.getX()<=67.0) {// near eastern edge
			if(odometer.getT()>=260.0 || odometer.getT()<=100.0) {// going north
				avoidleft();
			}else if(odometer.getT()>=173.0 && odometer.getT()<=187.0) {// going south
				avoidright();
			}
			else {
				avoidright();
			}
		}
		else if(odometer.getX()>=-7.0 && odometer.getX()<=7.0) {//near western edge
			if(odometer.getT()>=353.0 || odometer.getT()<=7.0) {// going north
				avoidright();
			}else if(odometer.getT()>=173.0 && odometer.getT()<=187.0) {// going south
				avoidleft();
			}
			else {
				avoidright();
			}
		}
		else if(odometer.getY()>=53.0 && odometer.getY()<=67.0) {// near northern edge
			if(odometer.getT()>= 83 && odometer.getT()<= 97.0) {// going east
				avoidright();
			}else if(odometer.getT()>=260 && odometer.getT()<=270.0) {// going west
				avoidleft();;
			}
			else {
				avoidright();
			}
		}
		else if(odometer.getY()>=-7.0 && odometer.getY()<=7.0) {// near southern edge
			if(odometer.getT()>= 83 && odometer.getT()<= 97.0) {// going east
				avoidleft();
			}else if(odometer.getT()>=263 && odometer.getT()<=277.0) {// going west
				avoidright();
			}
			else{
				avoidright();
			}
		}
		else {
			avoidright();
		}	
	}
	/**
	 * 
	 * @return boolean
	 */
	public boolean needAvoid() {
		leftMotor.stop();
		rightMotor.stop();
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
        // this instance
		float[] usDistance = new float[3];
		sampleProvider.fetchSample(usDistance, 0);
		LCD.drawString("us Distance: " + usDistance[0], 0, 5);
		if(usDistance[0]*100 < 20) {
			return true;
		}
		return false;
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
		leftMotor.rotate(convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, turnAngle), true);
		rightMotor.rotate(-convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, turnAngle), false);
	}
	
	
	public boolean isNavigating(){
		return leftMotor.isMoving() || rightMotor.isMoving();
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	public void avoidleft() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		//both motors turn 90 degrees
		leftMotor.rotate(convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, -90.0), true);
		rightMotor.rotate(-convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, -90.0), false);
			
		centerMotor.setSpeed(150);
		centerMotor.rotateTo(90);
			
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		float[] usDistance = new float[3];
		sampleProvider.fetchSample(usDistance , 0);
		float distance = usDistance[0] * 100;
		while(usDistance[0] * 100 < 30){
			leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 25 ), true);     
			rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 25 ), false);		
			sampleProvider.fetchSample(usDistance , 0);
		}
		centerMotor.rotateTo(0);
		leftMotor.stop();
		rightMotor.stop();
		//turns left 90 degrees
		leftMotor.rotate(convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, 90.0), false);
		//moves straight
		leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, distance * 2.5 ), true);     
		rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, distance * 2.5 ), false);	
				
		centerMotor.rotateTo(90);
		sampleProvider.fetchSample(usDistance , 0);
		while(usDistance[0] * 100 < 30){
			leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 20 ), true);     
			rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 20 ), false);		
			sampleProvider.fetchSample(usDistance , 0);
		}
		centerMotor.rotateTo(0);
		leftMotor.stop();
		rightMotor.stop();
	}

	public void avoidright() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		//both motors turn 90 degrees
		leftMotor.rotate(convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, 90.0), false);
			
		centerMotor.setSpeed(150);
		centerMotor.rotateTo(-90);
			
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		float[] usDistance = new float[3];
		sampleProvider.fetchSample(usDistance , 0);
		float distance = usDistance[0] * 100;
		while(usDistance[0] * 100 < 30){
			leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 30 ), true);     
			rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 30 ), false);		
			sampleProvider.fetchSample(usDistance , 0);
		}
		centerMotor.rotateTo(0);
		leftMotor.stop();
		rightMotor.stop();
		//turns left 90 degrees
		leftMotor.rotate(convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, -90.0), true);
		rightMotor.rotate(-convertAngle(Lab3_v2.WHEEL_RAD, Lab3_v2.TRACK, -90.0), false);
		//moves straight
		leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, distance*3 ), true);     
		rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, distance*3 ), false);	
				
		centerMotor.rotateTo(-90);
		sampleProvider.fetchSample(usDistance , 0);
		while(usDistance[0] * 100 < 30){
			leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 20 ), true);     
			rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 20 ), false);		
			sampleProvider.fetchSample(usDistance , 0);
		}
		centerMotor.rotateTo(0);
		leftMotor.stop();
		rightMotor.stop();
	}
}

