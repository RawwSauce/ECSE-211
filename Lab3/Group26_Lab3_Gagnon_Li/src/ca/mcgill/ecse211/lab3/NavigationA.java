package ca.mcgill.ecse211.lab3;

/**
 * This class is used to drive the robot to specified locations with avoiding block
 */
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
	private double WpTreshold=4.0;
	
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED =100;
	
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
				travelTo(Lab3.x[counter] * Lab3.TILE_SIZE, Lab3.y[counter] * Lab3.TILE_SIZE);
			} catch (OdometerExceptions e) {
				e.printStackTrace();
			}
      }
	}
	
	/**
	 * See if the robot is on one of the waypoints
	 * @param x
	 * @param y
	 * @return
	 */
	public boolean isRightPosition(double x, double y) {
		
		return (odometer.getX()<(x+WpTreshold) && odometer.getX()>(x-WpTreshold) && odometer.getY()<(y+WpTreshold) 
				&& odometer.getY()>(y-WpTreshold));
		
	}
	/**
	 * This method causes the robot to move to the destination and check if there are blocks 
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
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, (distance)+Lab3.CorrectionFactor), true);
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, (distance+Lab3.CorrectionFactor)), true);
			while(isNavigating()) {
				if(needAvoid()) {
					avoidWall();
				}
			}
			if(isRightPosition(x,y)) {
				counter++;
			}
		}
	}
	
	/**
	 * this method decides whether to avoid the block from left or left based on its current position 
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
	 * this method use the ultrasonic sensor to determine whether the robot needs to avoid block
	 * @return boolean -true if need to avoid, false otherwise
	 */
	public boolean needAvoid() {
		//leftMotor.stop();
		//rightMotor.stop();
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
        // this instance
		float[] usDistance = new float[3];
		sampleProvider.fetchSample(usDistance, 0);
		LCD.drawString("us Distance: " + usDistance[0], 0, 5);
		if(usDistance[0]*100 < 15) {
			return true;
		}
		return false;
	}
	
	/**
	 * This method causes the robot to turn to the absolute heading theta
	 * calculate the angle based on the current heading
	 * @param theta
	 */
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
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, turnAngle), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, turnAngle), false);
	}
	
	/**
	 * This method checks if the robot is moving
	 * @return boolean - true if robot is moving, false otherwise
	 */
	public boolean isNavigating(){
		return leftMotor.isMoving() || rightMotor.isMoving();
	}
	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * This method causes the robot to avoid the wall from the left
	 */
	public void avoidleft() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		//both motors turn 90 degrees
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -90.0), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -90.0), false);
		
		centerMotor.setSpeed(150);
		centerMotor.rotateTo(90);
			
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		float[] usDistance = new float[3];
		sampleProvider.fetchSample(usDistance , 0);
		float distance = usDistance[0] * 100;
		while(usDistance[0] * 100 <= 30){
			
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 28), true);     
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 28), false);		
			sampleProvider.fetchSample(usDistance , 0);
		}
		
		//leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 18 ), true);     
		//rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 18), false);
		
		centerMotor.rotateTo(0);
		leftMotor.stop();
		rightMotor.stop();
		//turns left 90 degrees
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
		//moves straight
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * 2.1 ), true);     
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * 2.1 ), false);	
				
		centerMotor.rotateTo(90);
		sampleProvider.fetchSample(usDistance , 0);
		while(usDistance[0] * 100 <= 30){
			
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20 ), true);     
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20 ), false);		
			sampleProvider.fetchSample(usDistance , 0);
		}
		
		//leftMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 10 ), true);     
		//rightMotor.rotate(convertDistance(Lab3_v2.WHEEL_RAD, 10), false);
		
		centerMotor.rotateTo(0);
		leftMotor.stop();
		rightMotor.stop();
	}
	
	/**
	 * This method causes the robot to avoid the wall from the right
	 */
	public void avoidright() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		//both motors turn 90 degrees
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
		
		centerMotor.setSpeed(150);
		centerMotor.rotateTo(-90);
			
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		float[] usDistance = new float[3];
		sampleProvider.fetchSample(usDistance , 0);
		float distance = usDistance[0] * 100;
		while(usDistance[0] * 100 <= 30){
			
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 28), true);     
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 28), false);		
			sampleProvider.fetchSample(usDistance , 0);
		}
		
		
		centerMotor.rotateTo(0);
		leftMotor.stop();
		rightMotor.stop();
		//turns left 90 degrees
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -90.0), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -90.0), false);
		//moves straight
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * 2.1 ), true);     
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * 2.1 ), false);	
				
		centerMotor.rotateTo(-90);
		sampleProvider.fetchSample(usDistance , 0);
		while(usDistance[0] * 100 < 30){
			
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20 ), true);     
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20 ), false);		
			sampleProvider.fetchSample(usDistance , 0);
		}
		
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 10 ), true);     
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 10), false);
		
		centerMotor.rotateTo(0);
		leftMotor.stop();
		rightMotor.stop();
	}
}

