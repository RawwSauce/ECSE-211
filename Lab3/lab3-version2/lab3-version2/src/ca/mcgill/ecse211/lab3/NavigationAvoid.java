package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.lab3.Navigation;

public class NavigationAvoid implements UltrasonicController{
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor centerMotor;
	private Odometer odometer;
	private EV3UltrasonicSensor usSensor;
	
	private static final double track = 12.7;
	private static final double WHEEL_RADIUS = 2.2;
	private static final double TILE_SIZE = 30.48;
	
	private SampleProvider us;
	private float[] usData;
	
	public NavigationAvoid(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		//this.usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		this.centerMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("A"));
		try {
			this.odometer = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
		
	/*public void processUSData(int distance) {
		int sumRight = 0;
		int sumLeft = 0;
		//get the samples from the right
		
		for(int i = 0; i <= 18; i++) {
			centerMotor.rotateTo(5);
			usSensor.fetchSample(dataRight, 0);
			sumRight += (int) (dataRight[0] * 100);
		}
		int averageRight = sumRight/19;
		
		//get the samples from the left
		centerMotor.rotateTo(-180);
		for(int i = 0; i <= 18; i++) {
			centerMotor.rotateTo(5);
			usSensor.fetchSample(dataLeft, i);
			sumLeft += (int) (dataLeft[0] * 100);
		}
		int averageLeft = sumLeft/19;
		//obstacle is in front of the robot
		if(Math.abs(averageRight-averageLeft) < 5) {
			//turn 90 degrees to the right
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, track, 90.0), true);
		    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, track, 90.0), false);
		    //move straight for 20cm
			leftMotor.rotate(convertDistance(WHEEL_RADIUS, 20), true);
			rightMotor.rotate(convertDistance(WHEEL_RADIUS, 20), false);
			//turn 90 degrees to the left
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, track, 90.0), true);
		    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, track, 90.0), false);
		    //move straight
		    leftMotor.rotate(convertDistance(WHEEL_RADIUS, 20), true);
			rightMotor.rotate(convertDistance(WHEEL_RADIUS, 20), false);
			//turn 90 degrees to the left
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, track, 90.0), true);
		    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, track, 90.0), false);
		    //move straight
		    leftMotor.rotate(convertDistance(WHEEL_RADIUS, 20), true);
			rightMotor.rotate(convertDistance(WHEEL_RADIUS, 20), false);
		}	
	}*/
	
	public void run() {
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	@Override
	public void processUSData(int distance) {
		// TODO Auto-generated method stub
		if(distance < 20) {
			leftMotor.stop();
			rightMotor.stop();
			centerMotor.rotateTo(-90);
			//turn 90 degrees to the right
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, track, 90.0), true);
		    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, track, 90.0), false);
		    //move straight for 20cm
			leftMotor.rotate(convertDistance(WHEEL_RADIUS, 20), true);
			rightMotor.rotate(convertDistance(WHEEL_RADIUS, 20), false);
		}
		else{
		}
	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return 0;
	}

}
