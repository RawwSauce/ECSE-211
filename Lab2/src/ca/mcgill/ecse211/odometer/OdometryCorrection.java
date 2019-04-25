/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private double coords[];
  private EV3ColorSensor cSensor;
  
  private double error=0.25;
  private int counterX=0;
  private int counterY=0;
  private double tilesize=30.48;
  //private int lineNum;
  //private float R,G,B;
  
  private static SampleProvider sampleProvider;
  private static int sampleSize;
  
  //private float[] colorSample;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.cSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
  }

  public boolean BlackLine(float R,float G, float B) {
	  if(R<=0.002 || G<=0.002 || B<=0.002) {
		  //false if the values are too low, happens when looking at something far
		  return false;
	  }
	  else if(R<=0.185 && G<=0.11 && B<=0.08) {
		  // true if the values are within the range of a blackline
		  return true;
	  }
	  else {
		  // false if doesn't match a blackline
		  return false;
	  }
  }
  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      
      cSensor.setFloodlight(lejos.robotics.Color.RED);
      
      sampleProvider = cSensor.getRGBMode();
      sampleSize = sampleProvider.sampleSize();
      float []colorSample = new float[sampleSize];
    		  
      sampleProvider.fetchSample(colorSample, 0);
      LCD.drawString("R: " + colorSample[0] , 0, 4);
      LCD.drawString("G: " + colorSample[1], 0, 5);
      LCD.drawString("B: " + colorSample[2], 0, 6);
      
      coords= odometer.getXYT();
      coords[2]=Math.toRadians(coords[2]);
      
      if(BlackLine(colorSample[0],colorSample[1],colorSample[2])) {
    	  	
    	  	Sound.beep();
    	  	
    	  	if (Math.cos(coords[2]) <= 1 + error
					&& Math.cos(coords[2]) >= 1 - error) { // Look
															// for
															// direction
				odometer.setY(counterY*tilesize); // Setting the Y
														// coordinate of the
														// line for the
														// robot
				counterY++; // increment because going away from origin
				LCD.drawInt(counterX, 0, 7);
				LCD.drawInt(counterY, 0, 8);

			}

			if (Math.sin(coords[2]) <= 1 + error
					&& Math.sin(coords[2]) >= 1 - error) {
				odometer.setX(counterX * tilesize); // Setting the X
														// coordinate of the
														// line for the
														// robot
				counterX++; // increment because going away from origin

				LCD.drawInt(counterX, 0, 7);
				LCD.drawInt(counterY, 0, 8);
			}

			if (Math.cos(coords[2]) <= -1 + error
					&& Math.cos(coords[2]) >= -1 - error) {
				counterY--; // decrement because going away from origin
				odometer.setY(counterY * tilesize); // Setting the Y
														// coordinate of the
														// line for the
														// robot
				LCD.drawInt(counterX, 0, 7);
				LCD.drawInt(counterY, 0, 8);

			}
			if (Math.sin(coords[2]) <= -1 + error
					&& Math.sin(coords[2]) >= -1 - error) {

				counterX--; // decrement because going away from origin
				odometer.setX(counterX * 30.48); // Setting the X
														// coordinate of the
														// line for the
														// robot
				LCD.drawInt(counterX, 0, 7);
				LCD.drawInt(counterY, 0, 8);

			}
    	  	/*
    	  	lineNum++;
    	  	switch(lineNum) {
    	  		//moves North
    	  		case 1: odometer.setY(0);
    	  				odometer.setTheta(0);
    	  				break;
    	  		case 2:	odometer.setY(30.48);
    	  				odometer.setTheta(0);
    	  				break;
    	  		case 3:	odometer.setY(2*30.48);
    	  				odometer.setTheta(0);
    	  				break;
    	  		//moves East
    	  		case 4:	odometer.setX(0);
    	  				odometer.setTheta(90);
    	  				break;
    	  		case 5:	odometer.setX(30.48);
  						odometer.setTheta(90);
  						break;
    	  		case 6:	odometer.setX(2*30.48);
						odometer.setTheta(90);
						break;
    	  		//moves South:
    	  		case 5:	odometer.setY(30.48);
    	  				odometer.setTheta(180);
    	  				break;
    	  		case 6:	odometer.setY(0);
    	  				odometer.setTheta(180);
    	  				break;
    	  		//moves West
    	  		case 7:	odometer.setX(30.48);
    	  				odometer.setTheta(270);
    	  				break;	
    	  		case 8:	odometer.setX(0);
    	  				odometer.setTheta(270);
    	  				break;
    	  			
    	  	}
      */
      }
      
      // TODO Calculate new (accurate) robot position

      // TODO Update odometer with new calculated (and more accurate) vales

      //odometer.setXYT(0.3, 19.23, 5.0);

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
