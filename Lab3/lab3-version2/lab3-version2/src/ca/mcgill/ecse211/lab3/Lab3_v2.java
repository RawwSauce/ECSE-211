/**
 * Lab2.java
 * 
 */

package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3_v2 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  
  //private static final Port usPort = LocalEV3.get().getPort("S4");
  public static final double WHEEL_RAD = 2.18;
  public static final double TRACK = 12.25;
  public static final double leftRadius = 2.2;
  public static final double rightRadius = 2.2;
  public static final double TILE_SIZE = 30.48;
  
  public static int x[] = {0,1,2,2,1};
  public static int y[] = {2,1,2,1,0};

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    //OdometryCorrection odometryCorrection = new OdometryCorrection(); 
    Display odometryDisplay = new Display(lcd); // No need to change
    //Avoid avoid = new Avoid(leftMotor, rightMotor);
    NavigationA navigationA = new NavigationA(leftMotor, rightMotor);


    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString(" avoid | Drive  ", 0, 2);
      lcd.drawString("obstacle| in    ", 0, 3);
      lcd.drawString("       | navigation", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {
    		//NavigationAvoid navigationAvoid = new NavigationAvoid(leftMotor, rightMotor);
      // Navigation with obstacle avoidance
    		

      // Display changes in position
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      
      //Thread avoidThread = new Thread(avoid);
      //avoidThread.start();
      Thread navigationAThread = new Thread(navigationA);
      navigationAThread.start();

    } else {
      // clear the display
      lcd.clear();

     // buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

      // Start odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      

      // spawn a new Thread to avoid SquareDriver.drive() from blocking
      (new Thread() {
        public void run() {
          Navigation navigation = new Navigation(leftMotor, rightMotor);
          //int x[] = {1,0,2,2,1};
          //int y[] = {1,2,2,1,0};
          for(int i = 0; i < 5; i++) {
        	  	navigation.travelTo(x[i] * TILE_SIZE, y[i] * TILE_SIZE);
        	  	try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
          }
          
        }
      }).start();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
