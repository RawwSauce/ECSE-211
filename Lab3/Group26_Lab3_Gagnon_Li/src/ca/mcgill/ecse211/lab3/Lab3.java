package ca.mcgill.ecse211.lab3;
/**
 * Lab3_v3.java
 * This class implements Navigation lab on the EV3 platform.
 * @author Lily Li, Rene Gagnon
 * 
 */

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Lab3 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  
  public static final double WHEEL_RAD = 2.156;
  public static final double TRACK = 12.4;
  public static final double leftRadius = 2.2;
  public static final double rightRadius = 2.2;
  public static final double TILE_SIZE = 30.48;
  
  public static final double CorrectionFactor=0.6;
  
  //map1
  	//public static int x[] = {0,1,2,2,1};
  	//public static int y[] = {2,1,2,1,0};
  //map2
  	//public static int x[] = {1,0,2,2,1};
  	//public static int y[] = {1,2,2,1,0};
  //map3
  public static int x[] = {1,2,2,0,1};
  public static int y[] = {0,1,2,2,1};
  //map4
  //public static int x[] = {0,1,1,2,2};
  //public static int y[] = {1,2,0,1,2};
  
  /**
   * This main method runs at start, creates main menu, initializes motors
   * at the start and start navigation or obstacle avoidance based
   * on the user input
   * @param args NOT USED
   * @throws OdometerExceptions
   */

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    Display odometryDisplay = new Display(lcd); 
    NavigationA navigationA = new NavigationA(leftMotor, rightMotor);

    do {
      // clear the display
      lcd.clear();
      // ask the user whether the motors should drive in navigation or avoid obstacle while navigation
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString(" avoid |Drive   ", 0, 2);
      lcd.drawString("       |in      ", 0, 3);
      lcd.drawString("       |navigation", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {
      // Navigation with obstacle avoidance
 
      // Display changes in position
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      
      //start navigationA thread 
      Thread navigationAThread = new Thread(navigationA);
      navigationAThread.start();

    } else {
      // clear the display
      lcd.clear();

      // Start odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      

      // spawn a new Thread for navigation class
      (new Thread() {
        public void run() {
          Navigation navigation = new Navigation(leftMotor, rightMotor);
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
