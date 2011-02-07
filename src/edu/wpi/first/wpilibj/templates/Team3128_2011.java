/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.defaultCode;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;

import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.defaultCode.Action;
import java.util.*;

/**
 * This "BuiltinDefaultCode" provides the "default code" functionality as used in the "Benchtop Test."
 *
 * The BuiltinDefaultCode extends the IterativeRobot base class to provide the "default code"
 * functionality to confirm the operation and usage of the core control system components, as
 * used in the "Benchtop Test" described in Chapter 2 of the 2009 FRC Control System Manual.
 *
 * This program provides features in the Disabled, Autonomous, and Teleop modes as described
 * in the benchtop test directions, including "once-a-second" debugging printouts when disabled,
 * a "KITT light show" on the solenoid lights when in autonomous, and elementary driving
 * capabilities and "button mapping" of joysticks when teleoperated.  This demonstration
 * program also shows the use of the user watchdog timer.
 *
 * This demonstration is not intended to serve as a "starting template" for development of
 * robot code for a team, as there are better templates and examples created specifically
 * for that purpose.  However, teams may find the techniques used in this program to be
 * interesting possibilities for use in their own robot code.
 *
 * The details of the behavior provided by this demonstration are summarized below:
 *
 * Disabled Mode:
 * - Once per second, print (on the console) the number of seconds the robot has been disabled.
 *
 * Autonomous Mode:
 * - Flash the solenoid lights like KITT in Knight Rider
 * - Example code (commented out by default) to drive forward at half-speed for 2 seconds
 *
 * Teleop Mode:
 * - Select between two different drive options depending upon Z-location of Joystick1
 * - When "Z-Up" (on Joystick1) provide "arcade drive" on Joystick1
 * - When "Z-Down" (on Joystick1) provide "tank drive" on Joystick1 and Joystick2
 * - Use Joystick buttons (on Joystick1 or Joystick2) to display the button number in binary on
 *   the solenoid LEDs (Note that this feature can be used to easily "map out" the buttons on a
 *   Joystick.  Note also that if multiple buttons are pressed simultaneously, a "15" is displayed
 *   on the solenoid LEDs to indicate that multiple buttons are pressed.)
 *
 * This code assumes the following connections:
 * - Driver Station:
 *   - USB 1 - The "right" joystick.  Used for either "arcade drive" or "right" stick for tank drive
 *   - USB 2 - The "left" joystick.  Used as the "left" stick for tank drive
 *
 * - Robot:
 *   - Digital Sidecar 1:
 *     - PWM 1/3 - Connected to "left" drive motor(s)
 *     - PWM 2/4 - Connected to "right" drive motor(s)
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Team3128_2011 extends SimpleRobot {
	// Declare variable for the robot drive system
	RobotDrive m_robotDrive;		// robot will use PWM 1-4 for drive motors

        Jaguar m_robotDrive2;
//        Jaguar m_robotDrive3;
//
//        Jaguar m_robotDrive4;
//
//        Jaguar m_robotDrive5;

        RobotDrive m_robotDrive6;

        AxisCamera camera;

	// Declare a variable to use to access the driver station object
	DriverStation m_ds;                     // driver station object
	int m_priorPacketNumber;                // keep track of the most recent packet number from the DS
	int m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second

        
	// Declare variables for the two joysticks being used
	Joystick m_rightStick;			// joystick 1 (arcade stick or right tank stick)
	Joystick m_leftStick;			// joystick 2 (tank left stick)

        Joystick m_imaginaryStick;

	static final int NUM_JOYSTICK_BUTTONS = 16;
	boolean[] m_rightStickButtonState = new boolean[(NUM_JOYSTICK_BUTTONS+1)];
	boolean[] m_leftStickButtonState = new boolean[(NUM_JOYSTICK_BUTTONS+1)];

	// Declare variables for each of the eight solenoid outputs
	static final int NUM_SOLENOIDS = 0;
	Solenoid[] m_solenoids = new Solenoid[NUM_SOLENOIDS];

        Gyro gyro;
							// drive mode selection
	static final int UNINITIALIZED_DRIVE = 0;
	static final int ARCADE_DRIVE = 1;
	static final int TANK_DRIVE = 2;
	int m_driveMode;

	// Local variables to count the number of periodic loops performed
	int m_autoPeriodicLoops;
	int m_disabledPeriodicLoops;
	int m_telePeriodicLoops;

        double firstMotorSpeed = 0;//test for value later
        double secondMotorSpeed = 0;//test for value later
        double motoRatio = (firstMotorSpeed/secondMotorSpeed);
        TrackerDashboard trackerDashboard = new TrackerDashboard();
        Action[] actions = new Action[10];

        public static int ACTION_TURN_CCW = -1;
        public static int ACTION_TURN_CW =  +1;
        public static int ACTION_FORWARD =  +2;
        public static int ACTION_BACKWARD = -2;
        public static int ACTION_NONE =      0;

        double autoSecGreen = 0;
        Connections Connections = new Connections();


    /**
     * Constructor for this "BuiltinDefaultCode" Class.
     *
     * The constructor creates all of the objects used for the different inputs and outputs of
     * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
     * providing named objects for each of the robot interfaces.
     */
    public DefaultRobot() {
        System.out.println("BuiltinDefaultCode Constructor Started\n");
            Watchdog.getInstance().feed();
		// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
		m_robotDrive = new RobotDrive(Connections.DriveMotor1, Connections.DriveMotor2,
                        Connections.DriveMotor3, Connections.DriveMotor4);//drive
                //m_robotDrive3 = new Jaguar(Connections.CamMotor);//cam
                m_robotDrive2 = new Jaguar(Connections.ScissorMotor);//scissor

//                m_robotDrive4 = new Jaguar(Connections.WinchMotor1);//winch 1 - 3
//                m_robotDrive5 = new Jaguar(6);// winch 2 - 6

                m_robotDrive6 = new RobotDrive(Connections.LightSpike1,Connections.LightSpike2);//Lights
		// Acquire the Driver Station object
		m_ds = DriverStation.getInstance();
                m_ds.getBatteryVoltage();
                System.out.println(m_ds.getBatteryVoltage());

                gyro = new Gyro(Connections.GyroPort);




		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;

		// Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
		m_rightStick = new Joystick(Connections.RightStick);
		m_leftStick = new Joystick(Connections.LeftStick);
                m_imaginaryStick = new Joystick(Connections.ImaginaryStick);

		// Iterate over all the buttons on each joystick, setting state to false for each
		int buttonNum = 1;						// start counting buttons at button 1
		for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
			m_rightStickButtonState[buttonNum] = false;
			m_leftStickButtonState[buttonNum] = false;
		}

		// Iterate over all the solenoids on the robot, constructing each in turn
		int solenoidNum = 0;						// start counting solenoids at solenoid 1
		for (solenoidNum = 0; solenoidNum < NUM_SOLENOIDS; solenoidNum++) {
			m_solenoids[solenoidNum] = new Solenoid(solenoidNum + 1);
		}

		// Set drive mode to uninitialized
		m_driveMode = UNINITIALIZED_DRIVE;

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;


                {
                    m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
                    m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
                    m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
                    m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
                }


                for(int g = 0;g<actions.length;g++)
                {
                    actions[g] = new Action(0,0.0);
                }

		System.out.println("BuiltinDefaultCode Constructor Completed\n");


	}


	/********************************** Init Routines *************************************/

	public void robotInit() {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.

		System.out.println("RobotInit() completed.\n");
                camera = AxisCamera.getInstance();
                camera.writeResolution(AxisCamera.ResolutionT.k320x240);
                camera.writeBrightness(0);
        }

	public void disabledInit() {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		ClearSolenoidLEDsKITT();
		// Move the cursor down a few, since we'll move it back up in periodic.
		//System.out.println("\x1b[2B");
	}

	public void autonomousInit() {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		ClearSolenoidLEDsKITT();
                final double autoSec = (Timer.getUsClock() / 1000000.0);
                autoSecGreen = autoSec;
                gyro.reset();

                  // if (m_autoPeriodicLoops == (3 * GetLoopsPerSec()))
                    //{ m_robotDrive.drive(0.3,0.0);}
	}

        public int myGetLoopsPerSecond() {
            return (GetLoopsPerSec()/200);

        }

	public void teleopInit() {
            Watchdog.getInstance().feed();
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
		m_driveMode = UNINITIALIZED_DRIVE;		// Set drive mode to uninitialized
		ClearSolenoidLEDsKITT();
	}

	/********************************** Periodic Routines *************************************/
	static int printSec = (int)((Timer.getUsClock() / 1000000.0) + 1.0);
	static final int startSec = (int)(Timer.getUsClock() / 1000000.0);

	public void disabledPeriodic()  {
		// feed the user watchdog at every period when disabled
		Watchdog.getInstance().feed();

		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;

		// while disabled, printout the duration of current disabled mode in seconds
		if ((Timer.getUsClock() / 1000000.0) > printSec) {
			// Move the cursor back to the previous line and clear it.
			//System.out.println("\x1b[1A\x1b[2K");
			System.out.println("Disabled seconds: " + (printSec - startSec) + "\r\n");
			printSec++;
		}
                m_robotDrive6.drive(0.0,0);
	}


        // returns the time to drive to cover a given distance
        public double getSecondsPerDistance(double distanceTotal, double power)
        {
            Watchdog.getInstance().feed();
            double distUnit = (66.5*power)+0.93;
            double time = (distanceTotal/distUnit)*myGetLoopsPerSecond();
            return time;
        }

        //returns time to turn the given degrees
        public double getSecondsForTurn(double degrees)
        {
            Watchdog.getInstance().feed();
            double time = (degrees/90)*myGetLoopsPerSecond();
            return time;
        }

	
	public void autonomousPeriodic() {
            

		// feed the user watchdog at every period when in autonomous
		Watchdog.getInstance().feed();

                actions[0] = new Action(ACTION_FORWARD,144);
                actions[1] = new Action(ACTION_BACKWARD,144);
		m_autoPeriodicLoops++;


               //System.out.println("Angle: "+gyro.getAngle());
                
                //Position 1 - 1 Ball
                //actions[0] = new Action(ACTION_FORWARD,136.0);
                //actions[1] = new Action(ACTION_TURN_CCW,90.0);
//                actions[2] = new Action(ACTION_FORWARD,90.0);
//                actions[3] = new Action(ACTION_TURN_CW,153.0);

//                Position 2 - 2 Balls
//                actions[0] = new Action(ACTION_FORWARD,126.0);
//                actions[1] = new Action(ACTION_TURN_CW,165.0);
//                actions[2] = new Action(ACTION_FORWARD,108.0);
//                actions[3] = new Action(ACTION_TURN_CCW,62.0);


                //Position 3 - 3 Balls
                //actions[0] = new Action(ACTION_FORWARD,126.0);//forward 10.5 feet
                //actions[1] = new Action(ACTION_TURN_CCW,33.0);//turn 33 deg counterclockwise
                //actions[2] = new Action(ACTION_FORWARD,144.0);//forward 12 feet
                //actions[3] = new Action(ACTION_TURN_CCW,85.0);//turn 85 counterclockwise
                //actions[4] = new Action(ACTION_FORWARD,36.0);//forward 3 feet
                //actions[5] = new Action(ACTION_TURN_CCW,81.0);//turn 81 counterclockwise
                //actions[6] = new Action(ACTION_FORWARD,144.0);//forward 12 feet
                //actions[7] = new Action(ACTION_TURN_CW,150.0);//turn 150 clockwise
                //actions[8] = new Action(ACTION_FORWARD,?.0);//forward ? feet
                //actions[9] = new Action(ACTION_TURN_CCW,50.0);//turn 50 counterclockwise

                //All working parts -- DO NOT CHANGE

                double nowSec = (Timer.getUsClock() / 1000000.0);

                if(gyro.getAngle() < 15 && gyro.getAngle() > -15)
                {

                for(int i = 0;i<actions.length;i++)
                {
                    double totalTime = allPreviousTimes(i);
                    if (actions[i].type == ACTION_TURN_CW)//then were turning clockwise
                    {
                        if (nowSec > totalTime && nowSec < actions[i].returnTime() + totalTime) {
                            m_robotDrive.drive(0.7, 1.0);
                            System.out.println("turning clockwise");
                        }
                    } else if (actions[i].type == ACTION_TURN_CCW)//then were turning counterclockwise
                    {
                        if (nowSec > totalTime && nowSec < actions[i].returnTime() + totalTime) {
                            m_robotDrive.drive(0.7, -1.0);
                            System.out.println("turning counter clockwise");
                        }
                    } else if (actions[i].type == ACTION_FORWARD)//then were going forward
                    {
                        if (nowSec > totalTime && nowSec < actions[i].returnTime() + totalTime) {
                            m_robotDrive.drive(0.5, 0.0);
                            System.out.println("drive forward");
                        }
                    } else if (actions[i].type == ACTION_BACKWARD)//then were going backwards
                    {
                        if (nowSec > totalTime && nowSec < actions[i].returnTime() + totalTime) {
                            m_robotDrive.drive(-0.5, 0.0);
                            System.out.println("drive backwards");
                        }
                    }
                    else if (actions[i].type == ACTION_NONE)//then were stopped
                    {
                        if (nowSec > totalTime) {
                            m_robotDrive.drive(0.0, 0.0);
                            System.out.println("stopped");
                        }
                    }
                }

                }
                else
                {
                    m_robotDrive.drive(0.0, 0.0);
                    System.out.println("too much tilt!");
                }
	}

        public double allPreviousTimes(int i)
        {
            Watchdog.getInstance().feed();
            double nowSec = (Timer.getUsClock() / 1000000.0);
            double time = autoSecGreen;
            for(int x = i-1;x>=0;x--)
            {
                time += actions[x].returnTime();
            }
            System.out.println("Action number: "+i+" -- All previous times :"+time+" -- Time now:"+nowSec +" -- Time for action:"+actions[i].returnTime());
            return time;
        }

        public void teleopContinuous()
        {
            Watchdog.getInstance().feed();
//            //cam here
//            if(m_rightStick.getRawButton(5))
//            {
//                m_robotDrive3.set(0.3);
//            }
//            else if(m_rightStick.getRawButton(4))
//            {
//                m_robotDrive3.set(-0.3);
//            }
//            else
//            {
//                m_robotDrive3.set(0.0);
//            }
////            if(m_rightStick.getRawButton(7))
////            {
////                raiseTheWinch();
////            }
//
//            if(m_rightStick.getRawButton(3) && m_rightStick.getTrigger())
//            {
//                m_robotDrive4.set(0.5);
//                m_robotDrive5.set(0.6);
//                firstMotorSpeed += 50;
//                secondMotorSpeed += 60;
//                motoRatio = (firstMotorSpeed/secondMotorSpeed);
//                System.out.println("First Motor Speed "+firstMotorSpeed);
//                System.out.println("Second Motor Speed "+secondMotorSpeed);
//                System.out.println("Ratio Motor Speeds "+motoRatio);
//            }
//            else if(m_rightStick.getRawButton(3) && !m_rightStick.getTrigger())
//            {
//                m_robotDrive4.set(0.5);
//                m_robotDrive5.set(0.5);
//                firstMotorSpeed += 50;
//                secondMotorSpeed += 50;
//                motoRatio = (firstMotorSpeed/secondMotorSpeed);
//                System.out.println("First Motor Speed "+firstMotorSpeed);
//                System.out.println("Second Motor Speed "+secondMotorSpeed);
//                System.out.println("Ratio Motor Speeds "+motoRatio);
//            }
//            else if(m_rightStick.getRawButton(2))
//            {
//                m_robotDrive4.set(-0.5);
//                m_robotDrive5.set(-0.5);
//            }
//            else
//            {
//                m_robotDrive4.set(0.0);
//                m_robotDrive5.set(0.0);
//            }
//
//            if(m_rightStick.getRawButton(7))
//            {
//                m_robotDrive4.set(0.5);
//            }
//            else if(m_rightStick.getRawButton(6))
//            {
//                m_robotDrive4.set(-0.5);
//            }
//            else
//            {
//                m_robotDrive4.set(0.0);
//            }
//
//            if(m_rightStick.getRawButton(11))
//            {
//                m_robotDrive5.set(0.5);
//            }
//            else if(m_rightStick.getRawButton(10))
//            {
//                m_robotDrive5.set(-0.5);
//            }
//            else
//            {
//                m_robotDrive5.set(0.0);
//            }

            if(m_leftStick.getRawButton(11))
            {
                m_robotDrive6.drive(0.8,0);
            }
            else
            {
                m_robotDrive6.drive(0.0,0);
            }

            if(m_rightStick.getTrigger())
            {
                m_robotDrive.arcadeDrive(m_leftStick);
            }
            else
            {
                m_robotDrive.arcadeDrive(m_leftStick.getAxis(Joystick.AxisType.kY)*.6, m_leftStick.getAxis(Joystick.AxisType.kX)*.7);
            }


        }
        
	public void teleopPeriodic() {
		// feed the user watchdog at every period when in autonomous
		Watchdog.getInstance().feed();

                camera.freshImage();
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;

		/*
		 * No longer needed since periodic loops are now synchronized with incoming packets.
		if (m_ds->GetPacketNumber() != m_priorPacketNumber) {
		*/
			/*
			 * Code placed in here will be called only when a new packet of information
			 * has been received by the Driver Station.  Any code which needs new information
			 * from the DS should go in here
			 */

			m_dsPacketsReceivedInCurrentSecond++;					// increment DS packets received

			// put Driver Station-dependent code here

                        
                        m_robotDrive2.set(m_rightStick.getY(Hand.kRight));
		/*
		}  // if (m_ds->GetPacketNumber()...
		*/

	}

	/**
	 * Clear KITT-style LED display on the solenoids
	 *
	 * Clear the solenoid LEDs used for a KITT-style LED display.
	 */
	public void ClearSolenoidLEDsKITT() {
		// Iterate over all the solenoids on the robot, clearing each in turn
		int solenoidNum = 1;						// start counting solenoids at solenoid 1
		for (solenoidNum = 0; solenoidNum < NUM_SOLENOIDS; solenoidNum++) {
			m_solenoids[solenoidNum].set(false);
		}
	}

	/**
	 * Generate KITT-style LED display on the solenoids
	 *
	 * This method expects to be called during each periodic loop, with the argument being the
	 * loop number for the current loop.
	 *
	 * The goal here is to generate a KITT-style LED display.  (See http://en.wikipedia.org/wiki/KITT )
	 * However, since the solenoid module has two scan bars, we can have ours go in opposite directions!
	 * The scan bar is written to have a period of one second with six different positions.
	 */
	public void SolenoidLEDsKITT(int numloops) {
		final int NUM_KITT_POSITIONS = 6;
		int numloop_within_second = numloops % GetLoopsPerSec();

		if (numloop_within_second == 0) {
			// position 1; solenoids 1 and 8 on
			m_solenoids[1].set(true);  m_solenoids[8].set(true);
			m_solenoids[2].set(false); m_solenoids[7].set(false);
		} else if (numloop_within_second == (GetLoopsPerSec() / NUM_KITT_POSITIONS)) {
			// position 2; solenoids 2 and 7 on
			m_solenoids[2].set(true);  m_solenoids[7].set(true);
			m_solenoids[1].set(false); m_solenoids[8].set(false);
		} else if (numloop_within_second == (GetLoopsPerSec() * 2 / NUM_KITT_POSITIONS)) {
			// position 3; solenoids 3 and 6 on
			m_solenoids[3].set(true);  m_solenoids[6].set(true);
			m_solenoids[2].set(false); m_solenoids[7].set(false);
		} else if (numloop_within_second == (GetLoopsPerSec() * 3 / NUM_KITT_POSITIONS)) {
			// position 4; solenoids 4 and 5 on
			m_solenoids[4].set(true);  m_solenoids[5].set(true);
			m_solenoids[3].set(false); m_solenoids[6].set(false);
		} else if (numloop_within_second == (GetLoopsPerSec() * 4 / NUM_KITT_POSITIONS)) {
			// position 5; solenoids 3 and 6 on
			m_solenoids[3].set(true);  m_solenoids[6].set(true);
			m_solenoids[4].set(false); m_solenoids[5].set(false);
		} else if (numloop_within_second == (GetLoopsPerSec() * 5 / NUM_KITT_POSITIONS)) {
			// position 6; solenoids 2 and 7 on
			m_solenoids[2].set(true);  m_solenoids[7].set(true);
			m_solenoids[3].set(false); m_solenoids[6].set(false);
		}
	}

        int GetLoopsPerSec() {
            return 10000;
        }

	/**
	 * Demonstrate handling of joystick buttons
	 *
	 * This method expects to be called during each periodic loop, providing the following
	 * capabilities:
	 * - Print out a message when a button is initially pressed
	 * - Solenoid LEDs light up according to joystick buttons:
	 *   - When no buttons pressed, clear the solenoid LEDs
	 *   - When only one button is pressed, show the button number (in binary) via the solenoid LEDs
	 *   - When more than one button is pressed, show "15" (in binary) via the solenoid LEDs
	 */
	public void DemonstrateJoystickButtons(Joystick currStick,
									boolean[] buttonPreviouslyPressed,
									String stickString,
									Solenoid solenoids[]) {

		int buttonNum = 1;				// start counting buttons at button 1
		boolean outputGenerated = false;		// flag for whether or not output is generated for a button
		int numOfButtonPressed = 0;		// 0 if no buttons pressed, -1 if multiple buttons pressed

		/* Iterate over all the buttons on the joystick, checking to see if each is pressed
		 * If a button is pressed, check to see if it is newly pressed; if so, print out a
		 * message on the console
		 */
		for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
			if (currStick.getRawButton(buttonNum)) {
				// the current button is pressed, now act accordingly...
				if (!buttonPreviouslyPressed[buttonNum]) {
					// button newly pressed; print out a message
					if (!outputGenerated) {
						// print out a heading if no other button pressed this cycle
						outputGenerated = true;
						System.out.println("%s button pressed:" + stickString);
					}
					System.out.println(" " + buttonNum);
				}
				// remember that this button is pressed for the next iteration
				buttonPreviouslyPressed[buttonNum] = true;

				// set numOfButtonPressed appropriately
				if (numOfButtonPressed == 0) {
					// no button pressed yet this time through, set the number correctly
					numOfButtonPressed = buttonNum;
				} else {
					// another button (or buttons) must have already been pressed, set appropriately
					numOfButtonPressed = -1;
				}
			} else {
				buttonPreviouslyPressed[buttonNum] = false;
			}
		}

		// after iterating through all the buttons, add a newline to output if needed
		if (outputGenerated) {
			System.out.println("\n");
		}

		if (numOfButtonPressed == -1) {
			// multiple buttons were pressed, display as if button 15 was pressed
			DisplayBinaryNumberOnSolenoidLEDs(15, solenoids);
		} else {
			// display the number of the button pressed on the solenoids;
			// note that if no button was pressed (0), the solenoid display will be cleared (set to 0)
			DisplayBinaryNumberOnSolenoidLEDs(numOfButtonPressed, solenoids);
		}
	}


	/**
	 * Display a given four-bit value in binary on the given solenoid LEDs
	 */
	void DisplayBinaryNumberOnSolenoidLEDs(int displayNumber, Solenoid[] solenoids) {

		if (displayNumber > 15) {
			// if the number to display is larger than can be displayed in 4 LEDs, display 0 instead
			displayNumber = 0;
		}

		solenoids[3].set( (displayNumber & 1) != 0);
		solenoids[2].set( (displayNumber & 2) != 0);
		solenoids[1].set( (displayNumber & 4) != 0);
		solenoids[0].set( (displayNumber & 8) != 0);
	}
}
