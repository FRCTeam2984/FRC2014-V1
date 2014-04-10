/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2984;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;

/**
 * 
 * @author The Francesco, Gabby Person
 */
public class Drivetrain {

	private Jaguar left1, left2, right1, right2;
	private RobotDrive rd;
	private Jaguar wind1;
	private Jaguar wind2;
	public DigitalInput catapultLimit, catapultLimit2;
	// public DigitalInput armUp;
	private final static int FRONT_RIGHT = 4;
	private final static int FRONT_LEFT = 7;
	private final static int BACK_RIGHT = 3;
	private final static int BACK_LEFT = 8;
	protected static final double AUTO_WIND_SPEED = 1;

	private boolean winding;
	private Jaguar grabber;
	private RobotMain robotMain;

	public Drivetrain(RobotMain robotMain) {

		this.robotMain = robotMain;

		winding = false;

		catapultLimit = new DigitalInput(2);
		catapultLimit2 = new DigitalInput(4);
		// armUp = new DigitalInput(5);

		left1 = new Jaguar(FRONT_LEFT);
		left2 = new Jaguar(BACK_LEFT);
		right1 = new Jaguar(FRONT_RIGHT);
		right2 = new Jaguar(BACK_RIGHT);

		wind1 = new Jaguar(1);
		wind2 = new Jaguar(2);

		grabber = new Jaguar(9);

		rd = new RobotDrive(left1, left2, right1, right2);
		rd.setInvertedMotor(MotorType.kRearLeft, true);
		rd.setInvertedMotor(MotorType.kFrontLeft, true);

	}

	public void driveMec(double x, double y, double turn) {

		rd.mecanumDrive_Cartesian(x, y, turn, 0);
	}

	public void setWinder(double d) {

		if (winding) {
			if (d != 0.0)
				winding = false;
			return;
		}

		if ((catapultLimit.get() || !catapultLimit2.get()) && d > 0) {
			wind1.set(0);
			wind2.set(0);
			return;
		}

		if ((catapultLimit.get() || !catapultLimit2.get()) && wind1.get() > 0) {
			wind1.set(0);
			wind2.set(0);
			return;
		}

		wind1.set(d);
		wind2.set(d);

	}

	public void load(final Runnable runnable) {

		if ((catapultLimit.get() || !catapultLimit2.get()) || winding)
			return;

		winding = true;

		robotMain.table.putBoolean("loading", true);

		System.out.println((System.currentTimeMillis() / 1000) + " Loading...");

		new Thread() {

			public void run() {

				wind1.set(AUTO_WIND_SPEED);
				wind2.set(AUTO_WIND_SPEED);

				while (!(catapultLimit.get() || !catapultLimit2.get())
						&& winding)
					Timer.delay(.01);

				wind1.set(0);
				wind2.set(0);

				winding = false;

				robotMain.table.putBoolean("loading", false);

				if (runnable != null)
					runnable.run();

			}

		}.start();
	}

	public void setGrabber(double d) {
		grabber.set(d);
	}

	public void updateLimitData() {
		robotMain.table.putBoolean("loaded",
				(catapultLimit.get() || !catapultLimit2.get()));
	}

	public void stopLoad() {
		winding = false;
	}

	public void setSafety(boolean b) {
		rd.setSafetyEnabled(b);
	}

}