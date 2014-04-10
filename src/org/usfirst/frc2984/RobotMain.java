/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc2984;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends SimpleRobot {

	public Drivetrain drivetrain;
	private Joystick joystick1;
	private double xHigh, xLow, yHigh, yLow;
	public Pneumatics pneumatics;
	public NetworkTable table;
	private Joystick joystick2;
	private DriverStation ds;
	public final static double JOYSTICK_SENSITIVITY = .7;
	public final static double MAX_DRIVE_SPEED = .8;
	public final static double MAX_TURN_SPEED = 1;
	public final static double MAX_WIND_SPEED = .8;
	public final static double GRABBER_SPEED = 1.0;

	public void robotInit() {

		drivetrain = new Drivetrain(this);
		pneumatics = new Pneumatics(this);

		xHigh = 200;
		xLow = -200;
		yHigh = 200;
		yLow = -200;

		joystick1 = new Joystick(1);
		joystick2 = new Joystick(2);

		table = NetworkTable.getTable("CustomData1");

		ds = DriverStation.getInstance();

	}

	public void disabled() {
		table.putNumber("axis1", 0);
		table.putNumber("axis2", 0);
		table.putNumber("yaw", 0);
		table.putNumber("winder", 0);
		table.putNumber("grabber", 0);
		table.putBoolean("fire", false);
		table.putBoolean("load", false);
		table.putBoolean("feederArm", false);
		table.putNumber("time", 0);

		drivetrain.stopLoad();
		pneumatics.extendArm(false);

		while (this.isDisabled())
			table.putNumber("time", table.getNumber("time") + 1);

	}

	public void operatorControl() {

		drivetrain.setSafety(true);
		

		while (isOperatorControl() && isEnabled()) {

			double x, y, turn;

			ds.setDigitalOut(1, drivetrain.catapultLimit.get());
			ds.setDigitalOut(2, drivetrain.catapultLimit2.get());

			if (!ds.getDigitalIn(1)) {
				drivetrain.setWinder(joystick1.getRawAxis(2));
				if (joystick1.getRawButton(6))
					drivetrain.load(null);
				pneumatics.extendArm(!joystick1.getRawButton(1));
				if (joystick1.getRawButton(8))
					pneumatics.fire(null);

				if (joystick1.getRawButton(5))
					drivetrain.setGrabber(GRABBER_SPEED);
				else if (joystick1.getRawButton(7))
					drivetrain.setGrabber(-GRABBER_SPEED);
				else
					drivetrain.setGrabber(0);
			} else {
				drivetrain
						.setWinder(table.getNumber("winder") * MAX_WIND_SPEED);
				if (table.getBoolean("load")) {
					table.putBoolean("load", false);
					drivetrain.load(null);
				}

				pneumatics.extendArm(table.getBoolean("feederArm"));

				if (table.getBoolean("fire")) {
					table.putBoolean("fire", false);
					pneumatics.fire(null);
				}

				drivetrain.setGrabber(table.getNumber("grabber")
						* GRABBER_SPEED);
			}

			drivetrain.updateLimitData();

			if (!ds.getDigitalIn(2)) {

				y = joystick2.getRawAxis(2);
				x = joystick2.getRawAxis(1);
				turn = joystick2.getRawAxis(3);
			} else {

				y = table.getNumber("axis1");
				x = table.getNumber("axis2");
				turn = table.getNumber("yaw");

				xHigh = Math.max(x, xHigh);
				xLow = Math.min(x, xLow);

				yHigh = Math.max(y, yHigh);
				yLow = Math.min(y, yLow);

				if (x > 0)
					x /= xHigh;
				else
					x /= -xLow;

				if (y > 0)
					y /= yHigh;
				else
					y /= -yLow;

			}

			/*
			x = x > MAX_DRIVE_SPEED ? MAX_DRIVE_SPEED : x;
			y = y > MAX_DRIVE_SPEED ? MAX_DRIVE_SPEED : y;
			*/

			drivetrain
					.driveMec(regression(x), regression(-y), turn);

			table.putNumber("time", table.getNumber("time") + 1);

			Timer.delay(.01);
		}
	}

	public void autonomous() {

		drivetrain.setSafety(false);
		pneumatics.extendArm(true);
		
		drivetrain.load(null);

		drivetrain.driveMec(0, -0.5, 0);
		Timer.delay(1.2);
		drivetrain.driveMec(0, 0, 0);
		
		Timer.delay(3);
		pneumatics.fire(new Runnable() {
			public void run() {
				synchronized (RobotMain.this) {
					RobotMain.this.notifyAll();
				}
			}
		});
		
		/*

		try {
			synchronized (this) {
				this.wait();
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		drivetrain.load(new Runnable() {
			public void run() {
				synchronized (RobotMain.this) {
					RobotMain.this.notifyAll();
				}
			}
		});
		
		/*
		
		try {
			synchronized (this) {
				this.wait();
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		

		drivetrain.setGrabber(1.0);
		drivetrain.driveMec(0, .8, 0);
		Timer.delay(1.3);
		drivetrain.driveMec(0, 0, 0);

		drivetrain.driveMec(0, -.8, 0);
		Timer.delay(1.3);
		drivetrain.setGrabber(0);
		drivetrain.driveMec(0, 0, 0);
		Timer.delay(2);
		pneumatics.fire(null);*/
		

	}

	public static double regression(double d) {
		return JOYSTICK_SENSITIVITY * (d * d * d) + (1 - JOYSTICK_SENSITIVITY)
				* d;
	}
}