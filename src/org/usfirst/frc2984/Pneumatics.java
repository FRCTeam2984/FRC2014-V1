package org.usfirst.frc2984;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics {

	private static final int FEEDER_SOLENOID_EXTEND = 1;
	private static final int FEEDER_SOLENOID_RETRACT = 2;
	private static final int LAUNCHER_SOLENOID = 3;

	private Solenoid feederExtend, feederRetract, launcher;
	private Compressor comp;
	private boolean firing;
	private RobotMain robotMain;

	public Pneumatics(RobotMain robotMain) {

		this.robotMain = robotMain;

		comp = new Compressor(1, 1);
		comp.start();

		feederExtend = new Solenoid(FEEDER_SOLENOID_EXTEND);
		feederRetract = new Solenoid(FEEDER_SOLENOID_RETRACT);

		launcher = new Solenoid(LAUNCHER_SOLENOID);
	}

	public void extendArm(boolean b) {
		feederRetract.set(!b);
		feederExtend.set(b);
	}

	private void setDisengaged(boolean b) {
		launcher.set(b);
	}

	public void fire(final Runnable runnable) {

		if (firing)
			return;

		firing = true;
		setDisengaged(true);

		System.out.println((System.currentTimeMillis() / 1000) + " Firing!");

		new Thread() {

			public void run() {
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				setDisengaged(false);
				firing = false;

				if (runnable != null)
					runnable.run();

			}

		}.start();

	}

}
