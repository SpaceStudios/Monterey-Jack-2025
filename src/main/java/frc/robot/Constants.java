// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class Constants {

    // Drive train
    public class DrivetrainConstants {
        public static final Distance wheelRadius = Inches.of(3); // Wheel Radius, Units provided is in inches
        public static final double driveGearing = 1.0; // Gearing of the Drivetrain
        public static final LinearVelocity driveVelocity = MetersPerSecond.of(2); // Max Drive Velocity, Only use this if you are using the drive velocity with motors that have encoders
        public static final AngularVelocity driveTurnSpeed = RotationsPerSecond.of(1); // Max drive rotational speed
        public static final boolean useVolts = true; // Use volts for driving instead of chassis speeds if true else use chassis speeds if false
        public static final Voltage driveVoltage = Volts.of(6); // Drive Voltage 
        public static final int[][] driveIDs = new int[][] {
            {1,2},
            {3,4}
        }; // Shown in a differential drive configuration
    }

    // Launcher
    public class LauncherConstants {
        public static final double launchForce = 1.0; // Launch Force in percent, 0.0 = 0%, 1.0 = 100%
        public static final int leftLauncher = 0; // Left Launcher ID
        public static final int rightLauncher = 0; // Right Launcher ID
        public static final int feeder = 0; // Center Motor ID
        public static final int hoodID = 0; // Hood Pneumatics ID
    }

    // Intake
    public class IntakeConstants {
        public static final double intakeForce = 1.0; // Intake Force in percent, 0.0 = 0%, 1.0 = 100%
        public static final int leftIntakeID = 0;
        public static final int rightIntakeID = 0;
        public static final int centerIntakeID = 0;
    }

    // Robot Can IDs for stuff not used in any other subsystem
    public class CANIDs {
        public static final int pneumaticHub = 21;
    }
}
