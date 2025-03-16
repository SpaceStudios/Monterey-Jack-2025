// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface DrivetrainIO {
    @AutoLog
    public class DriveData {
        public Voltage leftDriveVoltage = Volts.zero(); // Left Drive Voltage;
        public Current leftDriveCurrent = Amps.zero(); // Left Drive Current
        public AngularVelocity leftDriveVelocity = RadiansPerSecond.zero(); // Left Drive Velocity
        public Temperature leftDriveTemperature = Celsius.zero(); // Left Drive Temperature

        public Voltage rightDriveVoltage = Volts.zero(); // Right Drive Voltage
        public Current rightDriveCurrent = Amps.zero(); // Right Drive Current
        public AngularVelocity rightDriveVelocity = RadiansPerSecond.zero(); // Right Drive Velocity
        public Temperature rightDriveTemperature = Celsius.zero(); // Right Drive Temperature

        public Pose2d estimatedPose;
    }

    public abstract void driveVolts(double lVolts, double rVolts); // Drive based on volts
    public abstract void driveSpeed(LinearVelocity lVelocity, LinearVelocity rVelocity); // Drive Velocity based on WPILIB units class
    public abstract void driveSpeed(double lVelocity, double rVelocity); // Drive Velocity in Meters per a second
    public abstract void driveSpeed(DifferentialDriveWheelSpeeds speeds); // Drive based on Differential Drive Wheel Speeds
    public abstract void getData(DriveDataAutoLogged data); // Get Data from robot
}
