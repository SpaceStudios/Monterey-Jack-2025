// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;

/** Add your docs here. */
public class DrivetrainIO_SparkMax implements DrivetrainIO {
    SparkMax frontLeftSpark;
    SparkMax rearLeftSpark;
    SparkMax frontRightSpark;
    SparkMax rearRightSpark;

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;
    
    DifferentialDrivePoseEstimator poseEstimator;
    DifferentialDriveKinematics kinematics;

    public DrivetrainIO_SparkMax() {
        frontLeftSpark = new SparkMax(driveIDs[0][0], MotorType.kBrushless);
        frontRightSpark = new SparkMax(driveIDs[0][1], MotorType.kBrushless);
        rearLeftSpark = new SparkMax(driveIDs[1][0], MotorType.kBrushless);
        rearRightSpark = new SparkMax(driveIDs[1][1], MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.smartCurrentLimit(40);
        
        frontLeftSpark.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontRightSpark.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rearLeftConfig = new SparkMaxConfig();
        rearLeftConfig.follow(frontLeftSpark);
        rearLeftConfig.smartCurrentLimit(40);

        SparkMaxConfig rearRightConfig = new SparkMaxConfig();
        rearRightConfig.follow(frontLeftSpark);
        rearRightConfig.smartCurrentLimit(40);

        rearLeftSpark.configure(rearLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rearRightSpark.configure(rearRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftEncoder = frontLeftSpark.getEncoder();
        rightEncoder = frontRightSpark.getEncoder();
        kinematics = new DifferentialDriveKinematics(0.5);
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0, 0, new Pose2d());
    }

    @Override
    public void driveVolts(double lVolts, double rVolts) {
        frontLeftSpark.setVoltage(lVolts);
        frontRightSpark.setVoltage(rVolts);
    }

    @Override
    public void driveSpeed(LinearVelocity lVelocity, LinearVelocity rVelocity) {
        // To be done once i find gearing
    }

    @Override
    public void driveSpeed(double lVelocity, double rVelocity) {
        // to be done
    }

    @Override
    public void driveSpeed(DifferentialDriveWheelSpeeds speeds) {
        // to be done
    }

    @Override
    public void getData(DriveDataAutoLogged data) {
        data.leftDriveCurrent = Amps.of(frontLeftSpark.getOutputCurrent());
        data.leftDriveTemperature = Celsius.of(frontLeftSpark.getMotorTemperature());
        data.leftDriveVelocity = RPM.of(leftEncoder.getVelocity());
        data.leftDriveVoltage = Volts.of(frontLeftSpark.getBusVoltage()*frontLeftSpark.getAppliedOutput());

        data.rightDriveCurrent = Amps.of(frontRightSpark.getOutputCurrent());
        data.rightDriveTemperature = Celsius.of(frontRightSpark.getMotorTemperature());
        data.rightDriveVelocity = RPM.of(rightEncoder.getVelocity());
        data.rightDriveVoltage = Volts.of(frontRightSpark.getBusVoltage()*frontRightSpark.getAppliedOutput());

        data.estimatedPose = poseEstimator.getEstimatedPosition();
    }
}
