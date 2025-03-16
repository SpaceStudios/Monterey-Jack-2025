// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.io.DriveDataAutoLogged;
import frc.robot.subsystems.drivetrain.io.DrivetrainIO;
import frc.robot.subsystems.drivetrain.io.DrivetrainIO_SparkMax;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  DriveDataAutoLogged data;
  DrivetrainIO io;
  public Drivetrain() {
    data = new DriveDataAutoLogged();
    if (RobotBase.isReal()) {
      io = new DrivetrainIO_SparkMax();
    }
  }

  public void drive(double drive, double steer) {
    if (useVolts) {
      WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(drive, steer, false);
      io.driveVolts(speeds.left*driveVoltage.in(Volts), speeds.right*driveVoltage.in(Volts));
    } else {
      
    }
  }

  @Override
  public void periodic() {
    io.getData(data);
    Logger.processInputs("Drivetrain", data);
  }
}
