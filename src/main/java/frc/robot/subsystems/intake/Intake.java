// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.io.IntakeDataAutoLogged;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIO_REAL;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  PneumaticHub pneumaticHub;
  IntakeIO io;
  IntakeDataAutoLogged data;

  public Intake(PneumaticHub pneumaticHub) {
    this.pneumaticHub = pneumaticHub;
    if (Robot.isReal()) {
      io = new IntakeIO_REAL(pneumaticHub); 
    }
  }

  public void setPosition(boolean position) { // If true it is down, else it is up
    io.setIntakePosition(position);
  }

  public void setVolts(double volts) {
    io.setVolts(volts);
  }

  public boolean getPosition() {
    return data.IntakePosition;
  }

  @Override
  public void periodic() {
    io.getData(data);
    Logger.processInputs("Intake", data);
  }
}
