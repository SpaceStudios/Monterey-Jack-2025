// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  PneumaticHub pneumaticHub;
  public Intake(PneumaticHub pneumaticHub) {
    this.pneumaticHub = pneumaticHub;
  }

  public void setIntakePosition(boolean position) { // If true it is down, else it is up

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
