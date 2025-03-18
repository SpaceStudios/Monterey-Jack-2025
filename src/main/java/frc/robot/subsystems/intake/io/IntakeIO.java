// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public class IntakeData {
        public Current IntakeCurrent;
        public Temperature IntakeTemperature;
        public Voltage IntakeVolts;
        public AngularVelocity IntakeVelocity;

        public boolean IntakePosition = false;
    }

    public abstract void setIntakePosition(boolean position);
    public abstract void setVolts(double volts);
    public abstract void getData(IntakeDataAutoLogged data);
}
