// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class IntakeIO_REAL implements IntakeIO {
    PneumaticHub pneumaticHub;

    Solenoid intakePiston;

    TalonSRX leftSRX;
    TalonSRX rightSRX;
    TalonSRX centerSRX;

    public IntakeIO_REAL(PneumaticHub pneumaticHub) {
        this.pneumaticHub = pneumaticHub;

        intakePiston = pneumaticHub.makeSolenoid(intakeSolenoid);

        leftSRX = new TalonSRX(leftIntakeID);
        rightSRX = new TalonSRX(rightIntakeID);
        centerSRX = new TalonSRX(centerIntakeID);

        // Resetting Settings
        rightSRX.configFactoryDefault();
        leftSRX.configFactoryDefault();
        centerSRX.configFactoryDefault();

        //Setting current limits
        rightSRX.configContinuousCurrentLimit(40);
        leftSRX.configContinuousCurrentLimit(40);
        centerSRX.configContinuousCurrentLimit(40);

        rightSRX.follow(centerSRX);
        leftSRX.follow(centerSRX);
    }

    @Override
    public void setIntakePosition(boolean position) {
        intakePiston.set(position);
    }

    @Override
    public void setVolts(double volts) {
        centerSRX.set(ControlMode.PercentOutput, volts/12);
    }

    @Override
    public void getData(IntakeDataAutoLogged data) {
        data.IntakeCurrent = Amps.of(centerSRX.getStatorCurrent());
        data.IntakeTemperature = Celsius.of(centerSRX.getTemperature());
        data.IntakeVelocity = RotationsPerSecond.zero();
        data.IntakeVolts = Volts.of(centerSRX.getMotorOutputVoltage());
        
        data.IntakePosition = intakePiston.get();
    }
}
