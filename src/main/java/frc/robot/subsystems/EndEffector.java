// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

    public enum IntakeVelocity {
        FAST_INTAKE(100),
        SLOW_INTAKE(50),
        FAST_OUTPUT(-100),
        SLOW_OUTPUT(-50);

        public final double velocity;

        IntakeVelocity(double velocity) {
            this.velocity = velocity;
        }
    }

    private final TalonFX motor = new TalonFX(30);
    private final DigitalInput CoralSensor = new DigitalInput(1);
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public EndEffector() {
        CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimit = 30;

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    }

    public double getEndEffectorPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public void setSpeed(double velocity) {
        motor.setControl(velocityVoltage.withVelocity(velocity));
    }

    public void setSpeed(IntakeVelocity velocity) {
        setSpeed(velocity.velocity);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean hasCoral() {
        return !CoralSensor.get();
    }

    @Override
    public void periodic() {
    }
}
