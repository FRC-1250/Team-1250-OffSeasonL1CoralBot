// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  private final TalonFX motor = new TalonFX(30);
  private final DigitalInput CoralSensor = new DigitalInput(1);
   private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
   private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
   private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  public EndEffector() {
    CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.SupplyCurrentLimitEnable = true;
    currentLimitConfigs.SupplyCurrentLimit = 30;
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
     MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    
}
public double getEndEffectorPosition() {
    return motor.getPosition().getValueAsDouble();
  }
  public void setSpeed(double speed) {
    motor.set(speed);
  }
  public void setPosition(double position) {
    motionMagicRequest.Position = position;
    motor.setControl(motionMagicRequest);
  }
  public void stop() {
    motor.stopMotor();
  }
  public boolean hasCoral() {
    return !CoralSensor.get();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
