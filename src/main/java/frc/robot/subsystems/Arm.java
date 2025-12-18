// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  public enum Positions {
    Home(0),
    Intake(0),
    Climb(0),
    ClimbPrep(0),
    ScoreCoral(0),
    Dealgae(0), Shoot;

    public final double rotations;

    Positions(double rotations) {
      this.rotations = rotations;
    }

  }

  private final TalonFX motorLeft = new TalonFX(20);
  private final TalonFX motorRight = new TalonFX(21);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  public Arm() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = 0;
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = false;
    softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = 0;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = false;

    CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.SupplyCurrentLimitEnable = true;
    currentLimitConfigs.SupplyCurrentLimit = 50;

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    Slot0Configs slot0Configs = new Slot0Configs();

    talonFXConfiguration.SoftwareLimitSwitch = softwareLimitSwitchConfigs;
    talonFXConfiguration.Slot0 = slot0Configs;
    talonFXConfiguration.MotionMagic = motionMagicConfigs;
    talonFXConfiguration.CurrentLimits = currentLimitConfigs;
    talonFXConfiguration.MotorOutput = motorOutputConfigs;

    motorLeft.getConfigurator().apply(talonFXConfiguration);
    motorRight.getConfigurator().apply(talonFXConfiguration);

  }

  public double getArmPosition() {
    return motorLeft.getPosition().getValueAsDouble();
  }

  public double getArmVelocity() {
    return motorLeft.getVelocity().getValueAsDouble();
  }

  public void setPosition(double position) {
    motionMagicRequest.Position = position;
    motorLeft.setControl(motionMagicRequest);
    motorRight.setControl(motionMagicRequest);
  }

  public void setSpeed(double speed) {
    motorLeft.set(speed);
    motorRight.set(speed);
  }

  public void stop() {
    motorLeft.stopMotor();
    motorRight.stopMotor();

  }

  public void resetMotorPosition(double position) {
    motorLeft.setPosition(position);
    motorRight.setPosition(position);
  }

  public boolean isNearPositionAndTolerance(double position, double tolerance) {
    return MathUtil.isNear(position, getArmPosition(), tolerance);
  }

  public Command armCommand() {
    return new Command() {

    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
