// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;


import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Conversions;

// Add setspeed method or create PidController for driving TalonFX Motor (flare 2024 code will help)

public class MAXSwerveModule {
  public final TalonFX m_drivingTalon;
  public final SparkMax m_turningSpark;
  public final String id;

  private final AbsoluteEncoder m_turningEncoder;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.kS, ModuleConstants.kV,
      ModuleConstants.kA);

  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, String i, double offset) {
    m_drivingTalon = new TalonFX(drivingCANId, "FRC 1599B");
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
    id = i;
    m_chassisAngularOffset = offset;

    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(0);
  }

  public double getAngle() {
    return ((m_turningEncoder.getPosition() - m_chassisAngularOffset) * 360) * (Math.PI / 180);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingTalon.getVelocity().getValueAsDouble(),
        new Rotation2d(getAngle()));
  }

  public double getTurn()
  {
    return m_turningEncoder.getPosition();
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
      m_drivingTalon.get(),
        new Rotation2d(getAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SmartDashboard.putString(id + "des state ", desiredState.toString());

    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond; 
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    setSpeed(desiredState);

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SmartDashboard.putString(id + " pre ", correctedDesiredState.toString());
    correctedDesiredState.optimize(new Rotation2d(getAngle()));
    SmartDashboard.putString(id + " post ", correctedDesiredState.toString());

    // Command driving and turning SPARKS towards their respective setpoints.
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalon.setPosition(0);
  }

  private void setSpeed(SwerveModuleState desiredState) {
    double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.kMaxSpeedMetersPerSecond;
    m_drivingTalon.set(percentOutput);
  }
}