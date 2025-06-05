// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Channels;
import frc.robot.Constants.pivot;

public class PivotSubsystem extends SubsystemBase {
  private TalonFX pivotMotor;
  private PIDController pidController; 
  private DutyCycleEncoder pivotEncoder;

  /** Creates a new ArmSubsystem. */
  public PivotSubsystem() {

    pivotMotor = new TalonFX(pivot.PivotMotorID);
    pidController = new PIDController(pivot.pivotP, pivot.pivotI, pivot.pivotD);
    //pivotEncoder = new DutyCycleEncoder(Channels.pivotEncoderChannel);

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  public void runPivot(double Velocity){
  

    if (getAngle() <= pivot.pivotThreshold && Velocity < 0) {
      pivotMotor.set(0);
    } else if (getAngle() >= pivot.upperPivotThreshold && Velocity > 0){
      pivotMotor.set(0);
    }
     else {
      pivotMotor.set(Velocity);
    }

   
  }

  public void clearPID() {
    pidController = new PIDController(pivot.pivotP, pivot.pivotI, pivot.pivotD);
    pidController.reset();
  }

  public double getPID(double deg) {
    pidController.setSetpoint(deg);
    return pidController.calculate(getAngle());
  }

  public double getAngle() {
    return (pivotEncoder.get() - pivot.pivotOffset) * 360;
  }

  public void stop() {
    pivotMotor.set(0);
  }

  public void runToPosition(double deg)
  {
     
    double out = getPID(deg);

    runPivot(out);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Encoder Value:", getAngle());
  }
}
