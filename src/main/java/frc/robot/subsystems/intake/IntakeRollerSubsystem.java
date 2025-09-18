// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IntakeRollerConstants;

public class IntakeRollerSubsystem extends SubsystemBase {
  private SparkFlex m_rollerMotor = new SparkFlex(CANIDConstants.intake_roller, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_rollerMotor.getEncoder();
  private SparkClosedLoopController m_pid = m_rollerMotor.getClosedLoopController();

  public final Trigger hasCoral = new Trigger(
      () -> getCurrentDraw() < IntakeRollerConstants.k_stallCurrent && getSpeed() >= IntakeRollerConstants.k_inSpeed - IntakeRollerConstants.k_slowSpeed)
      .debounce(.25, DebounceType.kBoth);

  /** Creates a new RollerSubsystem. */
  public IntakeRollerSubsystem() {
    m_rollerMotor.configure(IntakeRollerConstants.rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getSpeed() {
    return m_encoder.getVelocity();
  }

  public double getCurrentDraw() {
    return m_rollerMotor.getAppliedOutput();
  }

  public Command run(boolean in) {
    return Commands.runOnce(
      () -> {m_pid.setReference(in ? IntakeRollerConstants.k_inSpeed : IntakeRollerConstants.k_outSpeed, ControlType.kVelocity);}, this);
  }

  public Command stop() {
    return Commands.runOnce(
      () -> {m_pid.setReference(0, ControlType.kVelocity);}, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Roller Speed", getSpeed());
  }
}
