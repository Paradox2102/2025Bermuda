// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.CageCatcherConstants;

public class CageCatchSubsystem extends SubsystemBase {
  private SparkFlex m_cageMotor = new SparkFlex(CANIDConstants.cage_grabber, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_cageMotor.getEncoder();
  private SparkClosedLoopController m_pid = m_cageMotor.getClosedLoopController();

  /** Creates a new CageCatchSubsystem. */
  public CageCatchSubsystem() {
    m_cageMotor.configure(CageCatcherConstants.cageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getSpeed() {
    return m_encoder.getVelocity();
  }

  public Command run() {
    return Commands.runOnce(
      () -> {m_pid.setReference(CageCatcherConstants.k_inSpeed, ControlType.kVelocity);}, this);
  }

  public Command stop() {
    return Commands.runOnce(
      () -> {m_pid.setReference(0, ControlType.kVelocity);}, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Cage Catcher Speed", getSpeed());
  }
}
