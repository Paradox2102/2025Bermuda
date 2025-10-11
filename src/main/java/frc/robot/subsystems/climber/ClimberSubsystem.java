// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  public enum ClimberState {
    STOW(0),
    EXTEND(90),
    CLIMB(45);

    private double m_angle;
    
    ClimberState(double angle) {
      m_angle = angle;
    }

    public double getAngle() {
      return m_angle;
    }
  }

  private SparkFlex m_climberMotor = new SparkFlex(CANIDConstants.climber, MotorType.kBrushless);
  private SparkFlex m_climberFollow = new SparkFlex(CANIDConstants.climber_follow, MotorType.kBrushless);

  private RelativeEncoder m_encoder = m_climberMotor.getEncoder();
  private SparkClosedLoopController m_pid = m_climberMotor.getClosedLoopController();

  private ClimberState m_state = ClimberState.STOW;

  public Trigger atPosition = new Trigger(() ->
    Math.abs(getPosition() - m_state.getAngle()) < ClimberConstants.k_deadzone);
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberMotor.configure(ClimberConstants.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_climberFollow.configure(ClimberConstants.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public Command setPosition(ClimberState state) {
    return Commands.run( () -> {
    m_state = state;
    m_pid.setReference(m_state.getAngle(), ControlType.kPosition);}, this).until(atPosition);
  }

  public Command reset() {
    return Commands.runOnce( () -> {
    m_state = ClimberState.STOW;
    m_pid.setReference(m_state.getAngle(), ControlType.kPosition);}, this);
  }

  public Command runOut(boolean in) {
    return Commands.runEnd(() ->
    m_pid.setReference(in ? 0.1 : -0.1, ControlType.kDutyCycle), 
    () -> m_pid.setReference(0, ControlType.kDutyCycle)
     ,this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Angle", getPosition());
  }
}
