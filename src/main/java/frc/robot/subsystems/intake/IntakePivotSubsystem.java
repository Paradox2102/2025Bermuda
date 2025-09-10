// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IntakePivotSubsystem extends SubsystemBase {
  public enum IntakeState {
    STOW(220),
    HANDOFF(220),
    INTAKE(0),
    L1(120);

    private double m_angle;
    
    IntakeState(double angle) {
      m_angle = angle;
    }

    public double getAngle() {
      return m_angle;
    }
  }

  private SparkFlex m_pivotMotor = new SparkFlex(Constants.canIDConstants.intake_pivot, MotorType.kBrushless);

  private SparkSim m_pivotMotorSim = new SparkSim(m_pivotMotor, DCMotor.getNeoVortex(1));
  private SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.IntakePivotConstants.k_gearRatio, Constants.IntakePivotConstants.k_momentOfInertia, Constants.IntakePivotConstants.k_armLengthMeters, 0, Math.toRadians(220), true, Math.toRadians(220));

  private AbsoluteEncoder m_encoder = m_pivotMotor.getAbsoluteEncoder();

  private IntakeState m_state = IntakeState.STOW;
  private PIDController m_pid = new PIDController(Constants.IntakePivotConstants.k_p, Constants.IntakePivotConstants.k_i, Constants.IntakePivotConstants.k_d);

  public Trigger atPosition = new Trigger(
    () -> Math.abs(getAngle() - m_state.getAngle()) < Constants.IntakePivotConstants.k_deadzone);

  /** Creates a new IntakePivot. */
  public IntakePivotSubsystem() {
    m_pivotMotor.configure(Constants.IntakePivotConstants.pivotConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

  public Command setPosition(IntakeState pos) {
    return Commands.runOnce(() -> {m_state = pos;}, this);
  }

  public IntakeState getPosition() {
    return m_state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pivotMotor.set(m_pid.calculate(getAngle(), m_state.getAngle()) + Constants.IntakePivotConstants.k_f * Math.cos(Math.toRadians(getAngle())));
  }

  public void simulationPeriodic() {

  }
}
