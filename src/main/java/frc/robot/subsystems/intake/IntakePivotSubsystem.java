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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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

  private SparkFlex m_pivotMotor = new SparkFlex(Constants.CANIDConstants.intake_pivot, MotorType.kBrushless);

  private SparkSim m_pivotMotorSim = new SparkSim(m_pivotMotor, DCMotor.getNeoVortex(1));
  private SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.IntakePivotConstants.k_gearRatio, Constants.IntakePivotConstants.k_momentOfInertia, Constants.IntakePivotConstants.k_armLengthMeters, 0, Math.toRadians(220), true, Math.toRadians(220));

  private AbsoluteEncoder m_encoder = m_pivotMotor.getAbsoluteEncoder();
  private double m_pivotSimAngle = 0;

  private IntakeState m_state = IntakeState.STOW;
  private PIDController m_pid = new PIDController(Constants.IntakePivotConstants.k_p, Constants.IntakePivotConstants.k_i, Constants.IntakePivotConstants.k_d);
  private double m_output = 0;

  public Trigger atPosition = new Trigger(
    () -> Math.abs(getAngle() - m_state.getAngle()) < Constants.IntakePivotConstants.k_deadzone);

  /** Creates a new IntakePivot. */
  public IntakePivotSubsystem() {
    m_pivotMotor.configure(Constants.IntakePivotConstants.pivotConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double getAngle() {
    if(RobotBase.isReal()){
      return m_encoder.getPosition();
    } else {
      return m_pivotSimAngle;
    }
  }

  public Command setPosition(IntakeState pos) {
    return Commands.runOnce(() -> {m_state = pos;}, this);
  }

  public IntakeState getSetPoint() {
    return m_state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_output = m_pid.calculate(getAngle(), m_state.getAngle()) + Constants.IntakePivotConstants.k_f * Math.cos(Math.toRadians(getAngle()));
    m_pivotMotor.set(m_output);
  }

  public void simulationPeriodic() {
    m_pivotSim.setInput(m_pivotMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_pivotSim.update(0.02);
    m_pivotMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(m_pivotSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_pivotSim.getCurrentDrawAmps()));
    m_pivotSimAngle = Math.toDegrees(m_pivotSim.getAngleRads());
  }
}
