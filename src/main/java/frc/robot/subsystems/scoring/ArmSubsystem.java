// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.scoring.ElevatorSubsystem.ElevatorState;

public class ArmSubsystem extends SubsystemBase {
  private SparkFlex m_armMotor = new SparkFlex(Constants.CANIDConstants.arm, MotorType.kBrushless);

  private SparkSim m_armMotorSim = new SparkSim(m_armMotor, DCMotor.getNeoVortex(1));
  private SingleJointedArmSim m_armSim = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.ArmConstants.k_gearRatio, Constants.ArmConstants.k_momentOfInertia, Constants.ArmConstants.k_armLengthMeters, Math.toRadians(-270), Math.toRadians(90), true, Math.toRadians(-90));

  private AbsoluteEncoder m_encoder = m_armMotor.getAbsoluteEncoder();
  private double m_armSimAngle = 0;

  private ElevatorState m_state;
  private PIDController m_pid = new PIDController(Constants.ArmConstants.k_p, Constants.ArmConstants.k_i, Constants.ArmConstants.k_d);
  private double m_output = 0;

  public Trigger atPosition = new Trigger(
    () -> Math.abs(getAngle() - m_state.getAngle()) < Constants.ArmConstants.k_deadzone);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ElevatorState state) {
    m_state = state;
  }

  public double getAngle() {
    if(RobotBase.isReal()){
      return m_encoder.getPosition();
    } else {
      return m_armSimAngle;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_output = m_pid.calculate(getAngle(), m_state.getAngle()) + Constants.ArmConstants.k_f * Math.cos(Math.toRadians(getAngle()));
    m_armMotor.set(m_output);
  }

  public void simulationPeriodic() {
    m_armSim.setInput(m_armMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_armSim.update(0.02);
    m_armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    m_armSimAngle = Math.toDegrees(m_armSim.getAngleRads());
  }
}
