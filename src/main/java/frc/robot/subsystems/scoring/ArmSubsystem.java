// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  public enum ArmState {
    STOW(0),
    HANDOFF(0),
    L1(0),
    L2(0),
    L3(0),
    L4(0),
    GROUND_ALGAE(0),
    ALGAE_LOW(0),
    ALGAE_HIGH(0),
    PROCESSOR(0),
    NET(0),
    LOLLIPOP(0);

    private double m_angle;
    
    ArmState(double angle) {
      m_angle = angle;
    }

    public double getAngle() {
      return m_angle;
    }
  }

  private SparkFlex m_armMotor = new SparkFlex(Constants.CANIDConstants.arm, MotorType.kBrushless);

  private SparkSim m_armMotorSim = new SparkSim(m_armMotor, DCMotor.getNeoVortex(1));
  private SingleJointedArmSim m_armSim = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.ArmConstants.k_gearRatio, Constants.ArmConstants.k_momentOfInertia, Constants.ArmConstants.k_armLengthMeters, Math.toRadians(-270), Math.toRadians(90), true, Math.toRadians(-90));

  private AbsoluteEncoder m_encoder = m_armMotor.getAbsoluteEncoder();
  private double m_armSimAngle = 0;

  private ArmState m_state = ArmState.STOW;
  private PIDController m_pid = new PIDController(Constants.ArmConstants.k_p, Constants.ArmConstants.k_i, Constants.ArmConstants.k_d);
  private double m_output = 0;

  private boolean m_invert = false;
  private double m_setPoint = m_state.getAngle();

  public Trigger atPosition = new Trigger(
    () -> Math.abs(getAngle() - m_setPoint) < Constants.ArmConstants.k_deadzone);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor.configure(ArmConstants.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getAngle() {
    if(RobotBase.isReal()){
      return m_encoder.getPosition();
    } else {
      return m_armSimAngle;
    }
  }

  public Command setPosition(ArmState pos) {
    return Commands.runOnce(() -> {
      m_state = pos;
      m_setPoint = m_state.getAngle();
    }, this).until(atPosition);
  }

  public Command scoreReef() {
    return Commands.runOnce(() -> {
      m_setPoint = m_state.getAngle() - ArmConstants.k_dunkAngle;
    }, this);
  }

  public Command switchSides(boolean invert) {
    return Commands.runOnce(() -> {
      m_invert = invert;
    }, this);
  }

  public Command reset() {
    return Commands.runOnce(() -> {m_state = ArmState.STOW;}, this);
  }

  public double invertPos(double angle) {
    double normAngle = -(angle - 90);
    return (2 * normAngle - 360) + 90;
  }

  public double getSetPoint() {
    return m_invert ? invertPos(m_setPoint) : m_setPoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_output = m_pid.calculate(getAngle(), m_invert ? invertPos(m_setPoint) : m_setPoint) + Constants.ArmConstants.k_f * Math.cos(Math.toRadians(getAngle()));
    m_armMotor.set(m_output);
    SmartDashboard.putNumber("Arm Angle", getAngle());
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
