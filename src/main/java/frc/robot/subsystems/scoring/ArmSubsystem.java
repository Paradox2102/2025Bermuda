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
    STOW(80),
    HANDOFF(-90),
    L1(-10),
    L2(35),
    L3(35),
    L4(35),
    GROUND_ALGAE(-11),
    ALGAE_LOW(0),
    ALGAE_HIGH(0),
    PROCESSOR(0),
    NET(60),
    LOLLIPOP(-10);

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

  private ArmState m_state = ArmState.STOW;
  private SingleJointedArmSim m_armSim = new SingleJointedArmSim(DCMotor.getNeoVortex(1), ArmConstants.k_gearRatio, ArmConstants.k_momentOfInertia, ArmConstants.k_armLengthMeters, Math.toRadians(-270), Math.toRadians(90), true, Math.toRadians(m_state.getAngle()));

  private AbsoluteEncoder m_encoder;
  private double m_armSimAngle = 0;

  private PIDController m_pid = new PIDController(ArmConstants.k_p, ArmConstants.k_i, ArmConstants.k_d);
  private double m_output = 0;

  private boolean m_invert = false;
  private double m_setPoint = m_state.getAngle();

  public Trigger atPosition = new Trigger(
    () -> Math.abs(getAngle() - m_setPoint) < ArmConstants.k_deadzone);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor.configure(ArmConstants.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_pid.setIZone(ArmConstants.k_izone);
    m_encoder = m_armMotor.getAbsoluteEncoder();
    m_pid.enableContinuousInput(-270, 90);
  }

  public double getAngle() {
    if(RobotBase.isReal()){
      return m_encoder.getPosition() - 270;
    } else {
      return m_armSimAngle;
    }
  }

  public Command setPosition(ArmState pos) {
    return Commands.run(() -> {
      m_state = pos;
      m_setPoint = m_state.getAngle();
    }, this).until(atPosition);
  }

  public Command scoreCoral() {
    return Commands.runOnce(() -> {
      m_setPoint = m_state.getAngle() - ArmConstants.k_dunkAngle;
      System.out.println("Arm scoreCoral");
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
    return -180 - angle;
  }

  public double getSetPoint() {
    return m_invert ? invertPos(m_setPoint) : m_setPoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_output = m_pid.calculate(getAngle(), getSetPoint()) + ArmConstants.k_f * Math.cos(Math.toRadians(getSetPoint()));
    m_armMotor.setVoltage(m_output);
    SmartDashboard.putNumber("Arm Angle", getAngle());
    SmartDashboard.putNumber("Arm voltage", m_output);
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
