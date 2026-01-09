// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Robot Pose", Pose2d.struct).publish();
  StructArrayPublisher<Pose3d> zeroedComponentPoses = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Zeroed Poses", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> finalComponentPoses = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Component Poses", Pose3d.struct).publish();

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
    posePublisher.set(m_robotContainer.m_swerveSubsystem.getPose());
    zeroedComponentPoses.set(new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});
    // finalComponentPoses.set(new Pose3d[] {
      // new Pose3d(0.226,0,0.158, new Rotation3d(0, Math.toRadians(-m_robotContainer.m_pivotSubsystem.getAngle()),0)),
      // new Pose3d(-0.069,0, 0.343 + m_robotContainer.m_elevatorSubsystem.getPosition(), new Rotation3d(0,0,0)),
      // new Pose3d(-0.069,0, 0.343 + m_robotContainer.m_elevatorSubsystem.getPosition(), new Rotation3d(Math.toRadians(m_robotContainer.m_armSubsystem.getAngle()),0,0))});
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
