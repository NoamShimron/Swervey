// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveWithNavX;
import frc.robot.commands.Move;
import frc.robot.subsystems.SwerveSystem;

public class Robot extends TimedRobot {

  SwerveSystem swerveSystem;
  PS4Controller controller;

  @Override
  public void robotInit() {
    controller = new PS4Controller(0);
    swerveSystem = new SwerveSystem();
    swerveSystem.setDefaultCommand(new DriveWithNavX(swerveSystem, controller));
    // swerveSystem.setDefaultCommand(new Move(swerveSystem, controller));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
/*
 * meow
 * bongobongobongo
 * mfmapfipoeafocjeofoiejroifjewoa4tpoepmfwoi3jifwejidiojef
 */