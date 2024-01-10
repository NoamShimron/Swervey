// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSystem;

public class Drive extends CommandBase {
  
  SwerveSystem swerveSystem;
  PS4Controller controller;
  double modifier;

  public Drive(SwerveSystem swerveSystem, PS4Controller controller) {
    this.swerveSystem = swerveSystem;
    this.controller = controller;
    addRequirements(swerveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    modifier = 0.4;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = applyDeadzone(controller.getLeftY());
    double x = applyDeadzone(controller.getLeftX());
    double rot = applyDeadzone(controller.getRightX());

    SmartDashboard.putNumber("Drive x" , x);
    SmartDashboard.putNumber("Drive y" , y);
    SmartDashboard.putNumber("Drive rot" , rot);

    swerveSystem.drive(-y * 4.4196 * modifier, -x * 4.4196 * modifier, rot * 4.4196 / 0.035); // * 180

    // swerveSystem.drive(-y * 4.4196, x * 4.4196, rot * 4.4196 / 0.035); // * 180

    // swerveSystem.drive(0.1 * 4.4196, 0.1 * 4.4196, 0); // * 180
  }

  private double applyDeadzone(double value) {
    if (Math.abs(value) < 0.1) return 0;
    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
