package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSystem;


public class DriveWithNavX extends Command {
  
  SwerveSystem swerveSystem;
  PS4Controller controller;
  double modifier;

  public DriveWithNavX(SwerveSystem swerveSystem, PS4Controller controller) {
    this.swerveSystem = swerveSystem;
    this.controller = controller;
    addRequirements(swerveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    modifier = 0.2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the input from the controllers and apply deadzonez and input shaping 
    double y = squareValues(applyDeadzone(controller.getLeftY())); // Linear speed in x direction
    double x = squareValues(applyDeadzone(controller.getLeftX())); // Linear speed in y direction 
    double rot = squareValues(applyDeadzone(controller.getRightX())); // rotational speed 

    // positive Y is forward
    // positive X is Left
    // positive rotation is counter clock wise
    swerveSystem.driveWithNavX(-y * 4.4196 * modifier, -x * 4.4196 * modifier, -rot * 4.4196); // * 180

    // swerveSystem.driveWithNavX(0, 0, -rot * Math.PI); // * 180

  }

  private double applyDeadzone(double value) {
    if (Math.abs(value) < 0.15){
      return 0;
    }
    return value;
  }


  private double squareValues(double value){
    // return Math.pow(Math.abs(value), 0.5) * Math.signum(value);
    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}