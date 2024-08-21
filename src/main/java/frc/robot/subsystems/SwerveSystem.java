// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveSystem extends SubsystemBase {
  
  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;

  SwerveModule[] modulesArray;

  SwerveDriveKinematics kinematics;

  AHRS navX;

  public SwerveSystem() {

    modulesArray = new SwerveModule[4];
                                                                          
    frontLeft = new SwerveModule(0, 40, 1, 125, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive);
    frontRight = new SwerveModule(22, 8, 15, 45, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive);
    backLeft = new SwerveModule(61, 60, 51, 261, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive);
    backRight = new SwerveModule(20, 30, 5, 75, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive);
                    
    
    modulesArray[0] = frontLeft;
    modulesArray[1] = frontRight;
    modulesArray[2] = backLeft;
    modulesArray[3] = backRight;

    kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.DRIVE_OFFSET, Constants.DRIVE_OFFSET),
      new Translation2d(Constants.DRIVE_OFFSET, -Constants.DRIVE_OFFSET),
      new Translation2d(-Constants.DRIVE_OFFSET, Constants.DRIVE_OFFSET),
      new Translation2d(-Constants.DRIVE_OFFSET, -Constants.DRIVE_OFFSET)
      );

    navX = new AHRS(SPI.Port.kMXP);
  }

  public void stop(){
    for(int i = 0; i < 4; i++){
      modulesArray[i].stop();
    } 
  }

  public void move(double drive, double rotaion){
    for(int i = 0; i < 4; i++){
      modulesArray[i].move(drive, rotaion);
    }
  }

  public void setDesiredStates(SwerveModuleState[] states){
    for(int i = 0; i < 4; i++){
      modulesArray[i].setDesiredState(states[i]);
    }
  }

  public void drive(double speedY, double speedX, double rotation){
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(speedY, speedX, Math.toRadians(rotation)));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.4196);
    setDesiredStates(states);
  }

  public void getData(){
    SmartDashboard.putNumber("front left heading", modulesArray[0].getHeadingDegrees());
    SmartDashboard.putNumber("front right heading", modulesArray[1].getHeadingDegrees());
    SmartDashboard.putNumber("back left heading", modulesArray[2].getHeadingDegrees());
    SmartDashboard.putNumber("back right heading", modulesArray[3].getHeadingDegrees());

    SmartDashboard.putNumber("front left abs encoder", modulesArray[0].getAbsEncoder());
    SmartDashboard.putNumber("front right abs encoder", modulesArray[1].getAbsEncoder());
    SmartDashboard.putNumber("back left abs encoder", modulesArray[2].getAbsEncoder());
    SmartDashboard.putNumber("back right abs encoder", modulesArray[3].getAbsEncoder());

    SmartDashboard.putNumber("front left steer sensoer pos", modulesArray[0].steerSensoerPos());
    SmartDashboard.putNumber("front right steer sensoer pos", modulesArray[1].steerSensoerPos());
    SmartDashboard.putNumber("back left steer sensoer pos", modulesArray[2].steerSensoerPos());
    SmartDashboard.putNumber("back right steer sensoer pos", modulesArray[3].steerSensoerPos());

    SmartDashboard.putNumber("front left steer velocity RPM", modulesArray[0].getVelocityRPM());
    SmartDashboard.putNumber("front right steer velocity RPM", modulesArray[1].getVelocityRPM());
    SmartDashboard.putNumber("back left steer velocity RPM", modulesArray[2].getVelocityRPM());
    SmartDashboard.putNumber("back right steer velocity RPM", modulesArray[3].getVelocityRPM());

    SmartDashboard.putNumber("navX pitch", navX.getPitch());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("front left heading", modulesArray[0].getHeadingDegrees());
    SmartDashboard.putNumber("front right heading", modulesArray[1].getHeadingDegrees());
    SmartDashboard.putNumber("back left heading", modulesArray[2].getHeadingDegrees());
    SmartDashboard.putNumber("back right heading", modulesArray[3].getHeadingDegrees());

    SmartDashboard.putNumber("front left abs encoder", modulesArray[0].getAbsEncoder());
    SmartDashboard.putNumber("front right abs encoder", modulesArray[1].getAbsEncoder());
    SmartDashboard.putNumber("back left abs encoder", modulesArray[2].getAbsEncoder());
    SmartDashboard.putNumber("back right abs encoder", modulesArray[3].getAbsEncoder());

    SmartDashboard.putNumber("front left steer sensoer pos", modulesArray[0].steerSensoerPos());
    SmartDashboard.putNumber("front right steer sensoer pos", modulesArray[1].steerSensoerPos());
    SmartDashboard.putNumber("back left steer sensoer pos", modulesArray[2].steerSensoerPos());
    SmartDashboard.putNumber("back right steer sensoer pos", modulesArray[3].steerSensoerPos());

    SmartDashboard.putNumber("front left steer velocity RPM", modulesArray[0].getVelocityRPM());
    SmartDashboard.putNumber("front right steer velocity RPM", modulesArray[1].getVelocityRPM());
    SmartDashboard.putNumber("back left steer velocity RPM", modulesArray[2].getVelocityRPM());
    SmartDashboard.putNumber("back right steer velocity RPM", modulesArray[3].getVelocityRPM());

    SmartDashboard.putNumber("navX pitch", navX.getPitch());
  }
}
