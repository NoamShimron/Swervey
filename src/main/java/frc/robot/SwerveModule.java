package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveModule {
    
  CANcoder canCoder;
  TalonFX drive;
  TalonFX steer;
  Slot0Configs driveConfigs = new Slot0Configs();
  Slot0Configs steerConfigs = new Slot0Configs();
  // Create TalonFXConfiguration objects for drive and steer
TalonFXConfiguration driveConfig = new TalonFXConfiguration();
TalonFXConfiguration steerConfig = new TalonFXConfiguration();
  
  public SwerveModule(int canCoderID, int driveID, int steerID, double offset, boolean isInvertedDrive, boolean isInvertedSteer) {
    canCoder = new CANcoder(canCoderID);
    drive = new TalonFX(driveID);
    steer = new TalonFX(steerID);
    Slot0Configs driveConfigs = new Slot0Configs();
    Slot0Configs steerConfigs = new Slot0Configs();
    

    drive.getConfigurator().apply(new TalonFXConfiguration());
    steer.getConfigurator().apply(new TalonFXConfiguration());

    drive.setInverted(false);
    steer.setInverted(true);

  
    CANcoderConfiguration  configs = new CANcoderConfiguration();
    configs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoder.getConfigurator().apply(configs);
    // drive.config_kP(0, 0.065); //0.07
    // drive.Slot0Configs.config_kP()
    // drive.config_kI(0, 0.0001);
    // drive.config_kD(0, 2);
    // drive.config_kF(0, 0.047);
    // drive.config_IntegralZone(0, 300);
    // steer.config_kP(0, 0.2);
    // steer.config_kI(0, 0.00003);
    // steer.config_kD(0, 2);
    // steer.config_kF(0, 0);

    driveConfigs.kP = 0.065; 
    driveConfigs.kI = 0.00001; 
    driveConfigs.kD = 2;
    driveConfigs.kS = 0.047;

    steerConfigs.kP = 4000;
    steerConfigs.kI = 7;
    steerConfigs.kD = 55;
    steerConfigs.kS = 999;
    driveConfig.Slot0= driveConfigs;
    steerConfig.Slot0= steerConfigs;

    // drive.setSelectedSensorPosition(0);
    drive.setPosition(0);
    //steer.setSelectedSensorPosition(0);
    // steer.setSelectedSensorPosition(canCoder.getAbsolutePosition() / 360 * Constants.TALONFX_CPR);
    steer.setSelectedSensorPosition((canCoder.getAbsolutePosition() - offset) / 360 * Constants.TALONFX_CPR / Constants.DRIVE_STEER_GEAR_RATIO);
  }

  public void stop(){
    drive.stopMotor();
    steer.stopMotor();
  }

  public void move(double drive, double rotation){
    this.drive.set(ControlMode.PercentOutput, drive);
    steer.set(ControlMode.PercentOutput, rotation);
  }

  public double getHeadingDegrees(){
    return steer.getSelectedSensorPosition() / Constants.TALONFX_CPR * Constants.DRIVE_STEER_GEAR_RATIO * 360;
  }

  public double steerSensoerPos(){
    return steer.getSelectedSensorPosition();
  }

  public double getVelocityRPM(){
    // return drive.getSelectedSensorVelocity() / Constants.DRIVE_GEAR_RATIO * 2 * Math.PI * Constants.DRIVE_WHEEL_RADIUS / 60;
    return drive.getSelectedSensorVelocity() * 600 / Constants.TALONFX_CPR * Constants.DRIVE_GEAR_RATIO;
  }

  public void setDesiredState(SwerveModuleState desiredState){
    SwerveModuleState optimizedDesiredState = optimize(desiredState, new Rotation2d(Math.toRadians(getHeadingDegrees())));
    double desiredSpeed = (optimizedDesiredState.speedMetersPerSecond * 60 / (2 * Math.PI * Constants.DRIVE_WHEEL_RADIUS)) / Constants.DRIVE_GEAR_RATIO * (Constants.TALONFX_CPR / 600.0);
    double desiredPosition = optimizedDesiredState.angle.getDegrees() / (360.0 / (Constants.TALONFX_CPR / Constants.DRIVE_STEER_GEAR_RATIO));
    SmartDashboard.putNumber("state speed"+hashCode(), desiredSpeed);
    SmartDashboard.putNumber("state pos"+hashCode(), desiredPosition);
    if (desiredSpeed == 0) {
      drive.set(ControlMode.PercentOutput, 0);
      steer.set(ControlMode.PercentOutput, 0);
    } else {
      drive.set(ControlMode.Velocity, desiredSpeed);
      steer.set(ControlMode.Position, desiredPosition);
    }
  }

  public double getAbsEncoder(){
    return canCoder.getAbsolutePosition();
  }

  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }
}
