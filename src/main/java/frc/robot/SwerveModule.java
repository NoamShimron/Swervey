package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  // Slot0Configs driveConfigs = new Slot0Configs(); should be initialized inside
  // constructor
  // Slot0Configs steerConfigs = new Slot0Configs(); should be initialized inside
  // constructor
  // // Create TalonFXConfiguration objects for drive and steer
  // TalonFXConfiguration driveConfig = new TalonFXConfiguration(); should be
  // initialized inside constructor
  // TalonFXConfiguration steerConfig = new TalonFXConfiguration(); should be
  // initialized inside constructor

  public SwerveModule(int canCoderID, int driveID, int steerID, double offset, InvertedValue driveDirection,
      InvertedValue steerDirection) {
    canCoder = new CANcoder(canCoderID);
    drive = new TalonFX(driveID);
    steer = new TalonFX(steerID);


    CANcoderConfiguration CanCoderConfig = new CANcoderConfiguration();
    CanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // this might be backwards
    CanCoderConfig.MagnetSensor.MagnetOffset = offset; // might need to be negative
    canCoder.getConfigurator().apply(CanCoderConfig);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    TalonFXConfiguration steerConfig = new TalonFXConfiguration();

    Slot0Configs driveSlot0 = driveConfig.Slot0;
    Slot0Configs steerSlot0 = steerConfig.Slot0;

    driveConfig.MotorOutput.Inverted = driveDirection;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted = steerDirection;
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
  
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

    driveSlot0.kP = 0.008;
    driveSlot0.kI = 0;
    driveSlot0.kD = 0.002;
    driveSlot0.kS = 0;

    steerSlot0.kP = 1.6;
    steerSlot0.kI = 0;
    steerSlot0.kD = 0.026;
    steerSlot0.kS = 0;


    drive.getConfigurator().apply(driveConfig);
    steer.getConfigurator().apply(steerConfig);

    // drive.setSelectedSensorPosition(0);
    drive.setPosition(0);
    // steer.setSelectedSensorPosition(0);
    // steer.setSelectedSensorPosition(canCoder.getAbsolutePosition() / 360 *
    // Constants.TALONFX_CPR);
    steer.setPosition(canCoder.getAbsolutePosition().getValue()   / Constants.DRIVE_STEER_GEAR_RATIO);
    // steer.setSelectedSensorPosition(
    //     (canCoder.getAbsolutePosition() - offset) / 360 * Constants.TALONFX_CPR / Constants.DRIVE_STEER_GEAR_RATIO);
  }

  public void stop() {
    drive.stopMotor();
    steer.stopMotor();
  }

  public void move(double driveDutyCycle, double steerDutyCycle) {
    // this.drive.set(ControlMode.PercentOutput, drive);
    this.drive.setControl(new DutyCycleOut(driveDutyCycle));
    this.steer.setControl(new DutyCycleOut(steerDutyCycle));
  }

  public double getHeadingDegrees() {
    return steer.getPosition().getValue() *  Constants.DRIVE_STEER_GEAR_RATIO * 360;
  }

  public double steerSensoerPos() {
    return steer.getPosition().getValue();
  }

  public double getVelocityRPM() {
    // return drive.getSelectedSensorVelocity() / Constants.DRIVE_GEAR_RATIO * 2 *
    // Math.PI * Constants.DRIVE_WHEEL_RADIUS / 60;
    return drive.getVelocity().getValue() * 600 * Constants.DRIVE_GEAR_RATIO;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedDesiredState = optimize(desiredState,new Rotation2d(Math.toRadians(getHeadingDegrees())));
    double desiredSpeed = (optimizedDesiredState.speedMetersPerSecond * 60 / (2 * Math.PI * Constants.DRIVE_WHEEL_RADIUS)) / Constants.DRIVE_GEAR_RATIO;
    double desiredPosition = optimizedDesiredState.angle.getDegrees() / 360.0 / Constants.DRIVE_STEER_GEAR_RATIO;
    SmartDashboard.putNumber("state speed" + hashCode(), desiredSpeed);
    SmartDashboard.putNumber("state pos" + hashCode(), desiredPosition);
    if (desiredSpeed == 0) {
      drive.setControl(new DutyCycleOut(0));
      steer.setControl(new DutyCycleOut(0));
    } else {
      drive.setControl(new VelocityDutyCycle(desiredSpeed));
      steer.setControl(new PositionDutyCycle(desiredPosition));
    }
  }

  public double getAbsEncoder() {
    return canCoder.getAbsolutePosition().getValue();
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
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  public double getDistanceMeters() {
    double sensorPosition = drive.getPosition().getValue();
    return Constants.DRIVE_WHEEL_RADIUS*2*Math.PI*(sensorPosition/Constants.TALONFX_CPR)/Constants.DRIVE_DRIVE_GEAR_RATIO;
  }

  
  public double getDistanceTraveled() {
    // Retrieve the position signal from the TalonFX
    double sensorPosition = drive.getPosition().getValue();

    // Perform the distance calculation
    return (sensorPosition / Constants.TALONFX_CPR * Constants.DRIVE_DRIVE_GEAR_RATIO * 2 * Constants.DRIVE_WHEEL_RADIUS * Math.PI);
}

  
  public SwerveModulePosition getModulePosition() {
    SwerveModulePosition pos = new SwerveModulePosition(getDistanceTraveled(),new Rotation2d(getHeadingRad()));
    return pos;
  }
  public double getHeadingRad() {
        double sensorPosition = drive.getPosition().getValue();

    return Math.toRadians(sensorPosition / Constants.TALONFX_CPR * Constants.DRIVE_STEER_GEAR_RATIO * 360) % 360;
  }
  
}
