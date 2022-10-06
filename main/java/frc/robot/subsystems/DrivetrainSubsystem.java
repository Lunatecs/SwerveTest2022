// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  private final Translation2d frontLeftLocation = new Translation2d(0.3, 0.3); // figure out what the numbers mean
  private final Translation2d frontRightLocation = new Translation2d(0.3, -0.3);
  private final Translation2d backLeftLocation = new Translation2d(-0.3, 0.3);
  private final Translation2d backRightLocation = new Translation2d(-0.3, -0.3);

  private final SwerveModule frontLeft = new SwerveModule(DriveTrainConstants.frontLeftDrive, DriveTrainConstants.frontLeftTurn,
   DriveTrainConstants.frontLeftEncoder, 359.473); // final hardcoded constants should be changed
  private final SwerveModule frontRight = new SwerveModule(DriveTrainConstants.frontRightDrive, DriveTrainConstants.frontRightTurn,
   DriveTrainConstants.frontRightEncoder, 1.758);
  private final SwerveModule backLeft = new SwerveModule(DriveTrainConstants.backLeftDrive, DriveTrainConstants.backLeftTurn,
   DriveTrainConstants.backLeftEncoder, 0.088);
  private final SwerveModule backRight = new SwerveModule(DriveTrainConstants.backRightDrive, DriveTrainConstants.backRightTurn,
   DriveTrainConstants.backRightEncoder, 0.703);

  private final PigeonIMU gyro = new PigeonIMU(DriveTrainConstants.pigeon);

  private boolean isFieldRelative; // might be useful idk
  private boolean isXDefault; // idk

  private double MaxSpeed = Constants.DriveTrainConstants.kMaxSpeed;

  public enum module {
    frontLeft, frontRight, backLeft, backRight;
  }

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, 
                                                                               backLeftLocation, backRightLocation);
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle());


  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {}

  public void setModule(double power){
    backRight.setDesiredState(new SwerveModuleState(1, new Rotation2d(1)));
  }

  public void CanDrive(boolean value){
    frontLeft.canDrive(value);
    frontRight.canDrive(value);
    backLeft.canDrive(value);
    backRight.canDrive(value);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(gyro.getFusedHeading());
  }

  public Double GetGyroDegrees(){
    return Math.abs(gyro.getFusedHeading() % 360);
  }
  
  public void zeroGyroHeading() {
    gyro.setFusedHeading(0.0);
  }

  public void setGyro(double degrees) {
    gyro.setFusedHeading(degrees);
  }

  public boolean IsFieldRelative() {
    return isFieldRelative;
  }

  public void SetFieldRelative(boolean newValue) {
    this.isFieldRelative = newValue;
  }

  public boolean isXDefault(){
    return isXDefault;
  }

  public void setXDefault(boolean value){
    isXDefault = value;
  }


  public void setMaxSpeed(double value) {
    MaxSpeed = Constants.DriveTrainConstants.kMaxSpeed + value;
  }

  public double getMaxSpeed() {
    return MaxSpeed;
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveDriveKinematics[] swerveModuleStates = kinematics //replaced var keyword with SwerveDriveKinematics object
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setWheelAngleStates(double fl, double fr, double bl, double br) {
    setWheelState(module.frontLeft, fl, 0.0);
    setWheelState(module.frontRight, fr, 0.0);
    setWheelState(module.backLeft, bl, 0.0);
    setWheelState(module.backRight, br, 0.0);
  }

  public double[] getWheelAngles(){
    double[] wheelAngles = new double[] {getWheelState(module.frontLeft).angle.getDegrees(), 
                                         getWheelState(module.frontRight).angle.getDegrees(),
                                         getWheelState(module.backLeft).angle.getDegrees(), 
                                         getWheelState(module.backRight).angle.getDegrees()};
    return wheelAngles;
  }

  public void setWheelState(module module, double angle, double speed) {
    angle = Math.toRadians(angle);

    switch (module) {
    case frontLeft:
      frontLeft.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    case frontRight:
      frontRight.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    case backLeft:
      backLeft.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    case backRight:
      backRight.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    }
  }

  public double getWheelDriveEncoder(module module) {
    switch (module) {
    case frontLeft:
      return frontLeft.GetDriveEncoder();
    case frontRight:
      return frontRight.GetDriveEncoder();
    case backLeft:
      return backLeft.GetDriveEncoder();
    case backRight:
      return backRight.GetDriveEncoder();
    default:
      return 0;
    }
  }

  public SwerveModuleState getWheelState(module module) {
    switch (module) {
    case frontLeft:
      return frontLeft.getState();
    case frontRight:
      return frontRight.getState();
    case backLeft:
      return backLeft.getState();
    case backRight:
      return backRight.getState();
    default:
      return null;
    }
  }

  public void resetAllDriveEncoders() {
    frontLeft.resetDriveEncoder();
    frontRight.resetDriveEncoder();
    backLeft.resetDriveEncoder();
    backRight.resetDriveEncoder();
  }

  public void updateOdometry() {
    odometry.update(getAngle(),
                    frontLeft.getState(), 
                    frontRight.getState(), 
                    backLeft.getState(),
                    backRight.getState());
  }

  public void initialize(){}
  @Override
public void periodic() {/*This method will be called once per scheduler run*/}

  
}

