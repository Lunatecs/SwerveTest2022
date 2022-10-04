package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;

  private final CANCoder turningEncoder;

  private final PIDController drivePIDController = new PIDController(Constants.DriveTrainConstants.driveSpeedP,
      Constants.DriveTrainConstants.driveSpeedI, Constants.DriveTrainConstants.driveSpeedD);
  private final PIDController turningPIDController = new PIDController(Constants.DriveTrainConstants.turnP,
      Constants.DriveTrainConstants.turnI, Constants.DriveTrainConstants.turnD);

  private double encoderOffset;

  private boolean driving;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoder, double encoderOffset) {
    driveMotor = new WPI_TalonFX(driveMotorChannel);
    driveMotor.configFactoryDefault();
    driveMotor.setNeutralMode(NeutralMode.Brake);

    turningMotor = new WPI_TalonFX(turningMotorChannel);
    turningMotor.configFactoryDefault();
    turningMotor.setNeutralMode(NeutralMode.Coast);

    turningEncoder = new CANCoder(turningEncoder);
    encoderOffset = encoderOffset;
    resetDriveEncoder();

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void canDrive(boolean value){
    driving = value;
  }

  // Do not use unless the turn encoders absolute value changed
  public void setAbsoluteOffset() {
    turningEncoder.configMagnetOffset(encoderOffset);
  }

  public void resetDriveEncoder() {
    driveMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public double GetDriveEncoder() {
    return driveMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getSensorCollection().getIntegratedSensorVelocity(),
        new Rotation2d(Math.toRadians(turningEncoder.getAbsolutePosition())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    // Changing base encoder velocity to meters per socond
    double driveEncoderVelocity = ((driveMotor.getSensorCollection().getIntegratedSensorVelocity())
        * Constants.DriveTrainConstants.driveEncoderMultiplier);
    double driveOutput = drivePIDController.calculate(driveEncoderVelocity, state.speedMetersPerSecond);

    // Clamps the drive output to prevent damage to gears
    if (driveOutput > Constants.DriveTrainConstants.driveMaxOutput) {
      driveOutput = Constants.DriveTrainConstants.driveMaxOutput;
    } else if (driveOutput < -Constants.DriveTrainConstants.driveMaxOutput) {
      driveOutput = -Constants.DriveTrainConstants.driveMaxOutput;
    }

    // Calculate the turning motor output from the turning PID controller.
    // Changing encoder value to radians between -pi to pi
    double turnEncoderPosition = (turningEncoder.getAbsolutePosition()
        * Constants.DriveTrainConstants.turnEncoderMultiplier) - Math.PI;
    double turnOutput = turningPIDController.calculate(turnEncoderPosition, state.angle.getRadians());

    // Clamps the turn output to prevent damage to gears
    if (turnOutput > Constants.DriveTrainConstants.turnMaxOutput) {
      turnOutput = Constants.DriveTrainConstants.turnMaxOutput;
    } else if (turnOutput < -Constants.DriveTrainConstants.turnMaxOutput) {
      turnOutput = -Constants.DriveTrainConstants.turnMaxOutput;
    }

    // Set motor power to pid loop outputs
    if(driving){
      driveMotor.set(-driveOutput);
      turningMotor.set(turnOutput);
    }
    else{
     driveMotor.set(0);
     turningMotor.set(0);
    }
  }
}
