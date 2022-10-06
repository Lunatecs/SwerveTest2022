// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  public final DrivetrainSubsystem swerveDrive = new DrivetrainSubsystem();
  public final Joystick driver = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    swerveDrive.SetFieldRelative(true);

    configureButtonBindings();
    configureDefaultCommands();


    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  public void configureDefaultCommands() {
    swerveDrive.setDefaultCommand(new SwerveModule(driveMotorChannel, turningMotorChannel, turningEncoder, encoderOffset));
  }

  private void configureButtonBindings() {



  }

  public void teleopExecute() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand; //cgange
  }


  public void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed or forward speed

    double xSpeed = (-controller.getRawAxis(x)) * Constants.DriveTrainConstants.kMaxSpeed;

    // Get the y speed or sideways/strafe speed.
    double ySpeed = -controller.getRawAxis(y) * Constants.DriveTrainConstants.kMaxSpeed;
    if(Math.abs(ySpeed) < 0.15){
        ySpeed = 0;
    }

    // Get the rate of angular rotation.
    double rot = -controller.getRawAxis(r) * Constants.DriveTrainConstants.kMaxAngularSpeed;

    // Increase max speed by throttle axis (inverted and add one makes the axis from 1 to 2)
    if (t != -1) {
        double throttle = (-controller.getRawAxis(t) + 1);
        swerveDrive.setMaxSpeed(throttle);
    }

     if ((Math.abs(xSpeed) < Constants.ControllerConstants.NoInputTolerance)
                && (Math.abs(ySpeed) < Constants.ControllerConstants.NoInputTolerance)
                && (Math.abs(rot) < Constants.ControllerConstants.NoInputTolerance)) {
         if(swerveDrive.isXDefault()){
             swerveDrive.CanDrive(true);
             swerveDrive.setWheelAngleStates(45, -45, -45, 45);
         }
         else{
            swerveDrive.CanDrive(false);
            swerveDrive.drive(0, 0, 0, fieldRelative);
        }
    }
    else{
        swerveDrive.CanDrive(true);
        swerveDrive.drive(getCurve(xSpeed), getCurve(ySpeed), getCurve(rot), fieldRelative);
    }
}
}
