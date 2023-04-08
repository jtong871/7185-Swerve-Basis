// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.DriverConstants;                 //needed if gunner is ever used
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  private final SwerveSubsystem swerve = new SwerveSubsystem();                                           //creates a new SwerveSubsystem, driver joystick and optional gunner joystick
  private final Joystick driver = new Joystick(OperatorConstants.kDriverPort);
 // private final Joystick gunner = new Joystick(OperatorConstants.kGunnerPort);          //useful for a second driver if needed

  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveCmd(swerve,            //asks for the joystick inputs from the driver
     () -> -driver.getRawAxis(1),
     () -> driver.getRawAxis(0),
     () -> driver.getRawAxis(4),
     () -> !driver.getRawButton(OperatorConstants.kBack)));



    configureBindings();
  }

  
  private void configureBindings() {
    new JoystickButton(driver, OperatorConstants.kStart).whenPressed(() -> swerve.zeroHeading());   //sets the specified button to zero the heading of the gyroscope when it's pressed
    
    
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    return null;
    
  }
}
