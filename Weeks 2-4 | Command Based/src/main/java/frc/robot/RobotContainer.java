// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LauncherSubsystem launcher = new LauncherSubsystem();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * CommandXboxController#<button>() call in this class.
   * 
   * Three key binding concepts demonstrated here:
   * 1. Default Commands: Commands that run when no other command is using the subsystem
   *    - Set using subsystem.setDefaultCommand()
   *    - Useful for continuous actions like driving
   * 
   * 2. Trigger Bindings: Mapping buttons to commands
   *    - whileTrue(): Command runs while button is held, stops when released
   *    - onTrue(): Command starts when button is pressed
   *    - toggleOnTrue(): Command toggles state each press
   * 
   * 3. Command Suppliers: Functions that provide real-time input values
   *    - () -> value : Lambda that returns current controller value
   *    - Used to continuously sample controller state
   */
  private void configureBindings() {

    // Default Command Example: Run drive command with continuous controller input
    drivetrain.setDefaultCommand(
      drivetrain.getDriveCommand(
        () -> -controller.getLeftY(),  // Supplier updates every loop
        () -> -controller.getRightX()
      )
    );

    // Trigger Binding Example: Command runs while button held
    controller.leftBumper()
      .whileTrue(launcher.getIntakeCommand());
      
    // Another Trigger Binding: Same pattern, different button
    controller.rightTrigger()
      .whileTrue(launcher.getLaunchCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand(() -> {});
  }
}
