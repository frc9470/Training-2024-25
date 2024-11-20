// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.LauncherConstants.*;

/**
 * The LauncherSubsystem class demonstrates different ways to create and manage commands
 * for a two-motor launcher mechanism in FRC. This subsystem controls two CANSparkMax motors
 * configured as brushed motors - one at the top and one at the bottom of the launcher.
 * 
 * The launcher has two primary functions:
 * 1. Intake - Both motors spin clockwise (negative voltage) to intake a game piece
 * 2. Launch - Motors spin counterclockwise (positive voltage) to launch the game piece
 */
public class LauncherSubsystem extends SubsystemBase {
  /** The top motor of the launcher mechanism */
  private final CANSparkMax top = new CANSparkMax(TOP_ID, CANSparkLowLevel.MotorType.kBrushed);
  /** The bottom motor of the launcher mechanism */
  private final CANSparkMax bottom = new CANSparkMax(BOTTOM_ID, CANSparkLowLevel.MotorType.kBrushed);

  /** 
   * Creates a new LauncherSubsystem.
   * Initializes both motors with factory defaults and sets current limits.
   */
  public LauncherSubsystem() {
    top.restoreFactoryDefaults();
    top.setSmartCurrentLimit(80);

    bottom.restoreFactoryDefaults();
    bottom.setSmartCurrentLimit(80);
  }

  /**
   * 1. Intake
   * - spin both mototrs clockwise (-ve voltage)
   * 2. Shoot
   * - spin top flywheel counterclockwise (+ve voltage)
   *  - wait one second to let it reach target velocity
   * - both spin counterclockwise (+ve voltage)
   */

  /**
   * Different ways to create commands for the launcher subsystem:
   * 1. Using anonymous Command class
   * 2. Using runEnd with lambda
   * 3. Using runEnd with helper method
   * 4. Using sequential commands
   * 5. Using command groups
   */

  /**
   * Creates an intake command using an anonymous Command class.
   * This demonstrates the most verbose but most flexible way to create a command.
   * 
   * @return A Command that runs both motors in reverse (-10V) while active
   */
  public Command getIntakeCommand(){
    return new Command(){

      @Override
      public void execute() {
        // every 20 ms
        top.setVoltage(-10);
        bottom.setVoltage(-10);
      }

      @Override
      public void end(boolean interrupted) {
        top.setVoltage(0);
        bottom.setVoltage(0);
      }

    };
  }

  /**
   * Creates an intake command using the runEnd method with lambda expressions.
   * This is a more concise way to create a command compared to the anonymous class approach.
   * 
   * @return A Command that runs both motors in reverse (-10V) while active
   */
  public Command getIntakeCommand2(){
    return this.runEnd(() -> {
      top.setVoltage(-10);
      bottom.setVoltage(-10);
    },
    () -> {
      top.setVoltage(0);
      bottom.setVoltage(0);
    });
  }
  
  /**
   * Sets the voltage for both motors simultaneously.
   * Helper method to reduce code duplication.
   * 
   * @param volts The voltage to apply to both motors
   */
  private void setVoltage(double volts){
    top.setVoltage(volts);
    bottom.setVoltage(volts);
  }

  /**
   * Creates an intake command using runEnd with the helper setVoltage method.
   * This demonstrates the most concise way to create a command.
   * 
   * @return A Command that runs both motors in reverse (-10V) while active
   */
  public Command getIntakeCommand3(){
    return this.runEnd(() -> setVoltage(-10), () -> setVoltage(0));
  }

  /**
   * Creates a command to run only the top motor.
   * Useful for the first stage of shooting where we want to rev up just the top flywheel.
   * 
   * @return A Command that runs the top motor at 12V while active
   */
  public Command runTop(){
    return this.runEnd(() -> top.setVoltage(12), () -> top.setVoltage(0));
  }

  /**
   * Creates a command to run both motors simultaneously.
   * Used for the second stage of shooting where both flywheels need to spin.
   * 
   * @return A Command that runs both motors at 12V while active
   */
  public Command runBoth(){
    return this.runEnd(() -> {
      top.setVoltage(12);
      bottom.setVoltage(12);
    },
    () -> {
      top.setVoltage(0);
      bottom.setVoltage(0);
    });
  }

  /**
   * Creates a shooting command using command chaining.
   * This method demonstrates using withTimeout and andThen to sequence actions.
   * 
   * @return A Command that first runs the top motor for 1 second, then runs both motors
   */
  public Command shoot(){
    return runTop()
      .withTimeout(1)
      .andThen(runBoth());
  }

  /**
   * Creates a shooting command using SequentialCommandGroup.
   * This method demonstrates an alternative way to sequence commands using command groups.
   * 
   * @return A Command that runs the top motor, waits 1 second, then runs both motors
   */
  public Command shoot2(){
    return new SequentialCommandGroup(
      runTop().raceWith(new WaitCommand(1)),
      runBoth()
    );
  }

  /**
   * Creates a shooting command using inline command chaining.
   * This method demonstrates creating the entire command sequence without helper methods.
   * 
   * @return A Command that runs the top motor for 1 second, then runs both motors
   */
  public Command shoot3(){
    return this.runEnd(() -> top.setVoltage(12), () -> top.setVoltage(0.))
      .withTimeout(1)
      .andThen(this.runEnd(() -> {
          top.setVoltage(12);
          bottom.setVoltage(12);
        },
        () -> {
          top.setVoltage(0);
          bottom.setVoltage(0);
        }));
  }

  /**
   * Creates a launching command using the run method instead of runEnd.
   * This demonstrates another way to chain commands with timing controls.
   * 
   * @return A Command that runs the top motor for 1 second, then runs both motors
   */
  public Command launchNote(){
    return this.run(() -> top.setVoltage(12.0))
      .withTimeout(1.0)
      .andThen(this.runEnd(() -> setVoltage(12.0), () -> setVoltage(0.0)));
  }

  /**
   * Creates a command to run both motors at a specified voltage.
   * This demonstrates how to create parameterized commands.
   * 
   * @param voltage The voltage to apply to both motors
   * @return A Command that runs both motors at the specified voltage while active
   */
  public Command runShooter(int voltage){
    return this.runEnd(() -> {
      top.setVoltage(voltage);
      bottom.setVoltage(voltage);
    },
    () -> {
      top.setVoltage(0);
      bottom.setVoltage(0);
    });
  }

  /**
   * Example method showing how to query subsystem state.
   * This could be used to check limit switches, encoders, or other sensors.
   * 
   * @return A boolean representing some subsystem state
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  /**
   * This method will be called once per scheduler run.
   * Override this to add custom periodic behavior to the subsystem.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * This method will be called once per scheduler run during simulation.
   * Override this to add custom simulation behavior to the subsystem.
   */
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
