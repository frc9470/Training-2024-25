// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LauncherConstants.*;

/**
 * The LauncherSubsystem controls a two-motor launcher mechanism.
 * It provides commands for intaking and launching game pieces using
 * two CANSparkMax motors in a brushed configuration.
 */
public class LauncherSubsystem extends SubsystemBase {
    /** The top motor of the launcher mechanism */
    private final CANSparkMax topMotor = new CANSparkMax(TOP_ID, CANSparkLowLevel.MotorType.kBrushed);
    /** The bottom motor of the launcher mechanism */
    private final CANSparkMax bottomMotor = new CANSparkMax(BOTTOM_ID, CANSparkLowLevel.MotorType.kBrushed);

    /** 
     * Creates a new LauncherSubsystem.
     * Initializes both motors with factory defaults and sets current limits.
     */
    public LauncherSubsystem() {
        // Configure top motor
        topMotor.restoreFactoryDefaults();
        topMotor.setSmartCurrentLimit(80);

        // Configure bottom motor
        bottomMotor.restoreFactoryDefaults();
        bottomMotor.setSmartCurrentLimit(80);
    
    }
    /**
     * Sets the voltage for both motors simultaneously.
     * 
     * @param voltage The voltage to apply to both motors
     */
    private void setVoltage(double voltage) {
        topMotor.setVoltage(voltage);
        bottomMotor.setVoltage(voltage);
    }

    /**
     * Creates a command to intake a game piece.
     * Runs both motors in reverse to pull in the game piece.
     * 
     * @return A Command that runs both motors in reverse while active
     */
    public Command getIntakeCommand() {
        return this.runEnd(
            () -> setVoltage(-10.0),  // Execute: run motors in reverse
            () -> setVoltage(0.0)     // End: stop motors
        );
    }

    /**
     * Creates a command to launch a game piece.
     * The sequence:
     * 1. Spins up the top motor
     * 2. Waits for motor to reach speed (1 second)
     * 3. Runs both motors to launch the game piece
     * 
     * @return A Command that executes the launch sequence
     */
    public Command getLaunchCommand() {
        return this.runEnd(
            () -> topMotor.setVoltage(12.0),    // Spin up top motor
            () -> topMotor.setVoltage(0.0)      // By default, this end method never runs because we have no end condition. it only works with the timeout
        )
        .withTimeout(1.0)                        // Wait for motor to reach speed
        .andThen(
            this.runEnd(
                () -> setVoltage(12.0),         // Run both motors
                () -> setVoltage(0.0)
            )
        );
    }

    public double getFlywheelRPM(){
        return encoder.getVelocityRPM();
    }

    

    public Command getPIDCommand(double targetRPM){
        return new Command() {
            PIDController pid = new PIDController(KP, KI, KD);
            SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, KV);

            @Override
            public void execute() {
                topMotor.setVoltage(ff.calculate(targetRPM) + pid.calculate(getFlywheelRPM()));
            }
            
        };
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
