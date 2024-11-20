package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static frc.robot.Constants.DrivetrainConstants.*;

/**
 * DrivetrainSubsystem implements a basic differential drive system for a kitbot.
 * A differential drive uses two sides of motors that can be driven independently,
 * allowing the robot to move forward/backward and turn by varying the speeds of each side.
 */
public class DrivetrainSubsystem extends SubsystemBase {
    // Instantiate motors
    /**
     * Motor controllers. In our kitbot, this is typically the only motor on the left side.
     * More complex robots might use multiple motors per side with follower configuration.
     */
    private final CANSparkMax leftFront = new CANSparkMax(LEFT_FRONT_ID, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax rightFront = new CANSparkMax(RIGHT_REAR_ID, CANSparkLowLevel.MotorType.kBrushed);
    
    /**
     * WPILib's DifferentialDrive class handles the calculations for tank and arcade drive.
     * It automatically handles basic motor safety and implements common drive algorithms.
     */
    private final DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);

    // Example of motor configuration for a four-motor drivetrain
    // private final CANSparkMax leftRear = new CANSparkMax(LEFT_REAR_ID, CANSparkLowLevel.MotorType.kBrushed);
    // private final CANSparkMax rightFront = new CANSparkMax(RIGHT_FRONT_ID, CANSparkLowLevel.MotorType.kBrushed);

    /**
     * Constructor for DrivetrainSubsystem. Here we configure all motor controllers
     * with their required settings:
     * - Factory defaults ensure we start from a known state
     * - Current limits prevent brownouts and protect motors
     * - Motor inversion ensures positive values move the robot forward
     */
    public DrivetrainSubsystem(){
        // Configure left side
        leftFront.restoreFactoryDefaults();
        leftFront.setSmartCurrentLimit(80);
        leftFront.setInverted(true);  // Left side typically needs to be inverted for robot to drive forward

        // Configure right side
        rightFront.restoreFactoryDefaults();
        rightFront.setSmartCurrentLimit(80);
        rightFront.setInverted(false);

        // Example of follower configuration for four-motor drivetrain
        // leftRear.restoreFactoryDefaults();
        // leftRear.setSmartCurrentLimit(80);
        // leftRear.follow(leftFront);  // Follower will copy the leader's output

        // rightRear.restoreFactoryDefaults();
        // rightRear.setSmartCurrentLimit(80);
        // rightRear.follow(rightFront);
    }

    /**
     * Sets the voltage of both motors directly.
     * This is useful for autonomous commands where you want precise control.
     * Voltage control is generally more consistent than speed control.
     * 
     * @param voltage The voltage to set (-12 to 12 volts)
     */
    public void setVoltage(double voltage) {
        leftFront.setVoltage(voltage); 
        rightFront.setVoltage(voltage);
    }

    /**
     * Implements arcade drive control scheme.
     * - Speed: Forward/backward motion (-1 to 1)
     * - Rotation: Turning motion (-1 to 1)
     * 
     * The DifferentialDrive class will handle mixing these inputs into
     * appropriate motor outputs for each side of the robot.
     * 
     * @param speed Forward/backward speed
     * @param rotation Turning speed
     */
    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    /**
     * Creates a command for driving the robot with arcade control.
     * Uses Suppliers to continuously sample the controller inputs.
     * 
     * Suppliers are used instead of direct values because:
     * 1. We want to read the latest controller input every robot loop
     * 2. The command is created once but needs to use updated values
     * 3. This allows the command to be created before the values are known
     * 
     * @param speedSupplier Supplier for forward/backward speed (-1 to 1)
     * @param rotationSupplier Supplier for rotation speed (-1 to 1)
     * @return Command that will drive robot based on supplied values
     */
    public Command getDriveCommand(Supplier<Double> speedSupplier, Supplier<Double> rotationSupplier) {
        return this.run(
            () -> {
                arcadeDrive(speedSupplier.get(), rotationSupplier.get());
            }
        );
    }
}