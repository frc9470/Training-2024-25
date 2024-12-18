package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class ArmSubsystem extends SubsystemBase{

    private final CANSparkMax arm = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder encoder = arm.getEncoder();

    private final ArmFeedforward ff = new ArmFeedforward(ARM_KS, ARM_KG, 0, 0);
    private final ProfiledPIDController pid = 
        new ProfiledPIDController(0, 0, 0, 
            new Constraints(4, 5)
        );

    private double goal = 0; 

    public ArmSubsystem() {
        // .. init 
        arm.restoreFactoryDefaults();
        arm.setSmartCurrentLimit(50);
        //.. reset your encoder to absolute position 
        

        encoder.setPositionConversionFactor(1/3.0);
    }

    @Override
    public void periodic() {

        arm.setVoltage(pid.calculate(encoder.getPosition(), goal) + ff.calculate(encoder.getPosition(), pid.getSetpoint().velocity /* (rad/s) */));
    }


    public Command moveToTarget(double target){
        return new InstantCommand(() -> goal = target);
    }


}