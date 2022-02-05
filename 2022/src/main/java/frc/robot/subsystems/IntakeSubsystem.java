package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class IntakeSubsystem implements Subsystem {
	private final CANSparkMax intakeMotor;


	public IntakeSubsystem(){
		intakeMotor = new CANSparkMax(Constants.kIntakeMotorPort, MotorType.kBrushless);
		intakeMotor.restoreFactoryDefaults();
		intakeMotor.setInverted(Constants.kIntakeInverted);

		// intakeMotor.burnFlash();
	}

	public void runIntake(){
		intakeMotor.set(0.5);
	}

	public void stop(){
		intakeMotor.set(0);
	}
}
