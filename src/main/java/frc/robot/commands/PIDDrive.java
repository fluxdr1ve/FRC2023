// package frc.robot.commands;

// import frc.robot.subsystems.DriveTrainSubsystem;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.math.controller.PIDController;

// public class PIDDrive extends PIDCommand 
// {

// double kP = DriveConstants.kP;
// double kI = DriveConstants.kI;
// double kD = DriveConstants.kD;
// double iLimit = DriveConstants.iLimit; 

//         public PIDDrive(double setpoint, DriveTrainSubsystem drive){
//             super(
//                 new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
                
//                 // Close loop on heading
//                 drive::getLeftSpeed,
//                 // Set reference to target
//                 setpoint,
//                 // Pipe output to turn robot
//                 output -> drive.arcadeDrive(output, 0),
//                 // Require the drive
//                 drive);
           
//         }

//         @Override
//         public boolean isFinished()
//         {
//             return getController().atSetpoint();
//         }
// }
// PID commands are bad just PIDcontroller