// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class SetElevator extends Command {
//   /** Creates a new SetElevator. */
//   Supplier<Double> m_supplier;
//   ExampleSubsystem subsystem;
//   public SetElevator(ExampleSubsystem sub, Supplier<Double> supplier) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     subsystem = sub;
//     m_supplier = supplier;
//     addRequirements(subsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     subsystem.setDuty(m_supplier.get());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
