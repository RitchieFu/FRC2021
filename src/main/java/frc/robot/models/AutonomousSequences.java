package frc.robot.models;

import java.sql.DriverManager;

import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathArcSegment;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.AutonomousTrajectoryCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FetchPowerCellCommand;
import frc.robot.commands.IntakeActuateCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDetectToElevatorIndexCommand;
import frc.robot.commands.RobotRotateCommand;
import frc.robot.commands.ShooterActuateCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.VisionRotationDriveCommand;

public class AutonomousSequences {

        public static CommandGroup GoFetchTest() {
                CommandGroup output = new CommandGroup();
                FetchPowerCellCommand fpcCommand = new FetchPowerCellCommand();

                output.addSequential(fpcCommand);
                
                return output;
        }

        public static CommandGroup DriveLeftThenRight() {
                CommandGroup output = new CommandGroup();
                Path driveLeft = new Path(Rotation2.ZERO);
                driveLeft.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2( 0.0, -120)
                        )
                );
                
                Trajectory driveLeftTrajectory = new Trajectory(driveLeft, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand driveLeftCommand = new AutonomousTrajectoryCommand(driveLeftTrajectory);

                Path driveRight = new Path(Rotation2.ZERO);
                driveRight.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2( 0.0, 120)
                        )
                );
                
                Trajectory driveRightTrajectory = new Trajectory(driveRight, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand driveRightCommand = new AutonomousTrajectoryCommand(driveRightTrajectory);


                output.addSequential(driveLeftCommand);
                output.addSequential(driveRightCommand);
                return output;
        }

        public static CommandGroup DriveStraightForwardAndBack() {
                CommandGroup output = new CommandGroup();

                Path driveForward = new Path(Rotation2.ZERO);
                driveForward.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-180, 0.0)
                        )
                );
                
                Trajectory driveforwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveforwardTrajectory);

                Path driveBackward = new Path(Rotation2.ZERO);
                driveBackward.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(180, 0.0)
                        )
                );
                
                Trajectory drivebackwardTrajectory = new Trajectory(driveBackward, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveBackwardCommand = new AutonomousTrajectoryCommand(drivebackwardTrajectory);

                output.addSequential(driveForwardCommand);
                output.addSequential(driveBackwardCommand);

                return output;
        }


        public static CommandGroup RotateTest(){
                CommandGroup output = new CommandGroup();

                RobotRotateCommand rotateCommand = new RobotRotateCommand(90);
                RobotRotateCommand rotateCommand2 = new RobotRotateCommand(90);
                RobotRotateCommand rotateCommand3 = new RobotRotateCommand(90);
                RobotRotateCommand rotateCommand4 = new RobotRotateCommand(90);

                Path driveForward1 = new Path(Rotation2.ZERO);
                driveForward1.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-30, 0.0)
                        )
                );
                
                Trajectory driveForwardTrajectory1 = new Trajectory(driveForward1, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand1 = new AutonomousTrajectoryCommand(driveForwardTrajectory1);

                Path driveForward2 = new Path(Rotation2.ZERO);
                driveForward2.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-30, 0.0)
                        )
                );
                
                Trajectory driveForwardTrajectory2 = new Trajectory(driveForward2, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand2 = new AutonomousTrajectoryCommand(driveForwardTrajectory2);

                Path driveForward3 = new Path(Rotation2.ZERO);
                driveForward3.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-30, 0.0)
                        )
                );
                
                Trajectory driveForwardTrajectory3 = new Trajectory(driveForward3, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand3 = new AutonomousTrajectoryCommand(driveForwardTrajectory3);

                Path driveForward4 = new Path(Rotation2.ZERO);
                driveForward4.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-30, 0.0)
                        )
                );
            
                Trajectory driveForwardTrajectory4 = new Trajectory(driveForward4, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand4 = new AutonomousTrajectoryCommand(driveForwardTrajectory4);



                output.addSequential(rotateCommand, 2);
                //output.addSequential(driveForwardCommand1);
                output.addSequential(rotateCommand2, 2);
                //output.addSequential(driveForwardCommand4);
                output.addSequential(rotateCommand3, 2);
                //output.addSequential(driveForwardCommand4);
                output.addSequential(rotateCommand4, 2);
                //output.addSequential(driveForwardCommand4);
                return output;
                //-----------------------------------------------------------
                
                // RobotRotateCommand rotateCommand = new RobotRotateCommand(90);
                // output.addSequential(rotateCommand);
                // Path driveToTrenchPath = new Path(Rotation2.ZERO);
                // driveToTrenchPath.addSegment(
                //         new PathLineSegment(
                //                 new Vector2(0.0, 0.0),
                //                 new Vector2(0.0, 0.0)
                //         ),
                //         Rotation2.fromDegrees(-90)
                //         // negative angle = clockwise when viewed from Frank's safe command station
                //         // positive angle = counterclockwise when viewed from Frank's safe command station       
                // );

                
                // PathArcSegment fooFi = PathArcSegment.fromPoints(new Vector2(0.0, 0.0), new Vector2(70.0, -20.0), new Vector2(0, -60.0));
                // Path arcFooFi = new Path(Rotation2.ZERO);
                // arcFooFi.addSegment(fooFi);

                // // Trajectory driveToTrenchTrajectory = new Trajectory(driveToTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);
                // // AutonomousTrajectoryCommand driveToTrenchCommand = new AutonomousTrajectoryCommand(driveToTrenchTrajectory);

                // Trajectory fooFiTrajectory = new Trajectory(arcFooFi, Robot.drivetrainSubsystem.CONSTRAINTS);
                // AutonomousTrajectoryCommand fooFiCommand = new AutonomousTrajectoryCommand(fooFiTrajectory);

                
                // output.addSequential(fooFiCommand);
        
                //return output;
        }

        public static CommandGroup DriveTwoFeetTwice() {
                CommandGroup output = new CommandGroup();
                RobotRotateCommand rotateCommand = new RobotRotateCommand(-90);
                Path driveForward = new Path(Rotation2.ZERO);
                driveForward.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-48, 0.0)
                        )
                );
                
                Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand1 = new AutonomousTrajectoryCommand(driveForwardTrajectory);

                Path driveForward2 = new Path(Rotation2.ZERO);
                driveForward2.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-48, 0.0)
                        )
                );
                
                Trajectory driveForwardTrajectory2 = new Trajectory(driveForward2, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand2 = new AutonomousTrajectoryCommand(driveForwardTrajectory2);

                output.addSequential(driveForwardCommand1);
                output.addSequential(rotateCommand, 2);
                output.addSequential(driveForwardCommand2);

                return output;
        }

        public static CommandGroup GalacticSearchRedPathA() {
                CommandGroup output = new CommandGroup();
                RobotRotateCommand rotateCommand = new RobotRotateCommand(20.8); // before 23.56, which is too far over
                RobotRotateCommand rotateCommand2 = new RobotRotateCommand(-78.988); // before -85.12 when rotateCommand angle was 23.56
                IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 1);

                Path driveForward = new Path(Rotation2.ZERO);
                driveForward.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-29.0, 0.0)
                        )
                );
                
                Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);

                
                Path driveToD5 = new Path(Rotation2.ZERO);
                driveToD5.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-78, 0.0)
                        )
                );
                
                Trajectory driveToD5Trajectory = new Trajectory(driveToD5, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveToD5Command = new AutonomousTrajectoryCommand(driveToD5Trajectory);


                Path driveToA6 = new Path(Rotation2.ZERO);
                driveToA6.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-105, 0.0)
                        )
                );
                
                Trajectory driveToA6Trajectory = new Trajectory(driveToA6, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveToA6Command = new AutonomousTrajectoryCommand(driveToA6Trajectory);

                output.addParallel(lowerIntake);
                output.addParallel(driveForwardCommand);
                output.addSequential(new IntakeDetectToElevatorIndexCommand(4));
                output.addSequential(rotateCommand, 2);
                output.addParallel(driveToD5Command);
                output.addSequential(new IntakeDetectToElevatorIndexCommand(5));
                output.addSequential(rotateCommand2, 2);
                output.addParallel(driveToA6Command);
                output.addSequential(new IntakeDetectToElevatorIndexCommand(8));

                return output; 
        }

        public static CommandGroup GalacticSearchBluePathA() {
                CommandGroup output = new CommandGroup();
                IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false,1);
                RobotRotateCommand rotateCommand0 = new RobotRotateCommand(-71.56+14); 
                RobotRotateCommand rotateCommand1 = new RobotRotateCommand(97.12-24); 
                
                Path driveForward = new Path(Rotation2.ZERO);
                driveForward.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-123.0, 0.0)
                        )
                );
                Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);


                Path driveToB7 = new Path(Rotation2.ZERO);
                driveToB7.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-94.86, 0.0)
                        )
                );
                
                Trajectory driveToB7Trajectory = new Trajectory(driveToB7, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveToB7Command = new AutonomousTrajectoryCommand(driveToB7Trajectory);

                Path driveToC9 = new Path(Rotation2.ZERO);
                driveToC9.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-120.0,0.0)
                        )
                );
                
                Trajectory driveToC9Trajectory = new Trajectory(driveToC9, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveToC9Command = new AutonomousTrajectoryCommand(driveToC9Trajectory);

                // Path driveToEndzone = new Path(Rotation2.ZERO);
                // driveToEndzone.addSegment(
                //         new PathLineSegment(
                //                 new Vector2(0.0,0.0), 
                //                 new Vector2(-67.08,0.0)
                //         )
                // );
                
                // Trajectory driveToEndzoneTrajectory = new Trajectory(driveToEndzone, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                // AutonomousTrajectoryCommand driveToEndzoneCommand = new AutonomousTrajectoryCommand(driveToEndzoneTrajectory);

                output.addParallel(lowerIntake);
                output.addParallel(driveForwardCommand);
                output.addSequential(new IntakeDetectToElevatorIndexCommand(5)); //is 3 seconds enough?
                output.addSequential(rotateCommand0);
                output.addParallel(driveToB7Command);
                output.addSequential(new IntakeDetectToElevatorIndexCommand(5)); //is 3 seconds enough?
                output.addSequential(rotateCommand1);
                output.addParallel(driveToC9Command);
                output.addSequential(new IntakeDetectToElevatorIndexCommand(5));
                return output; 

                // Why dont we make a function that takes the params for movement and such then returns the AutonomousTrajectoryCommand to make the code smaller, just an idea
        }

        public static CommandGroup GalacticSearchRedPathARotate() {
                CommandGroup output = new CommandGroup();
                double firstRotate = 20.8;
                double secondRotate = -78.988;
                IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 1);
                Rotation2 firstRotation = new Rotation2(Math.cos(firstRotate*Math.PI/180), Math.sin(firstRotate*Math.PI/180), true);
                Rotation2 secondRotation = new  Rotation2(Math.cos(secondRotate*Math.PI/180), Math.sin(secondRotate*Math.PI/180), true);

                Path driveForward = new Path(Rotation2.ZERO);
                driveForward.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-29.0, 0.0)
                        )
                );
                
                Trajectory driveForwardTrajectory = new Trajectory(driveForward, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);

                
                Path driveToD5 = new Path(Rotation2.ZERO);
                driveToD5.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-78, 0.0)
                        ), firstRotation
                ); // can add another segment to same path
                
                Trajectory driveToD5Trajectory = new Trajectory(driveToD5, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveToD5Command = new AutonomousTrajectoryCommand(driveToD5Trajectory);


                Path driveToA6 = new Path(Rotation2.ZERO);
                driveToA6.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0,0.0), 
                                new Vector2(-105, 0.0)
                        ), secondRotation
                );
                
                Trajectory driveToA6Trajectory = new Trajectory(driveToA6, Robot.drivetrainSubsystem.AUTONOMOUS_CONTRAINTS);
                AutonomousTrajectoryCommand driveToA6Command = new AutonomousTrajectoryCommand(driveToA6Trajectory);

                output.addParallel(lowerIntake);
                output.addParallel(driveForwardCommand);
                output.addSequential(new IntakeDetectToElevatorIndexCommand(4));
                // output.addSequential(rotateCommand, 2);
                output.addParallel(driveToD5Command);
                output.addSequential(new IntakeDetectToElevatorIndexCommand(5));
                // output.addSequential(rotateCommand2, 2);
                output.addParallel(driveToA6Command);
                output.addSequential(new IntakeDetectToElevatorIndexCommand(8));

                return output; 
        }

        public CommandGroup buildDriveAndCollect(Vector2 startPose, Vector2 endPose)
        {
                CommandGroup driveAndCollect = new CommandGroup();


                return driveAndCollect;
        }


        public static CommandGroup IntakeTest(){
                CommandGroup output = new CommandGroup();
                IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false,1);
                IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,1);
                Path driveForwardPath = new Path(Rotation2.ZERO);
                driveForwardPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-106, 0.0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveForwardTrajectory = new Trajectory(driveForwardPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

                AutonomousTrajectoryCommand driveForwardCommand = new AutonomousTrajectoryCommand(driveForwardTrajectory);

           
                output.addSequential(lowerIntake);
                output.addParallel(driveForwardCommand);
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                output.addSequential(raiseIntake);

                return output;
        }

	public static CommandGroup ShootThenCollectRight(){
                CommandGroup output = new CommandGroup();
                ShooterActuateCommand shooterAcuateCommand = new ShooterActuateCommand(true, 1);
                ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
                ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
                Path driveToTrenchPath = new Path(Rotation2.ZERO);
                driveToTrenchPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-44.63, -67.905) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveToTrenchTrajectory = new Trajectory(driveToTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);

                AutonomousTrajectoryCommand driveToTrenchCommand = new AutonomousTrajectoryCommand(driveToTrenchTrajectory);

                IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false,1);
                IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,1);

                
                output.addSequential(shooterAcuateCommand);
                output.addParallel(elevatorCommand);
                output.addSequential(shooterCommand);
                output.addParallel(lowerIntake);
                output.addSequential(driveToTrenchCommand);
                
                
                //We've reached the trench. Now collect power cell. 
                Path driveThroughTrenchPath = new Path(Rotation2.ZERO);
                driveThroughTrenchPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-106, 0.0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveThroughTrenchTrajectory = new Trajectory(driveThroughTrenchPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

                AutonomousTrajectoryCommand driveThroughTrenchCommand = new AutonomousTrajectoryCommand(driveThroughTrenchTrajectory);

           
        
                output.addParallel(driveThroughTrenchCommand);
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                output.addSequential(raiseIntake);
                return output;
        }

	public static CommandGroup ShootThenCollectLeft(){
                CommandGroup output = new CommandGroup();
                ShooterActuateCommand shooterAcuateCommand = new ShooterActuateCommand(true, 1);
                ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
                ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
                Path driveToLeftTrenchPath = new Path(Rotation2.ZERO);
                driveToLeftTrenchPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-75, 191.8) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveToLeftTrenchTrajectory = new Trajectory(driveToLeftTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);

                AutonomousTrajectoryCommand driveToTrenchCommand = new AutonomousTrajectoryCommand(driveToLeftTrenchTrajectory);

                IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false,1);
                IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,1);

                
                output.addSequential(shooterAcuateCommand);
                output.addParallel(elevatorCommand);
                output.addSequential(shooterCommand);
                output.addParallel(lowerIntake);
                output.addSequential(driveToTrenchCommand);
                
                Path driveToBallPath = new Path(Rotation2.ZERO);
                driveToBallPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-20, 0.0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveToBallTrajectory = new Trajectory(driveToBallPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

                AutonomousTrajectoryCommand driveToBallCommand = new AutonomousTrajectoryCommand(driveToBallTrajectory);

                
                output.addParallel(driveToBallCommand);
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                
                Path driveToNextBallPath = new Path(Rotation2.ZERO);
                driveToNextBallPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(20, 0.0) //FHE:TODO Confirm positive/negative
                        )
                );
                driveToNextBallPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(0.0, 18.3) //FHE:TODO Confirm positive/negative
                        )
                );
                driveToNextBallPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-20, 0.0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveToNextBallTrajectory = new Trajectory(driveToNextBallPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

                AutonomousTrajectoryCommand driveToNextBallCommand = new AutonomousTrajectoryCommand(driveToNextBallTrajectory);

                output.addParallel(driveToNextBallCommand);
                output.addSequential(new IntakeDetectToElevatorIndexCommand());

                output.addSequential(raiseIntake);
                return output;
        }


        public static CommandGroup ShootThenCollectRight_ThenShootAgain(){
                CommandGroup output =  ShootThenCollectRight();
                Path driveBackToShoot = new Path(Rotation2.ZERO);


                driveBackToShoot.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(106, 0.0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveBackToShootTrajectory = new Trajectory(driveBackToShoot, Robot.drivetrainSubsystem.CONSTRAINTS);

                AutonomousTrajectoryCommand driveBackToShootCommand= new AutonomousTrajectoryCommand(driveBackToShootTrajectory);
                output.addSequential(driveBackToShootCommand,3);
                VisionRotationDriveCommand rotateCommand = new VisionRotationDriveCommand(2);
             

                ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
                ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
                output.addSequential(rotateCommand);
                output.addParallel(elevatorCommand);
                output.addSequential(shooterCommand);;
                return output;
        }

        public static CommandGroup ShootFromRight_Of_Optimal_Then_Collect(){
                CommandGroup output =  new CommandGroup();
                IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,2);
                VisionRotationDriveCommand visionRotateCommand = new VisionRotationDriveCommand(2);
                RobotRotateCommand rotateCommand = new RobotRotateCommand(0);
                ShooterActuateCommand shooterAcuateCommand = new ShooterActuateCommand(true, 1);
                ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
                IntakeActuateCommand lowerIntake = new IntakeActuateCommand(false, 2);
                ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );

                output.addParallel(raiseIntake);
                output.addSequential(visionRotateCommand);
                
     
                output.addSequential(shooterAcuateCommand);
                output.addParallel(elevatorCommand);
                output.addSequential(shooterCommand);

                //output.addSequential(rotateCommand);
               
                Path driveToTrenchPath = new Path(Rotation2.ZERO);
                driveToTrenchPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-44.63, 0.0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveToTrenchTrajectory = new Trajectory(driveToTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);

                AutonomousTrajectoryCommand driveToTrenchCommand= new AutonomousTrajectoryCommand(driveToTrenchTrajectory);
                output.addSequential(driveToTrenchCommand);


                output.addParallel(lowerIntake);

                Path driveThroughTrenchPath = new Path(Rotation2.ZERO);
                driveThroughTrenchPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-106, 0.0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveThroughTrenchTrajectory = new Trajectory(driveThroughTrenchPath, Robot.drivetrainSubsystem.INTAKE_CONSTRAINTS);

                AutonomousTrajectoryCommand driveThroughTrenchCommand = new AutonomousTrajectoryCommand(driveThroughTrenchTrajectory);

           
        
                output.addParallel(driveThroughTrenchCommand);
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                output.addSequential(new IntakeDetectToElevatorIndexCommand());
                output.addSequential(new IntakeActuateCommand(true,2));

                return output;

        }

        public static CommandGroup shootThenBackAwayFromInitiationLine(){
                CommandGroup output =  new CommandGroup();
                IntakeActuateCommand raiseIntake = new IntakeActuateCommand(true,2);
                VisionRotationDriveCommand visionRotateCommand = new VisionRotationDriveCommand(2);
                RobotRotateCommand rotateCommand = new RobotRotateCommand(0);
                ShooterActuateCommand shooterAcuateCommand = new ShooterActuateCommand(true, 1);
                ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
                ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );

                output.addParallel(raiseIntake);
                output.addSequential(visionRotateCommand);
                
     
                output.addSequential(shooterAcuateCommand);
                output.addParallel(elevatorCommand);
                output.addSequential(shooterCommand);

                //output.addSequential(rotateCommand);
               
                Path driveAwayPath = new Path(Rotation2.ZERO);
                driveAwayPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-20, 0.0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveAwayTrajectory = new Trajectory(driveAwayPath, Robot.drivetrainSubsystem.CONSTRAINTS);

                AutonomousTrajectoryCommand driveAwayCommand= new AutonomousTrajectoryCommand(driveAwayTrajectory);
                output.addSequential(driveAwayCommand);

                return output;
        }


	public static CommandGroup backAwayFromInitiationLine(){
                CommandGroup output = new CommandGroup();
                Path backAwayPath = new Path(Rotation2.ZERO);
                backAwayPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(-48, 0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory backawayTrajectory = new Trajectory(backAwayPath, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand backAwayCommand = new AutonomousTrajectoryCommand(backawayTrajectory);
                output.addSequential(backAwayCommand, 2);

                return output;

        }

        public static CommandGroup barrelRacing() {
                CommandGroup output = new CommandGroup();
                Path barrelPath = new Path(Rotation2.ZERO);
                // TODO: update values in Vector2's using tangent calculators
                barrelPath.addSegment(
                        new PathLineSegment( // straight line to D5
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around D5
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment( // straight line to B8
                        new PathLineSegment(
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around B8
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment( // straight line to D10
                        new PathLineSegment(
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around D10
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment( // return to start zone
                        new PathLineSegment(
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );

                Trajectory barrelTrajectory = new Trajectory(barrelPath, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand barrelCommand = new AutonomousTrajectoryCommand(barrelTrajectory);
                output.addSequential(barrelCommand); 

                return output; 
        }

        public static CommandGroup slalom() {
                CommandGroup output = new CommandGroup();
                Path barrelPath = new Path(Rotation2.ZERO);
                // TODO: update values in Vector2's using tangent calculators
                barrelPath.addSegment( // avoid D2 start zone cone
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment(
                        new PathLineSegment( // straight line to D4
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around D4
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment( // straight line to D8
                        new PathLineSegment(
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around D8
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment( // straight line to D10
                        new PathLineSegment(
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around D10
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment( // straight line to D8
                new PathLineSegment(
                        new Vector2(0, 0),
                        new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around D8
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment(
                        new PathLineSegment( // straight line to D4
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around D4
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment(
                        new PathLineSegment( // straight line to finish line
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );

                Trajectory barrelTrajectory = new Trajectory(barrelPath, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand barrelCommand = new AutonomousTrajectoryCommand(barrelTrajectory);
                output.addSequential(barrelCommand); 

                return output; 
        }

        public static CommandGroup bounce() {
                CommandGroup output = new CommandGroup();
                Path barrelPath = new Path(Rotation2.ZERO);
                // TODO: update values in Vector2's using tangent calculators
                barrelPath.addSegment( // avoid B2 start zone cone
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment(
                        new PathLineSegment( // straight line to A3
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment(
                        new PathLineSegment( // straight line to B4
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        ), new Rotation2(0, 0, true) // TODO determine robot rotation so it's parallel to B4
                ); 
                barrelPath.addSegment( // loop around B4
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment( // straight line to D5
                        new PathLineSegment(
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around D5
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment( // straight line to A6
                        new PathLineSegment(
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // straight line to D7
                        new PathLineSegment(
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        ), new Rotation2(0, 0, true) // re-orient robot like path to B4
                );
                barrelPath.addSegment( // loop around D7
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment( // straight line to D8
                new PathLineSegment(
                        new Vector2(0, 0),
                        new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment( // loop around D8
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                );
                barrelPath.addSegment(
                        new PathLineSegment( // straight line to A9
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );
                barrelPath.addSegment(
                        new PathLineSegment( // straight line to B10
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        ), new Rotation2(0, 0, true) // reorientation again
                );
                barrelPath.addSegment( // loop around B10
                        new PathArcSegment(
                                new Vector2(0, 0), // start point
                                new Vector2(0, 0), // end point
                                new Vector2(0, 0) // center point
                        ), new Rotation2(0, 0, true) // spin the robot as it loops around
                ); 
                // TODO see if it's possible to drive straight into finish zone and break the plane without looping around B10 (similar to slalom ending)
                barrelPath.addSegment(
                        new PathLineSegment( // straight line to finish line
                                new Vector2(0, 0),
                                new Vector2(0, 0)
                        )
                );

                Trajectory barrelTrajectory = new Trajectory(barrelPath, Robot.drivetrainSubsystem.CONSTRAINTS);
                AutonomousTrajectoryCommand barrelCommand = new AutonomousTrajectoryCommand(barrelTrajectory);
                output.addSequential(barrelCommand); 

                return output; 
        }


       
    //Lifts intake
    //Drives forward 5 inches
    //Spins intake
    //Lowers intake and calls elevator state machine
    //Drives backward 5 inches.
    public static CommandGroup CollectPowerCell(){
        Path fiveInchesPath = new Path(Rotation2.ZERO);
        fiveInchesPath.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(5, 0)
                )
        );


        Trajectory fiveInchesForward = new Trajectory(fiveInchesPath, Robot.drivetrainSubsystem.CONSTRAINTS);


        CommandGroup output = new CommandGroup();
        
        AutonomousTrajectoryCommand trajectoryCommand = new AutonomousTrajectoryCommand(fiveInchesForward);
        IntakeCommand intakeCommand = new IntakeCommand(false);
        output.addParallel(intakeCommand);
        output.addParallel(trajectoryCommand);
	return output;
    }


    public static String getMethodName()
	{
		String methodName = Thread.currentThread().getStackTrace()[2].getMethodName();
		return methodName;
	}
}