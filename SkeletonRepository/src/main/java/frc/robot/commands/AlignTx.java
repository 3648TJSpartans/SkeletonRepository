package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class AlignTx extends Command{

    protected static double kP;
    protected  static double kD;
    protected  static double maxSpeed = 1.5;
    protected  static double maxAcceleration = 1.5;
    protected  final ProfiledPIDController xController; 
    protected  Drive drive;
    protected  Vision vision;
    protected  int pipeline;
    protected  Supplier<Rotation2d> txSupplier;
    protected  int cameraIndex;
    public AlignTx(Drive drive, Vision vision, int cameraIndex, int pipeline, Supplier<Rotation2d> txSupplier){
        this.drive = drive;
        this.vision = vision;
        this.pipeline = pipeline;
        this.txSupplier = txSupplier;
        xController = new ProfiledPIDController(
            kP, 0.0, kD, new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration), 0.02);
    }
    public AlignTx(Drive drive, Vision vision, int pipeline,int cameraIndex){
        this(drive,vision, cameraIndex, pipeline, ()->vision.getTargetX(cameraIndex));
    }
    @Override
    public void initialize(){
        vision.setPipeline(pipeline, cameraIndex);
    }
    @Override
    public void execute(){
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed(),0.0, 0.0, drive.getRotation()));
    }
    public double xSpeed(){
        return xController.calculate(txSupplier.get().getRadians());
    }
}