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

public class AlignTxDy extends AlignTx{
    private final Supplier<Double> dySupplier;
    private final ProfiledPIDController yController; 
    public AlignTxDy(Drive drive, Vision vision, int cameraIndex, int pipeline, Supplier<Rotation2d> txSupplier, Supplier<Double> dySupplier){
        super(drive, vision,cameraIndex,pipeline,txSupplier);
        this.dySupplier = dySupplier;
        yController =  new ProfiledPIDController(
            kP, 0.0, kD, new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration), 0.02);
        yController.setGoal(0);
    }
    public AlignTxDy(Drive drive, Vision vision, int pipeline,int cameraIndex, Supplier<Double> dySupplier){
        this(drive,vision, cameraIndex, pipeline, ()->vision.getTargetX(cameraIndex), dySupplier);
    }
    @Override
    public void execute(){
        double ySpeed = yController.calculate(dySupplier.get());
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed(),ySpeed, 0.0, drive.getRotation()));
    }
}