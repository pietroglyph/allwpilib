package edu.wpi.first.wpilibj.estimator;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

public class SwerveDrivePoseEstimator {
    private final KalmanFilter<N3, N3, N3> m_observer;
    private final LinearSystem<N3, N3, N3> m_observerSystem;
    private final KalmanFilterLatencyCompensator<N3, N3, N3> m_latencyCompensator;
    /** Seconds **/
    private final double m_nominalDt;

    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;

    public SwerveDrivePoseEstimator(Rotation2d gyroAngle, Pose2d initialPoseMeters, SwerveDriveKinematics kinematics,
        Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> measurementStdDevs) {
        this(gyroAngle, initialPoseMeters, kinematics, stateStdDevs, measurementStdDevs, 0.01);
    }

    public SwerveDrivePoseEstimator(Rotation2d gyroAngle, Pose2d initialPoseMeters, SwerveDriveKinematics kinematics,
            Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> measurementStdDevs, double nominalDtSeconds) {
        m_nominalDt = nominalDtSeconds;

        m_observerSystem = new LinearSystem<>(
            Nat.N3(), Nat.N3(), Nat.N3(),
            new MatBuilder<>(Nat.N3(), Nat.N3()).fill(

            ),
            new MatBuilder<>(Nat.N3(), Nat.N3()).fill(

            ),
            new MatBuilder<>(Nat.N3(), Nat.N3()).fill(

            ),
            MatrixUtils.zeros(Nat.N3(), Nat.N3()),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill()
        );
        m_observer = new KalmanFilter<>(Nat.N3(), Nat.N3(), Nat.N3(), m_observerSystem, stateStdDevs,
            measurementStdDevs, nominalDtSeconds);
        m_latencyCompensator = new KalmanFilterLatencyCompensator<>();

        m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPoseMeters.getRotation();
        m_observer.setXhat(poseToVector(initialPoseMeters));
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
        m_latencyCompensator.applyPastMeasurement(m_observer, m_nominalDt, poseToVector(visionRobotPose),
                timestampSeconds);
    }

    public Pose2d update(Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
        var angle = gyroAngle.plus(m_gyroOffset);
        var u = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(leftDistanceMeters, rightDistanceMeters,
                angle.minus(m_previousAngle).getRadians());
        m_previousAngle = angle;

        m_latencyCompensator.addObserverState(m_observer, u);
        m_observer.predict(u, m_nominalDt);

        return new Pose2d(m_observer.getXhat(0), m_observer.getXhat(1), new Rotation2d(m_observer.getXhat(2)));
    }

    // TODO: Deduplicate
    private Matrix<N3, N1> poseToVector(Pose2d pose) {
        return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(pose.getTranslation().getX(), pose.getTranslation().getY(),
                pose.getRotation().getRadians());
    }
}
