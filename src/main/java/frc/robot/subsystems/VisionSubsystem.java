package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final NetworkTable limelightTable;
    
    // Confidence thresholds
    private static final double MIN_TARGET_AREA = 0.5;  // Minimum area to trust
    private static final int MIN_TAGS_FOR_CORRECTION = 2;  // Need at least 2 tags
    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        // Configure Limelight for MegaTag2
        setPipeline(0);  // Make sure MegaTag2 pipeline is active
    }
    
    @Override
    public void periodic() {
        // This runs 50 times per second
        updateOdometryWithVision();
        
        // Display on dashboard
        Pose2d pose = drivetrain.getPose();
        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        SmartDashboard.putNumber("Robot Angle", pose.getRotation().getDegrees());
    }
    
    /**
     * Update odometry with vision measurements
     */
    private void updateOdometryWithVision() {
        // Check if we have valid vision data
        if (!hasValidVisionData()) {
            return;  // Use odometry only
        }
        
        // Get vision pose from MegaTag2
        Pose2d visionPose = getVisionPose();
        
        if (visionPose != null) {
            // Get number of tags seen
            int numTags = getNumberOfTags();
            
            // Calculate trust level based on number of tags
            double visionTrust = calculateVisionTrust(numTags);
            
            // Apply vision correction with fusion
            applyVisionMeasurement(visionPose, visionTrust);
        }
    }
    
    /**
     * Check if vision data is reliable
     */
    private boolean hasValidVisionData() {
        double tv = limelightTable.getEntry("tv").getDouble(0);
        return tv == 1.0;  // Has valid target
    }
    
    /**
     * Get robot pose from MegaTag2
     */
    private Pose2d getVisionPose() {
        // MegaTag2 provides botpose in field coordinates
        double[] botpose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        
        if (botpose.length >= 6) {
            // botpose format: [x, y, z, roll, pitch, yaw]
            double x = botpose[0];  // meters
            double y = botpose[1];  // meters
            double yaw = botpose[5];  // degrees
            
            return new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        }
        
        return null;
    }
    
    /**
     * Get number of AprilTags seen
     */
    private int getNumberOfTags() {
        // MegaTag2 provides tag count
        return (int) limelightTable.getEntry("botpose_tagcount").getDouble(0);
    }
    
    /**
     * Calculate how much to trust vision based on tag count
     */
    private double calculateVisionTrust(int numTags) {
        switch (numTags) {
            case 0:
            case 1:
                return 0.1;  // Low trust (10%)
            case 2:
                return 0.7;  // Good trust (70%)
            case 3:
            default:
                return 0.95; // High trust (95%)
        }
    }
    
    /**
     * Fuse vision measurement with odometry
     */
    private void applyVisionMeasurement(Pose2d visionPose, double visionTrust) {
        // Get current odometry pose
        Pose2d odometryPose = drivetrain.getPose();
        
        // Weighted average: blend odometry and vision
        double odometryWeight = 1.0 - visionTrust;
        
        double fusedX = (odometryPose.getX() * odometryWeight) + (visionPose.getX() * visionTrust);
        double fusedY = (odometryPose.getY() * odometryWeight) + (visionPose.getY() * visionTrust);
        
        double odometryAngle = odometryPose.getRotation().getRadians();
        double visionAngle = visionPose.getRotation().getRadians();
        
        // Find smallest angular difference (wraps correctly)
        double angleDiff = Math.IEEEremainder(visionAngle - odometryAngle, 2 * Math.PI);
    
        // Weighted blend
        double fusedAngle = odometryAngle + (angleDiff * visionTrust);
        
        Rotation2d fusedRotation = new Rotation2d(fusedAngle);
        
        Pose2d fusedPose = new Pose2d(fusedX, fusedY, fusedRotation);
        
        // Reset odometry to fused pose
        drivetrain.seedFieldRelative(fusedPose);
        
        // Log for debugging
        SmartDashboard.putNumber("Vision Trust", visionTrust);
        SmartDashboard.putNumber("Vision X", visionPose.getX());
        SmartDashboard.putNumber("Vision Y", visionPose.getY());
    }
    
    /**
     * Set Limelight pipeline
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    // Manually reset pose (for autonomous start)
    public void resetPose(Pose2d pose) {
        drivetrain.seedFieldRelative(pose);
    }
}