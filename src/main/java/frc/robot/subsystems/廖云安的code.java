// double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
//     double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
//     double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

//     var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
//         fieldRelative
//             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
//             : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
//     SwerveDriveKinematics.desaturateWheelSpeeds(
//         swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
//     m_frontLeft.setDesiredState(swerveModuleStates[0]);
//     m_frontRight.setDesiredState(swerveModuleStates[1]);
//     m_rearLeft.setDesiredState(swerveModuleStates[2]);
//     m_rearRight.setDesiredState(swerveModuleStates[3]);