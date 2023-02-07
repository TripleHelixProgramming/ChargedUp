// Copyright (c) FRC Team 2363. All Rights Reserved.

double currentVelocityNorm = v_norms[index];
    ChassisSpeeds currentVelocityHat = v_hats[index];
    ChassisSpeeds nextVelocityHat = v_hats[index + 1];
    Twist2d delta = poses[index].Log(poses[index + 1]);
    auto maxAllowableSquaredVelocityX = meters_per_second_t{(2 * maxAccelerationX * delta.dx).value() +
                                                            sgn(nextVelocityHat.vx.value()) *
                                                            (currentVelocityNorm * currentVelocityHat.vx).value()};
    meters_per_second_t =
    meters_per_second_t maxAllowableVelocityY = ;
    radians_per_second_t maxAllowableRotationalVelocity = ;

    double v_norm = std::min({maxAllowableVelocityX / v_hats[index].vx,
                              maxAllowableVelocityY / v_hats[index].vy,
                              maxAllowableRotationalVelocity / v_hats[index].omega});
