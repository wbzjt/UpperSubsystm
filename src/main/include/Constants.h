// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {



inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace UpperConstants {

    constexpr double ShootRatio = 1.0;  // 射球电机减速比
    constexpr double CatchRatio = 30.0; // 抓球电机减速比
    /* 相关控制参数 */
    constexpr double ShootseaweedSpeed = 10.0; // 放置海藻的射球速度




}
