#pragma once

enum ControlMode{
    DIRECT_CONTROL_MODE                 = 0,
    SHARED_CONTROL_MODE                 = 1,
    WAYPOINT_CONTROL_MODE               = 2,
    TRAJECTORY_CONTROL_MODE             = 3,
    PERCEPTION_MODIFICATION_CONTROL_MODE    = 4,
    CLOTHOID_CONTROL_MODE               = 5,
    NO_CONTROL_MODE                     = 99,
};

enum VideoRateControlMode{
    SINGLE                              = 0,
    COLLECTIVE                          = 1,
    AUTOMATIC                           = 2,
};
