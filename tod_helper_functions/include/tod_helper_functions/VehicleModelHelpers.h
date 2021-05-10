// Copyright 2020 Schimpe
#pragma once
#include <cmath>
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"

namespace VehicleModelHelpers {

inline double swa2rwa(const double steeringWheelAngle,
                      const double maxSteeringWheelAngle,
                      const double maxRoadWheelAngle) {
    return steeringWheelAngle / (maxSteeringWheelAngle / maxRoadWheelAngle);
}

inline double rwa2swa(const double roadWheelAngle,
                      const double maxSteeringWheelAngle,
                      const double maxRoadWheelAngle) {
    return roadWheelAngle * (maxSteeringWheelAngle / maxRoadWheelAngle);
}

inline void calc_vehicle_front_edges(const double xpos, const double ypos, const double theta,
                                     const double distanceFront, const double width,
                                     double &xfl, double &yfl, double &xfr, double &yfr) {
    xfl = xpos + distanceFront * std::cos(float(theta)) - width * std::sin(float(theta)) / 2.0;
    yfl = ypos + distanceFront * std::sin(float(theta)) + width * std::cos(float(theta)) / 2.0;
    xfr = xpos + distanceFront * std::cos(float(theta)) + width * std::sin(float(theta)) / 2.0;
    yfr = ypos + distanceFront * std::sin(float(theta)) - width * std::cos(float(theta)) / 2.0;
}

inline void calc_vehicle_rear_edges(const double xpos, const double ypos, const double theta,
                                    const double distanceRear, const double width,
                                    double &xrl, double &yrl, double &xrr, double &yrr) {
    xrl = xpos - distanceRear * std::cos(float(theta)) - width * std::sin(float(theta)) / 2.0;
    yrl = ypos - distanceRear * std::sin(float(theta)) + width * std::cos(float(theta)) / 2.0;
    xrr = xpos - distanceRear * std::cos(float(theta)) + width * std::sin(float(theta)) / 2.0;
    yrr = ypos - distanceRear * std::sin(float(theta)) - width * std::cos(float(theta)) / 2.0;
}

inline void calc_global_pose_from_local(const double xGlobalIn, const double yGlobalIn, const double thetaGlobalIn,
                                        const double relPosInChildFrameX, const double relPosInChildFrameY,
                                        float &xGlobalOut, float &yGlobalOut) {
    xGlobalOut = xGlobalIn  + relPosInChildFrameX * std::cos(float(thetaGlobalIn))
                            - relPosInChildFrameY * std::sin(float(thetaGlobalIn));
    yGlobalOut = yGlobalIn  + relPosInChildFrameX * std::sin(float(thetaGlobalIn))
                            + relPosInChildFrameY * std::cos(float(thetaGlobalIn));
}

inline void calc_global_pose_from_local(const geometry_msgs::Point in, const double thetaGlobalIn,
                                        const geometry_msgs::Point32 relIn, geometry_msgs::Point32& out) {
    out.x = in.x  + relIn.x * std::cos(float(thetaGlobalIn))
                            - relIn.y * std::sin(float(thetaGlobalIn));
    out.y = in.y  + relIn.x * std::sin(float(thetaGlobalIn))
                            + relIn.y * std::cos(float(thetaGlobalIn));
}

inline void calc_global_pose_from_local(const geometry_msgs::Point in, const double thetaGlobalIn,
                                        const geometry_msgs::Point relIn, geometry_msgs::Point& out) {
    out.x = in.x  + relIn.x * std::cos(float(thetaGlobalIn))
                            - relIn.y * std::sin(float(thetaGlobalIn));
    out.y = in.y  + relIn.x * std::sin(float(thetaGlobalIn))
                            + relIn.y * std::cos(float(thetaGlobalIn));
}

inline double curvature_to_rwa(const double& wheel_base, const double& kappa) {
    return atan(wheel_base * kappa);
}

inline double rwa_to_curvature(const double& wheel_base, const double& rwa) {
    return tan(rwa) / wheel_base;
}

inline double deg2rad(const double deg) { return deg * 3.1415 / 180.0; }
inline double rad2deg(const double rad) { return rad * 180.0 / 3.1415; }
inline double mps2kph(const double mps) { return mps * 3.60; }
inline double kph2mps(const double kph) { return kph / 3.60; }

}; // namespace VehicleModelHelpers
