#include <iostream>

#include <gtsam/base/serialization.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_ext/factors/loose_prior_factor.hpp>
#include <gtsam_ext/factors/rotate_vector3_factor.hpp>

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam::noiseModel::Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam::noiseModel::Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam::SharedNoiseModel");

GTSAM_VALUE_EXPORT(gtsam::Pose3);
GTSAM_VALUE_EXPORT(gtsam::Vector3);
GTSAM_VALUE_EXPORT(gtsam::imuBias::ConstantBias);

BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose3>, "gtsam::PriorFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Vector3>, "gtsam::PriorFactor<gtsam::Vector3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>, "gtsam::PriorFactor<gtsam::imuBias::ConstantBias>");

BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose3>, "gtsam::BetweenFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Vector3>, "gtsam::BetweenFactor<gtsam::Vector3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>, "gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>");

BOOST_CLASS_EXPORT_GUID(gtsam::ImuFactor, "gtsam::ImuFactor");

BOOST_CLASS_EXPORT_GUID(gtsam_ext::LoosePriorFactor<gtsam::Pose3>, "gtsam_ext::LoosePriorFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam_ext::RotateVector3Factor, "gtsam_ext::RotateVector3Factor");
