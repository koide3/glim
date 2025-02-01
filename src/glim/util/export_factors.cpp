#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/base/serialization.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/rotate_vector3_factor.hpp>

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel::Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel::Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Base, "gtsam::noiseModel::Base");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam::noiseModel::Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam::noiseModel::Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam::noiseModel::Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam::SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam::SharedDiagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedIsotropic, "gtsam::SharedIsotropic");

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam::noiseModel::Robust");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base, "gtsam::noiseModel::mEstimator::Base");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null, "gtsam::noiseModel::mEstimator::Null");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair, "gtsam::noiseModel::mEstimator::Fair");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber, "gtsam::noiseModel::mEstimator::Huber");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam::noiseModel::mEstimator::Tukey");

GTSAM_VALUE_EXPORT(gtsam::Rot3);
GTSAM_VALUE_EXPORT(gtsam::Pose3);
GTSAM_VALUE_EXPORT(gtsam::Vector3);
GTSAM_VALUE_EXPORT(gtsam::imuBias::ConstantBias);

BOOST_CLASS_EXPORT_GUID(gtsam::GaussianFactor, "gtsam::GaussianFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor, "gtsam::HessianFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::NonlinearFactor, "gtsam::NonlinearFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::LinearContainerFactor, "gtsam::LinearContainerFactor");

BOOST_CLASS_EXPORT_GUID(gtsam::NoiseModelFactor, "gtsam::NoiseModelFactor");

BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose3>, "gtsam::PriorFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Vector3>, "gtsam::PriorFactor<gtsam::Vector3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>, "gtsam::PriorFactor<gtsam::imuBias::ConstantBias>");

BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose3>, "gtsam::BetweenFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Vector3>, "gtsam::BetweenFactor<gtsam::Vector3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>, "gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>");

BOOST_CLASS_EXPORT_GUID(gtsam::PoseTranslationPrior<gtsam::Pose3>, "gtsam::PoseTranslationPrior<gtsam::Pose3>");

BOOST_CLASS_EXPORT_GUID(gtsam::ImuFactor, "gtsam::ImuFactor");

BOOST_CLASS_EXPORT_GUID(gtsam_points::LinearDampingFactor, "gtsam_points::LinearDampingFactor");
BOOST_CLASS_EXPORT_GUID(gtsam_points::RotateVector3Factor, "gtsam_points::RotateVector3Factor");
