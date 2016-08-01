#pragma once

#include <map>
#include <string>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/xmlUtil.h"
#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"

namespace drake {
namespace parsers {
namespace sdf {

// // TODO(liang.fok) Replace this method with one that returns a Model object.
// //
// /// Adds a SDF model to a rigid body system. The model's frame is equal to the
// /// world's coordinate frame.
// ///
// /// @param[in] urdf_filename The URDF file containing the model to be added.
// ///
// /// @param[in] model_name The model within the SDF to add. Throws a
// /// std::runtime_error if no such model exists.
// ///
// /// @param[in] package_map A map of ROS package names to their paths. These are
// /// the packages to search through when finding files referenced in the URDF.
// ///
// /// @param[in] floating_base_type The type of joint that connects the model's
// /// root to the existing rigid body tree.
// ///
// /// @param[out] tree The rigid body tree to which to add the model.
// #ifndef SWIG
//   DRAKE_DEPRECATED("Please use drake::parsers::sdf::AddModelFromSDF.")
// #endif
// DRAKERBM_EXPORT
// void AddRobotFromSDFInWorldFrame(
//     const std::string& sdf_filename,
//     const std::string& model_name,
//     const DrakeJoint::FloatingBaseType floating_base_type, RigidBodyTree* tree);

// // TODO(liang.fok) Replace this method with one that returns a Model object.
// //
// /// Adds a SDF model to a rigid body system.
// ///
// /// @param[in] urdf_filename The URDF file containing the model to be added.
// ///
// /// @param[in] model_name The name of the model within the SDF to add. Throws a
// /// std::runtime_error if no such model exists.
// ///
// /// @param[in] package_map A map of ROS package names to their paths. These are
// /// the packages to search through when finding files referenced in the URDF.
// ///
// /// @param[in] floating_base_type The type of joint that connects the model's
// /// root to the existing rigid body tree.
// ///
// /// @param[in] weld_to_frame The frame to which to connect the new model.
// ///
// /// @param[out] tree The rigid body tree to which to add the model.
// // #ifndef SWIG
// //   DRAKE_DEPRECATED("Please use drake::parsers::sdf::AddModelFromSDF.")
// // #endif
// // DRAKERBM_EXPORT
// // void AddRobotFromSDF(const std::string& sdf_filename,
// //                      const std::string& model_name,
// //                      const DrakeJoint::FloatingBaseType floating_base_type,
// //                      std::shared_ptr<RigidBodyFrame> weld_to_frame,
// //                      RigidBodyTree* tree);

/**
 * Adds a SDF model to a rigid body system. Note that only one model is added
 * even if multiple modes are defined in the SDF. The model that is added is
 * specified by @p model_name.
 *
 * @param[in] filename The SDF file containing the model to be added.
 *
 * @param[in] model_name The name of the model within the SDF to add. Throws a
 * std::runtime_error if no such model exists.
 *
 * @param[in] model_instance_name The instance name of the model. This must be
 * unique among all model instances in this `RigidBodySystem`. Throws a
 * std::runtime_error a model instance already exists with this name.
 *
 * @param[in] package_map A map of ROS package names to their paths. These are
 * the packages to search through when finding files referenced in the URDF.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 */
DRAKERBM_EXPORT
void AddModelFromSDF(const std::string& filename,
                     const std::string& model_name,
                     const std::string& model_instance_name,
                     const DrakeJoint::FloatingBaseType floating_base_type,
                     std::shared_ptr<RigidBodyFrame> weld_to_frame,
                     RigidBodyTree* tree);

}  // namespace sdf
}  // namespace parsers
}  // namespace drake
