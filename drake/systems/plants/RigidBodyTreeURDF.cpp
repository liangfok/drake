#include <fstream>
#include <sstream>
#include <string>

#include "drake/systems/plants/RigidBodyTree.h"
#include "joints/DrakeJoints.h"

#include "xmlUtil.h"

using namespace std;
using namespace Eigen;
using namespace tinyxml2;

// todo: rectify this with findLinkId in the class (which makes more
// assumptions)
int findLinkIndex(RigidBodyTree* model, string linkname) {
  int index = -1;
  for (unsigned int i = 0; i < model->bodies.size(); i++) {
    if (linkname.compare(model->bodies[i]->linkname) == 0) {
      index = i;
      break;
    }
  }
  return index;
}

int findLinkIndexByJointName(RigidBodyTree* model, string jointname) {
  int index = -1;
  for (unsigned int i = 0; i < model->bodies.size(); i++) {
    if (model->bodies[i]->hasParent() &&
        jointname.compare(model->bodies[i]->getJoint().getName()) == 0) {
      index = i;
      break;
    }
  }
  return index;
}

RigidBodyFrame::RigidBodyFrame(RigidBodyTree* tree, XMLElement* link_reference,
                               XMLElement* pose, std::string name)
    : name(name), frame_index(0) {
  string linkname = link_reference->Attribute("link");
  body = tree->findLink(linkname);
  if (!body)
    throw runtime_error("couldn't find link %s referenced in frame " + name);

  Vector3d xyz = Vector3d::Zero(), rpy = Vector3d::Zero();
  if (pose) {
    parseVectorAttribute(pose, "xyz", xyz);
    parseVectorAttribute(pose, "rpy", rpy);
  }
  transform_to_body.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
}

void parseInertial(shared_ptr<RigidBody> body, XMLElement* node,
                   RigidBodyTree* model) {
  Isometry3d T = Isometry3d::Identity();

  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) originAttributesToTransform(origin, T);

  XMLElement* mass = node->FirstChildElement("mass");
  if (mass) parseScalarAttribute(mass, "value", body->mass);

  body->com << T(0, 3), T(1, 3), T(2, 3);

  Matrix<double, TWIST_SIZE, TWIST_SIZE> I =
      Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
  I.block(3, 3, 3, 3) << body->mass * Matrix3d::Identity();

  XMLElement* inertia = node->FirstChildElement("inertia");
  if (inertia) {
    parseScalarAttribute(inertia, "ixx", I(0, 0));
    parseScalarAttribute(inertia, "ixy", I(0, 1));
    I(1, 0) = I(0, 1);
    parseScalarAttribute(inertia, "ixz", I(0, 2));
    I(2, 0) = I(0, 2);
    parseScalarAttribute(inertia, "iyy", I(1, 1));
    parseScalarAttribute(inertia, "iyz", I(1, 2));
    I(2, 1) = I(1, 2);
    parseScalarAttribute(inertia, "izz", I(2, 2));
  }

  body->I = transformSpatialInertia(T, I);
}

bool parseMaterial(XMLElement* node, MaterialMap& materials) {
  const char* attr;
  attr = node->Attribute("name");
  if (!attr || strlen(attr) == 0) {
    cerr << "WARNING: material tag is missing a name" << endl;
    return false;
  }
  string name(attr);
  auto material_iter = materials.find(name);
  bool already_in_map = false;
  if (material_iter != materials.end()) {
    already_in_map = true;
  }

  Vector4d rgba;
  XMLElement* color_node = node->FirstChildElement("color");
  if (color_node) {
    if (!parseVectorAttribute(color_node, "rgba", rgba)) {
      cerr << "WARNING: color tag is missing rgba attribute" << endl;
      return false;
    }
    materials[name] = rgba;
  } else if (!already_in_map) {
    cerr << "WARNING: material \"" << name
         << "\" is not a simple color material (so is currently unsupported)"
         << endl;
    return false;
  }
  return true;
}

bool parseGeometry(XMLElement* node, const PackageMap& package_map,
                   const string& root_dir, DrakeShapes::Element& element) {
  // DEBUG
  // cout << "parseGeometry: START" << endl;
  // END_DEBUG
  const char* attr;
  XMLElement* shape_node;
  if ((shape_node = node->FirstChildElement("box"))) {
    double x = 0, y = 0, z = 0;
    attr = shape_node->Attribute("size");
    if (attr) {
      stringstream s(attr);
      s >> x >> y >> z;
    } else {
      cerr << "ERROR parsing box element size" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Box(Vector3d(x, y, z)));
  } else if ((shape_node = node->FirstChildElement("sphere"))) {
    double r = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    } else {
      cerr << "ERROR parsing sphere element radius" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Sphere(max(DrakeShapes::MIN_RADIUS, r)));
  } else if ((shape_node = node->FirstChildElement("cylinder"))) {
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    } else {
      cerr << "ERROR parsing cylinder element radius" << endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    } else {
      cerr << "ERROR parsing cylinder element length" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Cylinder(r, l));
  } else if ((shape_node = node->FirstChildElement("capsule"))) {
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    } else {
      cerr << "ERROR parsing capsule element radius" << endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    } else {
      cerr << "ERROR: Failed to parse capsule element length" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Capsule(r, l));
  } else if ((shape_node = node->FirstChildElement("mesh"))) {
    attr = shape_node->Attribute("filename");
    if (!attr) {
      cerr << "ERROR mesh element has no filename tag" << endl;
      return false;
    }
    string filename(attr);
    string resolved_filename = resolveFilename(filename, package_map, root_dir);
    DrakeShapes::Mesh mesh(filename, resolved_filename);

    attr = shape_node->Attribute("scale");
    if (attr) {
      stringstream s(attr);
      s >> mesh.scale;
    }

    element.setGeometry(mesh);
  } else {
    cerr << "Warning: geometry element has an unknown type and will be ignored."
         << endl;
  }
  // DEBUG
  // cout << "parseGeometry: END" << endl;
  // END_DEBUG
  return true;
}

void parseVisual(shared_ptr<RigidBody> body, XMLElement* node,
                 RigidBodyTree* model, const MaterialMap& materials,
                 const PackageMap& package_map, const string& root_dir) {
  // DEBUG
  // cout << "parseVisual: START" << endl;
  // END_DEBUG
  Isometry3d T_element_to_link = Isometry3d::Identity();
  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) originAttributesToTransform(origin, T_element_to_link);

  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node)
    throw runtime_error("ERROR: Link " + body->linkname +
                        " has a visual element without geometry.");

  DrakeShapes::VisualElement element(T_element_to_link);
  if (!parseGeometry(geometry_node, package_map, root_dir, element))
    throw runtime_error("ERROR: Failed to parse visual element in link " +
                        body->linkname + ".");

  XMLElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    const char* attr;
    attr = material_node->Attribute("name");
    if (attr && strlen(attr) > 0 && materials.find(attr) != materials.end()) {
      element.setMaterial(materials.at(attr));
    } else {
      XMLElement* color_node = material_node->FirstChildElement("color");
      if (color_node) {
        Vector4d rgba;
        if (!parseVectorAttribute(color_node, "rgba", rgba)) {
          cerr << "WARNING: Failed to parse color element rgba in visual"
               << endl;
        } else {
          element.setMaterial(rgba);
        }
      } else {
        cerr << "WARNING: visual element had a material with neither a name "
                "nor a nested color element"
             << endl;
      }
    }
  }

  if (element.hasGeometry()) {
    // DEBUG
    // cout << "parseVisual: Adding element to body" << endl;
    // END_DEBUG
    body->addVisualElement(element);
  }

  // DEBUG
  // cout << "parseVisual: END" << endl;
  // END_DEBUG
}

void parseCollision(shared_ptr<RigidBody> body, XMLElement* node,
                    RigidBodyTree* model, const PackageMap& package_map,
                    const string& root_dir) {
  Isometry3d T_element_to_link = Isometry3d::Identity();
  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) originAttributesToTransform(origin, T_element_to_link);

  const char* attr;
  string group_name;

  attr = node->Attribute("group");
  if (attr) {
    group_name = attr;
  } else {
    group_name = "default";
    ;
  }

  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node)
    throw runtime_error("ERROR: Link " + body->linkname +
                        " has a collision element without geometry");

  RigidBody::CollisionElement element(T_element_to_link, body);
  if (!parseGeometry(geometry_node, package_map, root_dir, element))
    throw runtime_error("ERROR: Failed to parse collision element in link " +
                        body->linkname + ".");

  if (element.hasGeometry()) {
    model->addCollisionElement(element, *body, group_name);
  }
}

void parseLink(RigidBodyTree* model, std::string robot_name, XMLElement* node,
               const MaterialMap& materials, const PackageMap& package_map,
               const string& root_dir) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  shared_ptr<RigidBody> body(new RigidBody());
  body->model_name = robot_name;

  attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: link tag is missing name attribute");

  body->linkname = attr;
  if (body->linkname == "world")
    throw runtime_error(
        "ERROR: do not name a link 'world', it is a reserved name");

  XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node) parseInertial(body, inertial_node, model);

  for (XMLElement* visual_node = node->FirstChildElement("visual"); visual_node;
       visual_node = visual_node->NextSiblingElement("visual")) {
    parseVisual(body, visual_node, model, materials, package_map, root_dir);
  }

  for (XMLElement* collision_node = node->FirstChildElement("collision");
       collision_node;
       collision_node = collision_node->NextSiblingElement("collision")) {
    parseCollision(body, collision_node, model, package_map, root_dir);
  }

  model->bodies.push_back(body);
  body->body_index = static_cast<int>(model->bodies.size()) - 1;
}

template <typename JointType>
void setLimits(XMLElement* node, FixedAxisOneDoFJoint<JointType>* fjoint) {
  XMLElement* limit_node = node->FirstChildElement("limit");
  if (fjoint != nullptr && limit_node) {
    double lower = -numeric_limits<double>::infinity(),
           upper = numeric_limits<double>::infinity();
    parseScalarAttribute(limit_node, "lower", lower);
    parseScalarAttribute(limit_node, "upper", upper);
    fjoint->setJointLimits(lower, upper);
  }
}

template <typename JointType>
void setDynamics(XMLElement* node, FixedAxisOneDoFJoint<JointType>* fjoint) {
  XMLElement* dynamics_node = node->FirstChildElement("dynamics");
  if (fjoint != nullptr && dynamics_node) {
    double damping = 0.0, coulomb_friction = 0.0, coulomb_window = 0.0;
    parseScalarAttribute(dynamics_node, "damping", damping);
    parseScalarAttribute(dynamics_node, "friction", coulomb_friction);
    parseScalarAttribute(dynamics_node, "coulomb_window", coulomb_window);
    fjoint->setDynamics(damping, coulomb_friction, coulomb_window);
  }
}

void parseJoint(RigidBodyTree* model, XMLElement* node) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: joint tag is missing name attribute");
  string name(attr);

  attr = node->Attribute("type");
  if (!attr)
    throw runtime_error("ERROR: joint " + name +
                        " is missing the type attribute");
  string type(attr);

  // parse parent
  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node)
    throw runtime_error("ERROR: joint " + name + " doesn't have a parent node");

  attr = parent_node->Attribute("link");
  if (!attr)
    throw runtime_error("ERROR: joint " + name +
                        " parent does not have a link attribute");
  string parent_name(attr);

  int parent_index = findLinkIndex(model, parent_name);
  if (parent_index < 0)
    throw runtime_error("ERROR: could not find parent link named " +
                        parent_name);

  // parse child
  XMLElement* child_node = node->FirstChildElement("child");
  if (!child_node)
    throw runtime_error("ERROR: joint " + name + " doesn't have a child node");
  attr = child_node->Attribute("link");
  if (!attr)
    throw runtime_error("ERROR: joint " + name +
                        " child does not have a link attribute");
  string child_name(attr);

  int child_index = findLinkIndex(model, child_name);
  if (child_index < 0)
    throw runtime_error("ERROR: could not find child link named " + child_name);

  Isometry3d transform_to_parent_body = Isometry3d::Identity();
  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    originAttributesToTransform(origin, transform_to_parent_body);
  }

  Vector3d axis;
  axis << 1, 0, 0;
  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && type.compare("fixed") != 0 &&
      type.compare("floating") != 0) {
    parseVectorAttribute(axis_node, "xyz", axis);
    if (axis.norm() < 1e-8)
      throw runtime_error("ERROR: axis is zero.  don't do that");
    axis.normalize();
  }

  // now construct the actual joint (based on it's type)
  DrakeJoint* joint = nullptr;

  if (type.compare("revolute") == 0 || type.compare("continuous") == 0) {
    FixedAxisOneDoFJoint<RevoluteJoint>* fjoint =
        new RevoluteJoint(name, transform_to_parent_body, axis);
    setDynamics(node, fjoint);
    setLimits(node, fjoint);
    joint = fjoint;
  } else if (type.compare("fixed") == 0) {
    joint = new FixedJoint(name, transform_to_parent_body);
  } else if (type.compare("prismatic") == 0) {
    FixedAxisOneDoFJoint<PrismaticJoint>* fjoint =
        new PrismaticJoint(name, transform_to_parent_body, axis);
    setDynamics(node, fjoint);
    setLimits(node, fjoint);
    joint = fjoint;
  } else if (type.compare("floating") == 0) {
    joint = new RollPitchYawFloatingJoint(name, transform_to_parent_body);
  } else {
    throw runtime_error("ERROR: Unrecognized joint type: " + type);
  }

  unique_ptr<DrakeJoint> joint_unique_ptr(joint);
  model->bodies[child_index]->setJoint(move(joint_unique_ptr));
  model->bodies[child_index]->parent = model->bodies[parent_index];
}

void parseTransmission(RigidBodyTree* model, XMLElement* node) {
  const char* attr = nullptr;
  XMLElement* type_node = node->FirstChildElement("type");
  if (type_node) {
    attr = type_node->GetText();
  }

  if (!attr) {
    attr = node->Attribute("type");  // old URDF format, kept for convenience
    if (!attr)
      throw runtime_error(
          "ERROR: transmission element is missing the type child");
  }
  string type(attr);
  if (type.find("SimpleTransmission") == string::npos) {
    cerr << "WARNING: only SimpleTransmissions are supported right now.  this "
            "element will be skipped."
         << endl;
    return;
  }

  XMLElement* actuator_node = node->FirstChildElement("actuator");
  if (!actuator_node || !actuator_node->Attribute("name"))
    throw runtime_error("ERROR: transmission is missing an actuator element");
  string actuator_name(actuator_node->Attribute("name"));

  XMLElement* joint_node = node->FirstChildElement("joint");
  if (!joint_node || !joint_node->Attribute("name"))
    throw runtime_error("ERROR: transmission is missing a joint element");
  string joint_name(joint_node->Attribute("name"));

  int body_index = findLinkIndexByJointName(model, joint_name);

  if (model->bodies[body_index]->getJoint().getNumPositions() == 0) {
    cerr << "WARNING: Skipping transmission since it's attached to a fixed "
            "joint: "
         << joint_name << endl;
    return;
  }

  XMLElement* reduction_node = node->FirstChildElement("mechanicalReduction");
  double gain = 1.0;
  if (reduction_node) parseScalarValue(reduction_node, gain);

  XMLElement* limit_node = joint_node->FirstChildElement("limit");
  double effort_max = numeric_limits<double>::infinity();
  double effort_min = -numeric_limits<double>::infinity();
  if (limit_node) {
    parseScalarAttribute(limit_node, "effort", effort_max);
    effort_min = -effort_max;

    // effort_min and effort_max take precedence over effort if they exist
    parseScalarAttribute(limit_node, "effort_min", effort_min);
    parseScalarAttribute(limit_node, "effort_max", effort_max);
  }

  model->actuators.push_back(RigidBodyActuator(
      actuator_name, model->bodies[body_index], gain, effort_min, effort_max));
}

void parseLoop(RigidBodyTree* model, XMLElement* node) {
  Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  if (!node || !node->Attribute("name"))
    throw runtime_error("ERROR: loop is missing a name element");
  string name(node->Attribute("name"));

  XMLElement* link_node = node->FirstChildElement("link1");
  std::shared_ptr<RigidBodyFrame> frameA = allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), model, link_node, link_node,
      name + "FrameA");

  link_node = node->FirstChildElement("link2");
  std::shared_ptr<RigidBodyFrame> frameB = allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), model, link_node, link_node,
      name + "FrameB");

  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && !parseVectorAttribute(axis_node, "xyz", axis))
    throw runtime_error("ERROR parsing loop joint axis");

  model->addFrame(frameA);
  model->addFrame(frameB);
  RigidBodyLoop l(frameA, frameB, axis);
  model->loops.push_back(l);
}

void parseFrame(RigidBodyTree* model, XMLElement* node) {
  const char* frame_name = node->Attribute("name");
  if (!frame_name) throw runtime_error("ERROR parsing Drake frame name");

  std::shared_ptr<RigidBodyFrame> frame = allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), model, node, node,
      frame_name);
  model->addFrame(frame);
}

/**
 * Adds the floating joint that connects a robot model to the rest of the
 * rigid body tree.
 *
 * An exception is thrown if the floating base type is unknown or if there
 * is no robot in the rigid body model to which to add the floating joint.
 *
 * @param model A pointer to the RigidBodyTree model to which to add the
 * floating joint.
 * @param floating_joint_name The name of the floating joint.
 * @param floating_base_type The type of the floating base joint.
 * @param body The existing body in the rigid body tree to which to connect
 * the robot model.
 * @param transform_to_body The transform giving the pose of the robot's model
 * expressed in the body's coordinate frame.
 */
void AddFloatingJoint(RigidBodyTree* model,
  const std::string & floating_joint_name,
  const DrakeJoint::FloatingBaseType floating_base_type,
  const std::shared_ptr<RigidBody> body, const Isometry3d transform_to_body) {

  // Instantes a boolean variable that keeps track of whether a floating
  // joint was aded to the RigidBodyTree. This is needed to determine whether
  // this method is successful.
  bool floating_joint_added = false;

  // Search through all bodies in the rigid body tree. Assume the floating joint
  // should be added to all parent-less bodies in the rigid body tree.
  for (unsigned int i = 1; i < model->bodies.size(); i++) {
    if (model->bodies[i]->parent == nullptr) {

      // We have successfully identified a parent-less body in the RigidBodyTree.
      // Proceed to attach it to the tree using a floating joint.
      model->bodies[i]->parent = body;
      switch (floating_base_type) {
        case DrakeJoint::FIXED: {
          unique_ptr<DrakeJoint> joint(
              new FixedJoint(floating_joint_name, transform_to_body));
          model->bodies[i]->setJoint(move(joint));
        } break;
        case DrakeJoint::ROLLPITCHYAW: {
          unique_ptr<DrakeJoint> joint(new RollPitchYawFloatingJoint(
              floating_joint_name, transform_to_body));
          model->bodies[i]->setJoint(move(joint));
        } break;
        case DrakeJoint::QUATERNION: {
          unique_ptr<DrakeJoint> joint(new QuaternionFloatingJoint(
              floating_joint_name, transform_to_body));
          model->bodies[i]->setJoint(move(joint));
        } break;
        default:
          throw std::runtime_error("unknown floating base type");
      }

      // Record the fact that we've successfully added a floating joint
      // to the rigid body tree.
      floating_joint_added = true;
    }
  }

  // Throws an exception if no floating joints were added to the model.
  if (!floating_joint_added)
    throw std::runtime_error(
      "Failed to add floating joint to the rigid body tree!");
}

/**
 * Adds the floating joint that connects a robot model to the rest of the
 * rigid body tree. If the weld_to_frame parameter
 * is nullptr, simply connect the robot to the world with zero offset.
 *
 * Note that this method should only be called after AddRobot() is called
 * meaning the robot's bodies are in the RigidBodyTree but not yet attached
 * to anything in the tree.
 *
 * An exception is thrown if the floating base type is unknown or if there
 * is no robot in the rigid body model to which to add the floating joint.
 *
 * @param model A pointer to the RigidBodyTree model to which to add the
 * floating joint.
 * @param floating_base_type The type of the floating base joint.
 * @param weld_to_frame The frame to which to which to attach the floating joint.
 * This parameter may be nullptr.
 */
void AddFloatingJoint(RigidBodyTree* model,
  const DrakeJoint::FloatingBaseType floating_base_type,
  const std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr) {

  // The following lines of code instantiates and initializes some important
  // local variables.

  std::string floating_joint_name;
  std::shared_ptr<RigidBody> weld_to_body;
  Isometry3d transform_to_body;

  if (weld_to_frame == nullptr) {
    // If no weld_to_frame parameter is provided, weld the robot to the world.
    weld_to_body = model->bodies[0];
    floating_joint_name = "base";
    transform_to_body = Isometry3d::Identity();
  } else {
    if (weld_to_frame->name.compare("world") == 0) {
      // The robot is being welded to the world. Thus, ignore the "body"
      // variable within weld_to_frame. Instead, only use the transform_to_body
      // variable to initialize the robot at the desired location in the world.
      weld_to_body = model->bodies[0];  // the world's body
      floating_joint_name = "base";
      transform_to_body = weld_to_frame->transform_to_body;
    } else {
      // The robot is being welded to another body in the RigidBodyTree.
      // Use both the body and transform_to_body variables contained
      // within weld_to_frame.
      weld_to_body = weld_to_frame->body;
      transform_to_body = weld_to_frame->transform_to_body;
      floating_joint_name = "weld";
    }
  }

  AddFloatingJoint(model, floating_joint_name, floating_base_type, weld_to_body,
    transform_to_body);
}

/**
 * Adds the floating joint that connects a robot model to the world link in the
 * rigid body tree.
 *
 * Note that this method should only be called after AddRobot() is called
 * meaning the robot's bodies are in the RigidBodyTree but not yet attached
 * to anything in the tree.
 *
 * An exception is thrown if the floating base type is unknown or if there
 * is no robot in the rigid body model to which to add the floating joint.
 *
 * @param model A pointer to the RigidBodyTree model to which to add the
 * floating joint.
 * @param floating_base_type The type of the floating base joint.
 * @param pose_of_model_in_world The frame to which to which to attach the
 * floating joint. This parameter may be nullptr.
 */
void AddFloatingJoint(RigidBodyTree* model,
  const DrakeJoint::FloatingBaseType floating_base_type,
  const Eigen::Isometry3d pose_of_model_in_world = Isometry3d::Identity()) {

  // The name of the floating joint is "base" and model->bodies[0] is the world
  // link in the rigid body tree.
  AddFloatingJoint(model, "base", floating_base_type, model->bodies[0],
    pose_of_model_in_world);
}

/**
 * Adds a robot to the rigid body tree.
 *
 * @param model A pointer to the rigid body tree to which to add the robot.
 * @param node The XML node containing the URDF specifications.
 * @param package_map A map of all the ROS packages available on the local system.
 * This is needed to find meshes stored in the ROS ecosystem.
 * @param root_dir The root directory in which to search for meshes.
 */
void AddRobot(RigidBodyTree* model, XMLElement* node,
                const PackageMap& package_map,
                const string& root_dir) {
  if (!node->Attribute("name"))
    throw runtime_error("Error: your robot must have a name attribute");

  string robotname = node->Attribute("name");

  // parse material elements
  MaterialMap materials;
  for (XMLElement* link_node = node->FirstChildElement("material"); link_node;
       link_node = link_node->NextSiblingElement("material"))
    parseMaterial(link_node, materials);  // accept failed material parsing

  // parse link elements
  for (XMLElement* link_node = node->FirstChildElement("link"); link_node;
       link_node = link_node->NextSiblingElement("link"))
    parseLink(model, robotname, link_node, materials, package_map, root_dir);

  // DEBUG
  // else {
  // cout << "Parsed link" << endl;
  // cout << "model->bodies.size() = " << model->bodies.size() << endl;
  // cout << "model->num_bodies = " << model->num_bodies << endl;
  //}
  // END_DEBUG

  // todo: parse collision filter groups

  // parse joints
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint"))
    parseJoint(model, joint_node);

  // parse transmission elements
  for (XMLElement* transmission_node = node->FirstChildElement("transmission");
       transmission_node;
       transmission_node =
           transmission_node->NextSiblingElement("transmission"))
    parseTransmission(model, transmission_node);

  // parse loop joints
  for (XMLElement* loop_node = node->FirstChildElement("loop_joint"); loop_node;
       loop_node = loop_node->NextSiblingElement("loop_joint"))
    parseLoop(model, loop_node);

  // parse Drake frames
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame"))
    parseFrame(model, frame_node);
}

/**
 * Parses a URDF model of a robot and adds it to the rigid body tree. It uses
 * a pointer to a RigidBodyFrame object to define how the robot is connected to
 * the rigid body tree.
 *
 * @param model A pointer to the rigid body tree to which to add the robot.
 * @param xml_doc The XML document containing the URDF model.
 * @param packag_map Holds the ROS packages in which to search for meshes.
 * @param root_dir The root directory from which to search for meshes.
 * @param floating_base_type The type of joint used to connect the robot
 * to the world.
 * @param weld_to_frame The frame to which the newly added robot is added.
 */
void ParseURDF(RigidBodyTree* model, XMLDocument* xml_doc,
               PackageMap& package_map, const string& root_dir,
               const DrakeJoint::FloatingBaseType floating_base_type,
               std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr) {
  populatePackageMap(package_map);
  XMLElement* node = xml_doc->FirstChildElement("robot");
  if (!node) {
    throw std::runtime_error("ERROR: This urdf does not contain a robot tag");
  }

  AddRobot(model, node, package_map, root_dir);
  AddFloatingJoint(model, floating_base_type, weld_to_frame);

  model->compile();
}

/**
 * Parses a URDF model of a robot and adds it to the rigid body tree. It uses
 * an Eigen::Isometry3d object to define how the robot is connected to the
 * rigid body tree's word.
 *
 * @param model A pointer to the rigid body tree to which to add the robot.
 * @param xml_doc The XML document containing the URDF model.
 * @param packag_map Holds the ROS packages in which to search for meshes.
 * @param root_dir The root directory from which to search for meshes.
 * @param floating_base_type The type of joint used to connect the robot
 * to the world.
 * @param pose_of_model_in_world Transform giving the pose of the file's
 * model frame expressed in the world frame. By default this is identity.
 */
void ParseURDF(RigidBodyTree* model, XMLDocument* xml_doc,
               PackageMap& package_map, const string& root_dir,
               const DrakeJoint::FloatingBaseType floating_base_type,
               const Eigen::Isometry3d pose_of_model_in_world
                 = Isometry3d::Identity()) {
  populatePackageMap(package_map);
  XMLElement* node = xml_doc->FirstChildElement("robot");
  if (!node) {
    throw std::runtime_error("ERROR: This urdf does not contain a robot tag");
  }

  AddRobot(model, node, package_map, root_dir);
  AddFloatingJoint(model, floating_base_type, pose_of_model_in_world);

  model->compile();
}

int RigidBodyTree::AddRobotFromURDFString(
    const string& urdf_string,
    const string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {

  // Instantiates a temporary PackageMap object.
  PackageMap package_map;

  // Calls the other AddRobotFromURDFString() method that accepts a
  // package_map parameter.
  return AddRobotFromURDFString(urdf_string, package_map, root_dir,
    floating_base_type, weld_to_frame);
}

int RigidBodyTree::AddRobotFromURDFString(
    const string& urdf_string,
    PackageMap& package_map,
    const string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {

  // Parses the XML string.
  XMLDocument xml_doc;
  xml_doc.Parse(urdf_string.c_str());

  // Parses the URDF file and adds the robot to the rigid body tree.
  ParseURDF(this, &xml_doc, package_map, root_dir, floating_base_type,
            weld_to_frame);

  // Returns 1 because one robot was added to this rigid body tree.
  return 1;
}

int RigidBodyTree::AddRobotFromURDF(
    const string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {

  // Instantiates a temporary PackageMap object.
  PackageMap package_map;

  // Calls the other AddRobotFromURDF() method that accepts a package_map
  // parameter.
  return AddRobotFromURDF(urdf_filename, package_map, floating_base_type,
                   weld_to_frame);
}

int RigidBodyTree::AddRobotFromURDF(
    const string& urdf_filename,
    PackageMap& package_map,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {

  // Loads the URDF file.
  XMLDocument xml_doc;
  xml_doc.LoadFile(urdf_filename.data());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error("failed to parse xml in file " + urdf_filename +
                             "\n" + xml_doc.ErrorName());
  }

  // Computes the root directory
  string root_dir = ".";
  size_t found = urdf_filename.find_last_of("/\\");
  if (found != string::npos) {
    root_dir = urdf_filename.substr(0, found);
  }

  // Parses the URDF file and adds the robot to the rigid body tree.
  ParseURDF(this, &xml_doc, package_map, root_dir, floating_base_type,
            weld_to_frame);

  // Returns 1 because one robot was added to this rigid body tree.
  return 1;
}

int RigidBodyTree::AddRobotFromURDFStringIsometry3dPose(
    const string& urdf_string,
    const string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    const Eigen::Isometry3d pose_of_model_in_world) {

  // Instantiates a temporary PackageMap object.
  PackageMap package_map;

  // Calls the other AddRobotFromURDFString() method that accepts a
  // package_map parameter.
  return AddRobotFromURDFStringIsometry3dPose(urdf_string, package_map,
    root_dir, floating_base_type, pose_of_model_in_world);
}

int RigidBodyTree::AddRobotFromURDFStringIsometry3dPose(
    const string& urdf_string,
    PackageMap& package_map,
    const string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    const Eigen::Isometry3d pose_of_model_in_world) {

  // Parses the XML string.
  XMLDocument xml_doc;
  xml_doc.Parse(urdf_string.c_str());

  // Parses the URDF file and adds the robot to the rigid body tree.
  ParseURDF(this, &xml_doc, package_map, root_dir, floating_base_type,
            pose_of_model_in_world);

  // Returns 1 because one robot was added to this rigid body tree.
  return 1;
}

int RigidBodyTree::AddRobotFromURDFIsometry3dPose(
    const string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    const Eigen::Isometry3d pose_of_model_in_world) {

  // Instantiates a temporary PackageMap object.
  PackageMap package_map;

  // Calls the other AddRobotFromURDF() method that accepts a package_map
  // parameter.
  return AddRobotFromURDFIsometry3dPose(urdf_filename, package_map,
    floating_base_type, pose_of_model_in_world);
}

int RigidBodyTree::AddRobotFromURDFIsometry3dPose(
    const string& urdf_filename,
    PackageMap& package_map,
    const DrakeJoint::FloatingBaseType floating_base_type,
    const Eigen::Isometry3d pose_of_model_in_world) {

  // Loads the URDF file.
  XMLDocument xml_doc;
  xml_doc.LoadFile(urdf_filename.data());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error("failed to parse xml in file " + urdf_filename +
                             "\n" + xml_doc.ErrorName());
  }

  // Computes the root directory
  string root_dir = ".";
  size_t found = urdf_filename.find_last_of("/\\");
  if (found != string::npos) {
    root_dir = urdf_filename.substr(0, found);
  }

  // Parses the URDF file and adds the robot to the rigid body tree.
  ParseURDF(this, &xml_doc, package_map, root_dir, floating_base_type,
            pose_of_model_in_world);

  // Returns 1 because one robot was added to this rigid body tree.
  return 1;
}
