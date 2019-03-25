#include "drake/automotive/maliput/base/intersections_loader.h"

#include <unordered_map>
#include <utility>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"

namespace drake {
namespace maliput {
namespace {

using api::rules::LaneSRange;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;







// TODO(liang.fok): Eliminate overlapping regions. This is not required by the
// Intersection constructor's documentation, but it is nice-to-have.
// Currently, the Intersection's region is simply the concatenation of all the
// regions covered by the rules that govern the intersection, and thus may
// have overlaps.
std::vector<LaneSRange> GetRegion(const RightOfWayPhase &phase) {
  std::vector<LaneSRange> result;
  // for (const auto& it : rules) {
  //   const std::vector<LaneSRange>& ranges = it.second.zone().ranges();
  //   result.insert(region.end(), std::begin(ranges), std::end(ranges));
  // }
  return result;
}







    DRAKE_DEMAND(ring.phases().size() > 0);
    result.intersections.push_back(std::make_unique<Intersection>(Intersection::ID(ring.id().string()),
        GetRegion(ring.phases().begin().second), ring.id(), ));












std::unordered_map<RightOfWayRule::Id, RightOfWayRule> GetRules(const RoadRulebook* rulebook, const YAML::Node& rules_node) {
  DRAKE_DEMAND(rules_node.IsSequence());
  std::unordered_map<RightOfWayRule::Id, RightOfWayRule> result;
  for (const YAML::Node& rule_node : rules_node) {
    const RightOfWayRule::Id rule_id(rule_node.as<std::string>());
    result.emplace(std::make_pair(rule_id, rulebook->GetRule(rule_id)));
  }
  return result;
}

std::unique_ptr<const Intersection> BuildIntersection(const api::RoadGeometry* /*road_geometry*/,
    const RoadRulebook* rulebook, const YAML::Node& intersection_node) {
  DRAKE_DEMAND(intersection_node.IsMap());
  const Intersection::Id id(intersection_node["ID"].as<std::string>());
  std::cout << "Processing intersection: " << id.string() << std::endl;    // TEMP!!

  const std::unordered_map<RightOfWayRule::Id, RightOfWayRule> rules = GetRules(rulebook, intersection_node["Rules"]);
  DRAKE_DEMAND(rules.size() > 0);   /// TEMP!!
  std::cout << "Number of rules: " << rules.size() << std::endl;         // TEMP!!

  // To get the region of the Intersection, concatenate all of the regions covered by the rules mentioned that govern the intersection.
  std::vector<LaneSRange> region;
  for (const auto& it : rules) {
    const std::vector<LaneSRange>& ranges = it.second.zone().ranges();
    // TODO(liang.fok) Eliminate overlapping regions. I did not do this because
    // the Intersection constructor's documentation does not require it.
    region.insert(region.end(), std::begin(ranges), std::end(ranges));
  }

  return nullptr;  // std::make_unique<const Intersection>(id, region, nullptr, nullptr);
}

std::vector<std::unique_ptr<const Intersection>> BuildFrom(const api::RoadGeometry* road_geometry,
    const RoadRulebook* rulebook, const YAML::Node& root_node) {
  DRAKE_DEMAND(root_node.IsMap());
  const YAML::Node& intersections_node = root_node["Intersections"];
  DRAKE_DEMAND(intersections_node.IsSequence());
  std::vector<std::unique_ptr<const Intersection>> intersections;
  for (const YAML::Node& intersection_node : intersections_node) {
    intersections.push_back(BuildIntersection(road_geometry, rulebook, intersection_node));
  }
  return intersections;
}

}  // namespace

std::vector<std::unique_ptr<const Intersection>> LoadIntersections(const api::RoadGeometry* road_geometry,
    const RoadRulebook* rulebook, const std::string& input) {
  return BuildFrom(road_geometry, rulebook, YAML::Load(input));
}

/// Instantiates Intersection instances based on the specified @p road_geometry and files.
///
/// @param road_geometry The road geometry that includes the intersections to be loaded.
///
/// @param filename The YAML file that contains an Intersections document.
///
/// @return The loaded Intersection objects.
std::vector<std::unique_ptr<const Intersection>> LoadIntersectionsFromFile(
    const api::RoadGeometry* road_geometry,
    const RoadRulebook* rulebook, const std::string& filename){
  return BuildFrom(road_geometry, rulebook, YAML::LoadFile(filename));
}

}  // namespace maliput
}  // namespace drake
