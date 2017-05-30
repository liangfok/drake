#pragma once

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// DiagramBuilder is a factory class for Diagram. It collects the dependency
/// graph of constituent systems, and topologically sorts them. It is single
/// use: after calling Build or BuildInto, DiagramBuilder gives up ownership
/// of the constituent systems, and should therefore be discarded.
///
/// A system must be added to the DiagramBuilder with AddSystem before it can
/// be wired up in any way. Every system must have a unique, non-empty name.
template <typename T>
class DiagramBuilder {
 public:
  // DiagramBuilder objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramBuilder)

  DiagramBuilder() {}
  virtual ~DiagramBuilder() {}

  /// Takes ownership of @p system and adds it to the builder. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// Diagram built by this builder.
  ///
  /// If the system's name is unset, sets it to System::GetMemoryObjectName()
  /// as a default in order to have unique names within the diagram.
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.AddSystem(std::make_unique<Foo<T>>());
  /// @endcode
  ///
  /// @tparam S The type of system to add.
  template<class S>
  S* AddSystem(std::unique_ptr<S> system) {
    if (system->get_name().empty()) {
      system->set_name(system->GetMemoryObjectName());
    }
    S* raw_sys_ptr = system.get();
    systems_.insert(raw_sys_ptr);
    registered_systems_.push_back(std::move(system));
    return raw_sys_ptr;
  }

  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the Diagram built by this
  /// builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   auto foo = builder.AddSystem<Foo<double>>("name", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo<T>>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  ///
  /// @tparam S The type of System to construct. Must subclass System<T>.
  template<class S, typename... Args>
  S* AddSystem(Args&&... args) {
    return AddSystem(std::make_unique<S>(std::forward<Args>(args)...));
  }

  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the Diagram built by this
  /// builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   // Foo must be a template.
  ///   auto foo = builder.AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  /// @tparam S A template for the type of System to construct. The template
  /// will be specialized on the scalar type T of this builder.
  template<template<typename Scalar> class S, typename... Args>
  S<T>* AddSystem(Args&&... args) {
    return AddSystem(std::make_unique<S<T>>(std::forward<Args>(args)...));
  }

  /// Returns the list of contained Systems.
  std::vector<systems::System<T>*> GetMutableSystems() {
    std::vector<systems::System<T>*> result;
    result.reserve(registered_systems_.size());
    for (const auto& system : registered_systems_) {
      result.push_back(system.get());
    }
    return result;
  }

  /// Declares that input port @p dest is connected to output port @p src.
  void Connect(const OutputPort<T>& src,
               const InputPortDescriptor<T>& dest) {
    DRAKE_DEMAND(src.size() == dest.size());
    PortIdentifier dest_id{dest.get_system(), dest.get_index()};
    PortIdentifier src_id{&src.get_system(), src.get_index()};
    ThrowIfInputAlreadyWired(dest_id);
    ThrowIfSystemNotRegistered(&src.get_system());
    ThrowIfSystemNotRegistered(dest.get_system());
    dependency_graph_[dest_id] = src_id;
  }

  /// Declares that sole input port on the @p dest system is connected to sole
  /// output port on the @p src system.  Throws an exception if the sole-port
  /// precondition is not met (i.e., if @p dest has no input ports, or @p dest
  /// has more than one input port, or @p src has no output ports, or @p src
  /// has more than one output port).
  void Connect(const System<T>& src, const System<T>& dest) {
    DRAKE_THROW_UNLESS(src.get_num_output_ports() == 1);
    DRAKE_THROW_UNLESS(dest.get_num_input_ports() == 1);
    Connect(src.get_output_port(0), dest.get_input_port(0));
  }

  /// Cascades @p src and @p dest.  The sole input port on the @p dest system
  /// is connected to sole output port on the @p src system.  Throws an
  /// exception if the sole-port precondition is not met (i.e., if @p dest has
  /// no input ports, or @p dest has more than one input port, or @p src has no
  /// output ports, or @p src has more than one output port).
  void Cascade(const System<T>& src, const System<T>& dest) {
    Connect(src, dest);
  }

  /// Declares that the given @p input port of a constituent system is an input
  /// to the entire Diagram.
  /// @return The index of the exported input port of the entire diagram.
  int ExportInput(const InputPortDescriptor<T>& input) {
    PortIdentifier id{input.get_system(), input.get_index()};
    ThrowIfInputAlreadyWired(id);
    ThrowIfSystemNotRegistered(input.get_system());
    int return_id = static_cast<int>(input_port_ids_.size());
    input_port_ids_.push_back(id);
    diagram_input_set_.insert(id);
    return return_id;
  }

  /// Declares that the given @p output port of a constituent system is an
  /// output of the entire diagram.
  /// @return The index of the exported output port of the entire diagram.
  int ExportOutput(const OutputPort<T>& output) {
    ThrowIfSystemNotRegistered(&output.get_system());
    int return_id = static_cast<int>(output_port_ids_.size());
    output_port_ids_.push_back(
        PortIdentifier{&output.get_system(), output.get_index()});
    return return_id;
  }

  /// Builds the Diagram that has been described by the calls to Connect,
  /// ExportInput, and ExportOutput. Throws std::logic_error if the graph is
  /// not buildable.
  std::unique_ptr<Diagram<T>> Build() {
    std::unique_ptr<Diagram<T>> diagram(new Diagram<T>(Compile()));
    diagram->Own(std::move(registered_systems_));
    return std::move(diagram);
  }

  /// Configures @p target to have the topology that has been described by
  /// the calls to Connect, ExportInput, and ExportOutput. Throws
  /// std::logic_error if the graph is not buildable.
  ///
  /// Only Diagram subclasses should call this method. The target must not
  /// already be initialized.
  void BuildInto(Diagram<T>* target) {
    target->Initialize(Compile());
    target->Own(std::move(registered_systems_));
  }

 private:
  typedef typename Diagram<T>::PortIdentifier PortIdentifier;

  void ThrowIfInputAlreadyWired(const PortIdentifier& id) const {
    if (dependency_graph_.find(id) != dependency_graph_.end() ||
        diagram_input_set_.find(id) != diagram_input_set_.end()) {
      throw std::logic_error("Input port is already wired.");
    }
  }

  void ThrowIfSystemNotRegistered(const System<T>* system) const {
    DRAKE_THROW_UNLESS(systems_.find(system) != systems_.end());
  }

  // Runs Kahn's algorithm to compute the topological sort order of the
  // Systems in the graph. If EvalOutput is called on each System in
  // the order that is returned, each System's inputs will be valid by
  // the time its EvalOutput is called.
  //
  // TODO(david-german-tri): Update this sort to operate on individual
  // output ports once #2890 is resolved.
  //
  // TODO(david-german-tri, bradking): Consider using functional form to
  // produce a separate execution order for each output of the Diagram.
  std::vector<const System<T>*> SortSystems() const {
    std::vector<const System<T>*> sorted_systems;

    // Build two maps:
    // A map from each system, to every system that depends on it.
    std::map<const System<T>*, std::set<const System<T>*>> dependents;
    // A map from each system, to every system on which it depends.
    std::map<const System<T>*, std::set<const System<T>*>> dependencies;

    for (const auto& connection : dependency_graph_) {
      const System<T>* src = connection.second.first;
      const System<T>* dest = connection.first.first;
      // TODO(david-german-tri): Make direct-feedthrough resolution more
      // fine-grained once #2890 is resolved. Until then, avoiding false
      // positives in algebraic loop detection here would simply lead to
      // spurious infinite loops at diagram execution time.
      //
      // To understand this, consider the following diagram:
      //
      //  input A0--[A]-----output A0--input B0-X---[B]--output B0-|
      //     ^          |-X-output A1--input B1---|                |
      //     |-----------------------------------------------------|
      //
      // Suppose there is direct feedthrough from input A0 to output A0, but not
      // output A1.  Similarly, suppose there is direct feedthrough from input
      // B1 to output B0, but not from input B0 to output B0. (The X character
      // above indicates absence of direct-feedthrough.) There is no algebraic
      // loop in this diagram, and we could detect that there is no algebraic
      // loop at diagram build time. To detect it, we would construct a graph
      // where the vertices are input and output ports, and the edges are
      // direct-feedthrough connections (input to output) and Diagram edges
      // (output to input). Observing that this graph contains no cycles is
      // equivalent to observing that there is no algebraic loop.
      //
      // However, this observation does us no good without per-port evaluation
      // from #2890, because #3455 made the execution order implicit instead of
      // explicit. When we need to compute input A0, we will pull on output B0,
      // which will pull on both outputs of A. Pulling on output A0 in
      // particular will spuriously pull on input A0, yielding an infinite loop.
      //
      // Consequently, this function remains for now a topological sort of
      // Systems, not of ports, and we must restrict ourselves to considering
      // an entire System direct-feedthrough if it has direct feedthrough from
      // any input to any output.
      if (dest->HasAnyDirectFeedthrough()) {
        dependents[src].insert(dest);
        dependencies[dest].insert(src);
      }
    }

    // Find the systems that have no direct-feedthrough inputs.
    std::vector<const System<T>*> nodes_with_in_degree_zero;
    for (const auto& system : registered_systems_) {
      if (dependencies.find(system.get()) == dependencies.end()) {
        nodes_with_in_degree_zero.push_back(system.get());
      }
    }

    while (!nodes_with_in_degree_zero.empty()) {
      // Pop a node with in-degree zero.
      const System<T>* node = nodes_with_in_degree_zero.back();
      nodes_with_in_degree_zero.pop_back();

      // Push the node onto the sorted output.
      sorted_systems.push_back(node);

      for (const System<T>* dependent : dependents[node]) {
        dependencies[dependent].erase(node);
        if (dependencies[dependent].empty()) {
          nodes_with_in_degree_zero.push_back(dependent);
        }
      }
    }

    if (sorted_systems.size() != systems_.size()) {
      throw std::logic_error("Algebraic loop detected in DiagramBuilder.");
    }
    return sorted_systems;
  }

  /// Produces the Blueprint that has been described by the calls to
  /// Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  /// graph is not buildable.
  typename Diagram<T>::Blueprint Compile() const {
    if (registered_systems_.size() == 0) {
      throw std::logic_error("Cannot Compile an empty DiagramBuilder.");
    }
    typename Diagram<T>::Blueprint blueprint;
    blueprint.input_port_ids = input_port_ids_;
    blueprint.output_port_ids = output_port_ids_;
    blueprint.dependency_graph = dependency_graph_;
    blueprint.sorted_systems = SortSystems();
    return blueprint;
  }

  // The ordered inputs and outputs of the Diagram to be built.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  // For fast membership queries: has this input port already been declared?
  std::set<PortIdentifier> diagram_input_set_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // The unsorted set of Systems in this DiagramBuilder. Used for fast
  // membership queries.
  std::set<const System<T>*> systems_;
  // The Systems in this DiagramBuilder, in the order they were registered.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;
};

}  // namespace systems
}  // namespace drake
