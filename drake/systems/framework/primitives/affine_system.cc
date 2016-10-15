#include "drake/systems/framework/primitives/affine_system-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

// TODO(liang.fok) add `AutoDiffXd` explicit template instantiation.
template class DRAKE_EXPORT AffineSystem<double>;

}  // namespace systems
}  // namespace drake
