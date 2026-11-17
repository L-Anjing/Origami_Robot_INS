#pragma once
#include "common_lib.h"
namespace origami
{
M3D skew(const V3D& value);
M3D right_jacobian(const V3D& value);
Quaterniond increment(const V3D& value);
} // namespace origami
