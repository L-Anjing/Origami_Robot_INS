#pragma once
#include <string>
namespace origami
{
// Export schema 2 MAT v5 using only C++ (no Python or MATLAB runtime).
void export_mat(const std::string& folder);
} // namespace origami
