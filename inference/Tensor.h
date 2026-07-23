#ifndef DEPLOY_REAL_INFERENCE_TENSOR_H
#define DEPLOY_REAL_INFERENCE_TENSOR_H

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace deploy_real
{

struct Tensor
{
    std::vector<std::int64_t> shape;
    std::vector<float> values;

    Tensor() = default;
    Tensor(std::vector<std::int64_t> tensor_shape, std::vector<float> tensor_values)
        : shape(std::move(tensor_shape)), values(std::move(tensor_values))
    {
    }
};

using TensorMap = std::unordered_map<std::string, Tensor>;

} // namespace deploy_real

#endif
