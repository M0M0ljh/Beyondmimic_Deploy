#include "inference/OnnxRuntime.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <numeric>
#include <stdexcept>

namespace deploy_real
{
namespace
{

std::size_t ElementCount(const std::vector<std::int64_t> &shape)
{
    if (std::any_of(shape.begin(), shape.end(), [](std::int64_t dim) {
            return dim <= 0;
        }))
    {
        throw std::runtime_error("runtime tensor shape must contain positive dimensions");
    }
    return std::accumulate(
        shape.begin(), shape.end(), std::size_t{1},
        [](std::size_t count, std::int64_t dim) {
            return count * static_cast<std::size_t>(dim);
        });
}

void ValidateFloatTensor(const Ort::TypeInfo &type_info, const std::string &name)
{
    const auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    if (tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
    {
        throw std::runtime_error("ONNX tensor '" + name + "' must use float32");
    }
}

} // namespace

OnnxRuntime::OnnxRuntime(const std::string &model_path)
    : environment_(ORT_LOGGING_LEVEL_WARNING, "g1_deploy")
{
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ = std::make_unique<Ort::Session>(
        environment_, model_path.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;
    for (std::size_t i = 0; i < session_->GetInputCount(); ++i)
    {
        const auto name = session_->GetInputNameAllocated(i, allocator);
        input_names_.emplace_back(name.get());
        const Ort::TypeInfo type_info = session_->GetInputTypeInfo(i);
        ValidateFloatTensor(type_info, input_names_.back());
        input_shapes_.push_back(type_info.GetTensorTypeAndShapeInfo().GetShape());
    }
    for (std::size_t i = 0; i < session_->GetOutputCount(); ++i)
    {
        const auto name = session_->GetOutputNameAllocated(i, allocator);
        output_names_.emplace_back(name.get());
        const Ort::TypeInfo type_info = session_->GetOutputTypeInfo(i);
        ValidateFloatTensor(type_info, output_names_.back());
        output_shapes_.push_back(type_info.GetTensorTypeAndShapeInfo().GetShape());
    }
    if (input_names_.empty() || output_names_.empty())
    {
        throw std::runtime_error("ONNX model must expose at least one input and output");
    }

    input_name_ptrs_.reserve(input_names_.size());
    for (const std::string &name : input_names_)
    {
        input_name_ptrs_.push_back(name.c_str());
    }
    output_name_ptrs_.reserve(output_names_.size());
    for (const std::string &name : output_names_)
    {
        output_name_ptrs_.push_back(name.c_str());
    }
}

void OnnxRuntime::ValidateInput(std::size_t index, const Tensor &tensor) const
{
    if (tensor.shape.size() != input_shapes_[index].size())
    {
        throw std::runtime_error("rank mismatch for ONNX input '" + input_names_[index] + "'");
    }
    for (std::size_t dim = 0; dim < tensor.shape.size(); ++dim)
    {
        const std::int64_t expected = input_shapes_[index][dim];
        if (expected > 0 && tensor.shape[dim] != expected)
        {
            throw std::runtime_error(
                "shape mismatch for ONNX input '" + input_names_[index] + "'");
        }
    }
    if (ElementCount(tensor.shape) != tensor.values.size())
    {
        throw std::runtime_error(
            "data size does not match shape for ONNX input '" + input_names_[index] + "'");
    }
    if (!std::all_of(tensor.values.begin(), tensor.values.end(), [](float value) {
            return std::isfinite(value);
        }))
    {
        throw std::runtime_error("ONNX input '" + input_names_[index] + "' is not finite");
    }
}

TensorMap OnnxRuntime::Run(TensorMap inputs)
{
    if (inputs.size() != input_names_.size())
    {
        throw std::runtime_error("ONNX input count does not match the model");
    }

    const Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> input_values;
    input_values.reserve(input_names_.size());
    for (std::size_t i = 0; i < input_names_.size(); ++i)
    {
        const auto input = inputs.find(input_names_[i]);
        if (input == inputs.end())
        {
            throw std::runtime_error("missing ONNX input '" + input_names_[i] + "'");
        }
        ValidateInput(i, input->second);
        input_values.emplace_back(Ort::Value::CreateTensor<float>(
            memory_info, input->second.values.data(), input->second.values.size(),
            input->second.shape.data(), input->second.shape.size()));
    }

    std::vector<Ort::Value> output_values = session_->Run(
        Ort::RunOptions{nullptr}, input_name_ptrs_.data(), input_values.data(),
        input_values.size(), output_name_ptrs_.data(), output_name_ptrs_.size());

    TensorMap outputs;
    for (std::size_t i = 0; i < output_values.size(); ++i)
    {
        const auto tensor_info = output_values[i].GetTensorTypeAndShapeInfo();
        const std::size_t count = tensor_info.GetElementCount();
        const float *data = output_values[i].GetTensorData<float>();
        if (!std::all_of(data, data + count, [](float value) {
                return std::isfinite(value);
            }))
        {
            throw std::runtime_error("ONNX output '" + output_names_[i] + "' is not finite");
        }
        outputs.emplace(
            output_names_[i],
            Tensor{tensor_info.GetShape(), std::vector<float>(data, data + count)});
    }
    return outputs;
}

const std::vector<std::string> &OnnxRuntime::InputNames() const
{
    return input_names_;
}

const std::vector<std::string> &OnnxRuntime::OutputNames() const
{
    return output_names_;
}

const std::vector<std::int64_t> &OnnxRuntime::InputShape(std::size_t index) const
{
    return input_shapes_.at(index);
}

const std::vector<std::int64_t> &OnnxRuntime::OutputShape(std::size_t index) const
{
    return output_shapes_.at(index);
}

} // namespace deploy_real
