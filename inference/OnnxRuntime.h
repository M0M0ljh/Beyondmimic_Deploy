#ifndef DEPLOY_REAL_INFERENCE_ONNX_RUNTIME_H
#define DEPLOY_REAL_INFERENCE_ONNX_RUNTIME_H

#include "inference/Tensor.h"

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>
#include <vector>

namespace deploy_real
{

class OnnxRuntime
{
public:
    explicit OnnxRuntime(const std::string &model_path);

    TensorMap Run(TensorMap inputs);

    const std::vector<std::string> &InputNames() const;
    const std::vector<std::string> &OutputNames() const;
    const std::vector<std::int64_t> &InputShape(std::size_t index) const;
    const std::vector<std::int64_t> &OutputShape(std::size_t index) const;

private:
    void ValidateInput(std::size_t index, const Tensor &tensor) const;

    Ort::Env environment_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<const char *> input_name_ptrs_;
    std::vector<const char *> output_name_ptrs_;
    std::vector<std::vector<std::int64_t>> input_shapes_;
    std::vector<std::vector<std::int64_t>> output_shapes_;
};

} // namespace deploy_real

#endif
