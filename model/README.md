# Policy model provenance

`policy96.onnx`, `redred.onnx`, and any motion assets are ignored by default.
Before adding them to a public repository, document for each file:

- source repository or training run;
- model and training-data license, including redistribution permission;
- SHA-256 checksum and ONNX Runtime version used for validation;
- input/output tensor names and shapes.

If a model cannot be redistributed, publish this file and a download script or
release asset reference instead of committing the binary.  The current code
expects the following paths when assets are supplied locally:

```text
model/g1/policy96.onnx
model/g1/redred.onnx
```
