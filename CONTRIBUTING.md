# Contributing to VectorG Engine

Contributions may include bug reports, fixes, documentation improvements, examples, tests, and vehicle-dynamics features.

For significant changes, open an issue in the [VectorG Engine repository](https://github.com/VectorGEngine/vectorg-engine/issues) before implementation.

## Development checks

Run the checks relevant to your change:

```shell
cargo fmt -- --check
cargo test
cargo run --release --bin all_examples2
cargo run --release --bin all_examples3
cargo run --release --bin all_examples2 --features parallel,simd-stable
cargo run --release --bin all_examples3 --features parallel,simd-stable
```

Changes to simulation behavior should include focused tests. Branding or packaging changes must not alter simulation logic, serialization formats, or physics algorithms.

Submit completed work through a [pull request](https://github.com/VectorGEngine/vectorg-engine/pulls).

## JavaScript and WebAssembly bindings

The bindings are maintained in the VectorG Engine JS repository. They require `wasm-pack`, Node.js, and npm. Run the generated package builds and TypeScript checks before submitting binding changes.

## Upstream attribution

VectorG Engine originated from Rapier by Dimforge. Preserve the original license, copyright notices, and attribution in all derived files and distributions.
