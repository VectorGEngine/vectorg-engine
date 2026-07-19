# VectorG Engine

[![VectorG Engine CI](https://github.com/VectorGEngine/vectorg-engine/actions/workflows/vectorg-engine-ci-build.yml/badge.svg)](https://github.com/VectorGEngine/vectorg-engine/actions/workflows/vectorg-engine-ci-build.yml)
[![Crates.io](https://img.shields.io/crates/v/vectorg-engine.svg)](https://crates.io/crates/vectorg-engine)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

**Real-time vehicle dynamics engine for realistic driving simulation.**

VectorG Engine is a real-time vehicle dynamics engine powering the VectorG driving simulator.

The project originated from [Rapier](https://github.com/dimforge/rapier) and now contains substantial vehicle-simulation extensions, including:

- tire modeling;
- suspension systems;
- drivetrain simulation;
- differential modeling;
- a realistic vehicle controller; and
- racing-simulation features.

VectorG Engine is no longer API compatible with upstream Rapier.

## Packages

The primary 3D package is `vectorg-engine`. Additional workspace packages provide 2D, `f64`, testbed, URDF, and mesh-loading variants.

## Getting started

Build and run the 3D example browser:

```shell
cargo run --release --bin all_examples3
```

The examples are in `examples2d/` and `examples3d/`.

## Origin and attribution

VectorG Engine is derived from Rapier, originally developed by [Dimforge](https://dimforge.com), and continues to include the original license, copyright notices, and attribution.

See [LICENSE](LICENSE) and [NOTICE](NOTICE) for licensing and attribution details.

## Contributing

Read the [Code of Conduct](CODE_OF_CONDUCT.md) and [Contribution Guidelines](CONTRIBUTING.md) before contributing.
