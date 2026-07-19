# VectorG Engine URDF loader

`vectorg-engine-urdf` converts URDF files into VectorG Engine rigid bodies, colliders, and joints.

## Optional Cargo features

- `stl`: load referenced `.stl` meshes;
- `collada`: load referenced `.dae` meshes; and
- `wavefront`: load referenced `.obj` meshes.

## Limitations

- Supported mesh formats are STL, Collada, and Wavefront OBJ.
- Multibody joints are reset to their neutral position when inserted.
- `Joint::dynamics`, `Joint::limit.effort`, `Joint::limit.velocity`, `Joint::mimic`, and `Joint::safety_controller` are currently ignored.

VectorG Engine is derived from Rapier by Dimforge and preserves the original license and attribution.
