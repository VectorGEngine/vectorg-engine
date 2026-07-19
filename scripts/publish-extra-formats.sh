#!/bin/bash

currdir=$(pwd)

### Publish vectorg-engine-meshloader.
cd "crates/vectorg-engine-meshloader" && cargo publish $DRY_RUN || exit 1
cd "$currdir" || exit 2

### Publish vectorg-engine-urdf.
cd "crates/vectorg-engine-urdf" && cargo publish $DRY_RUN || exit 1
cd "$currdir" || exit 2