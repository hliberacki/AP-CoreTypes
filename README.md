# Core Types Functional Cluster 

[![Build Status](https://travis-ci.org/UmlautSoftwareDevelopmentAccount/AP-CoreTypes.svg?branch=master)](https://travis-ci.org/UmlautSoftwareDevelopmentAccount/AP-CoreTypes) [![codecov](https://codecov.io/gh/UmlautSoftwareDevelopmentAccount/AP-CoreTypes/branch/master/graph/badge.svg)](https://codecov.io/gh/UmlautSoftwareDevelopmentAccount/AP-CoreTypes) [![Gitter](https://badges.gitter.im/AUTOSAR-Adaptive/CoreTypes.svg)](https://gitter.im/AUTOSAR-Adaptive/CoreTypes?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge)

Core Types defines common classes and functionality used by multiple Functional Clusters as part of their public interfaces.

## Building project

```sh
mkdir build && cd build
meson setup
ninja
```

### Running tests and collecting test coverage

```sh
Run tests and coverage:
ninja test
ninja coverage-html
```