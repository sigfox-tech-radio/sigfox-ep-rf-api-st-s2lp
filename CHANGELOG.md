# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [v1.2](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v1.2) - 10 Aug 2023

### Changed

* Improve **spectrum template** with new amplitude profile tables.
* Improve **radio configuration** steps.
* Simplify **FIFO threshold** management and **uplink latency** computation.
* Improve **error codes** definition.
* Rename **internal process** function.
* Use **Sigfox types** in driver.

### Fixed

* Fix **downlink reception issue** in blocking mode (flag set missing).

### Known limitations

* **LBT** not implemented.
* **External radio front-end** not supported.

## [v1.1](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v1.1) - 28 Jun 2023

### Added

* Add signal parameter to HW API GPIO get function for future use.

### Fixed

* Replace RF API redirecting macros by inline fonctions to solve linking issues.

### Known limitations

* **LBT** not implemented.
* **External radio front-end** not supported.

## [v1.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v1.0) - 19 Jun 2023

### General

* First version of the S2LP RF API implementation example.

### Known limitations

* **LBT** not implemented.
* **External radio front-end** not supported.
