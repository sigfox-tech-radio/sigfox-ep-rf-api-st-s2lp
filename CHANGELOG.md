# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [v4.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v4.0) - 22 Nov 2024

### Added

* Add **timeout handling** in S2LP state switching function.

### Fixed

* Perform chip configuration in `READY` state instead of `STANDBY` to **relax SPI timings constraint** during registers programming.
* Remove **unifdef dependency** in all cmake with linked target.

### Changed

* Upgrade to **sigfox-ep-lib v4.0**.
* Add **radio configuration pointer** in HW API open function to manage multi-RC front-ends.
* Adjust **radio latencies** to add more margin regarding timings specification.

### Known limitations

* **LBT** not implemented.
* **Modulated CW** not supported for type approval addon.

## [v3.1](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v3.1) - 30 May 2024

### Added

* Add `weak` attribute to functions templates.
* Add **SFX_UNUSED** macro to remove extra warnings.

### Known limitations

* **LBT** not implemented.
* **Modulated CW** not supported for type approval addon.

## [v3.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v3.0) - 03 May 2024

### Added

* Add HW API functions to **enter and exit shutdown mode** with SDN pin.

### Fixed

* Reset **hardware latencies** to 0 before calling HW API function.

### Changed

* Update **S2LP library** from newer `x-cube-subg2` GitHub repository.
* Use `S2LP_CORE_SPI.h` header instead of `MCU_Interface_template.h` to define SPI access functions.

### Removed

* Remove `s2lp.patch` file since ST driver is now called as it is.

### Known limitations

* **LBT** not implemented.
* **Modulated CW** not supported for type approval addon.

## [v2.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v2.0) - 22 Mar 2024

### Added

* Add **dynamic amplitude tables** computation to **support all RF output powers**.

### Fixed

* Add cast on custom RF API **error codes**.
* Fix **compilation warning** in send function when `ERROR_CODES` flag is not defined.

### Changed

* Improve **HW API interface** to support external radio front-end.
* Remove `inline` keyword on RF API **functions redirection**.
* Rename `RCx` compilation flags to `RCx_ZONE` for **Microchip MCUs compatibility**.

### Known limitations

* **LBT** not implemented.
* **Modulated CW** not supported for type approval addon.

## [v1.3](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v1.3) - 09 Nov 2023

### Added

* Implement `RF_API_start_continuous_wave()` function for **type approval addon**.
* Add **RF frequency parameter check** in `RF_API_init()` function since it is not performed by ST driver.

### Fixed

* Fix **TX output power formula** in ST driver (patch file update).
* Do not close radio driver in `RF_API_error()` function.
* Add **missing pointer symbol** on HW API callback syntax.

### Removed

* Remove `doc` folder since images are now hosted on the GitHub wiki.

### Known limitations

* **LBT** not implemented.
* **External radio front-end** not supported.
* **Modulated CW** not supported for type approval addon.

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
