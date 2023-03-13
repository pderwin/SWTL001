# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.1.0] - 2022-04-08

### Added

- [app] Support of LR1121 transceiver

## [2.0.0] - 2022-04-08

### Added

- [app] Revamped the way core implementation and helper functions are interleaved for better clarity
- [system] `system_spi_read_with_dummy_byte()` function

### Changed

- [driver] Updated LoRa Basics Modem-E driver to v3.0.1
- [driver] Transitionned from LR1110 driver to LR11xx driver v2.1.1
- [driver] LR11xx HAL implementations
- [debug] Improved debug messages on both UART and display interfaces

### Removed

- [system] `system_spi_read()` and `system_spi_write_read()` functions

## [1.2.0] - 2020-06-02

### Changed

- [driver] Updated LoRa Basics Modem-E driver to v2.0.1
- [debug] Improved debug messages on both UART and display interfaces

## [1.1.0] - 2020-10-13

### Added

- Initial version (with LR1110 driver v3.0.0 and LoRa Basics Modem-E driver v1.0.0)
