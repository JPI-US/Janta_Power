# Setup
These are generic setup instructions. Specific steps will vary between systems.

## Prerequisites:
### General:
* [Git](https://git-scm.com/install/).
* [Rust](https://rustup.rs/).
  * [Rust toolchain for Xtensa Devices](https://docs.espressif.com/projects/rust/book/getting-started/toolchain.html#xtensa-devices).
* ldproxy (`cargo install ldproxy`).
* espflash (`cargo install espflash`).    
* [Python](https://www.python.org/downloads/).

 ### Linux:
 * [GitHub CLI](https://github.com/cli/cli#installation).
 * You may need to install a C linker (through something like `build-essentials` on Debian or `base-devel` on Arch).
 * [Espup environment variables](https://github.com/esp-rs/espup/?tab=readme-ov-file#environment-variables-setup).
 * Python venv is often a seperate package. On Debian, for example, you must install the `python3` and `python3-venv` packages.

## Steps:
**On Linux**, you must first authenticate using GitHub CLI.
```sh
gh auth login
```

Clone the repository.
```sh
git clone https://github.com/JPI-US/Janta_Power
```

Copy `.env.example` to a new file called `.env`. Configure important settings like `TOWER_LATITUDE`, `TOWER_LONGITUDE`, `WIFI_SSID`, `WIFI_PASSWORD`, and `DEVICE_ID`.

Place `AmazonRootCA1.pem`, `[DEVICE_ID]-certificate.pem.crt`, and `[DEVICE_ID]-private.pem.key` into `crates/infrastructure/network`.

In the repository, compile and upload to the ESP-32.
```sh
cargo run
```

# Helpful Commands
## Compile + flash
```sh
cargo run
```

## Monitor without re-flashing
```sh
espflash monitor
```

## Erase flash, reset NVS
```sh
espflash clean-flash
```

# Branch Structure
* **Master** – Production branch. Code deployed to customer towers that has been fully tested and proven stable.
* **Deployment** – Pre-production branch. Code currently being deployed to customer towers. Once it has been validated as stable, merge it into **Master**.
* **Testing** – Active testing branch. Code undergoing validation in a controlled environment on our towers. After the testing cycle is complete, merge it into **Deployment**.
* **Feature Branches** – Branches for new features and changes under development, based on **Testing**. After review and approval, merge them back into **Testing**.
