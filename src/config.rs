use std::{fs, path::Path};

use serde::{Deserialize, Serialize};
use toml;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub device: DeviceConfig,
    pub wifi: WifiConfig,
    pub location: LocationConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceConfig {
    pub tower_id: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WifiConfig {
    pub ssid: String,
    pub password: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocationConfig {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub timezone_offset_hours: i32,
}

/* impl Config {
    pub fn load() -> anyhow::Result<Self> {
        // Embedded configuration (compiled into binary)
        let config_content = include_str!("../config.toml");
        let config: Config = toml::from_str(config_content)?;
        log::info!("Loaded embedded configuration");
        Ok(config)
    }

    pub fn save(&self) -> anyhow::Result<()> {
        let config_content = toml::to_string_pretty(self)?;
        fs::write("config.toml", config_content)?;
        log::info!("Configuration saved to config.toml");
        Ok(())
    }
} */

impl Config {
    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    pub fn load() -> anyhow::Result<Self> {
        // Try external file first
        if Path::new("config.toml").exists() {
            let config_content = fs::read_to_string("config.toml")?;
            let config: Config = toml::from_str(&config_content)?;
            log::info!("Loaded configuration from file");
            Ok(config)
        } else {
            // Fallback to embedded defaults
            let config_content = include_str!("../config.toml.example");
            let config: Config = toml::from_str(config_content)?;
            log::warn!("Using embedded default configuration");
            Ok(config)
        }
    }
}

// Helper functions for easy access
impl Config {
    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    pub fn get_wifi_ssid(&self) -> &str {
        &self.wifi.ssid
    }

    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    pub fn get_wifi_password(&self) -> &str {
        &self.wifi.password
    }

    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    pub fn get_latitude(&self) -> f64 {
        self.location.latitude
    }

    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    pub fn get_longitude(&self) -> f64 {
        self.location.longitude
    }

    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    pub fn get_altitude(&self) -> f64 {
        self.location.altitude
    }

    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    pub fn get_tower_id(&self) -> u32 {
        self.device.tower_id
    }

    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    pub fn get_timezone_offset(&self) -> i32 {
        self.location.timezone_offset_hours
    }
}
