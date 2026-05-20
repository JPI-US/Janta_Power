use toml;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub device: DeviceConfig,
    pub wifi: WifiConfig,
    pub mqtt: MqttConfig,
    pub customer: CustomerConfig,
    pub location: LocationConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceConfig {
    pub tower_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WifiConfig {
    pub ssid: String,
    pub password: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MqttConfig {
    pub broker: String,
    pub port: u16,
    pub username: String,
    pub password: String,
    pub topic: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomerConfig {
    pub full_name: String,
    pub address: String,
    pub phone: String,
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
    pub fn get_wifi_ssid(&self) -> &str {
        &self.wifi.ssid
    }

    pub fn get_wifi_password(&self) -> &str {
        &self.wifi.password
    }

    pub fn get_latitude(&self) -> f64 {
        self.location.latitude
    }

    pub fn get_longitude(&self) -> f64 {
        self.location.longitude
    }

    pub fn get_altitude(&self) -> f64 {
        self.location.altitude
    }

    pub fn get_tower_id(&self) -> &str {
        &self.device.tower_id
    }

    // MQTT getters
    pub fn get_mqtt_broker(&self) -> &str {
        &self.mqtt.broker
    }

    pub fn get_mqtt_port(&self) -> u16 {
        self.mqtt.port
    }

    pub fn get_mqtt_username(&self) -> &str {
        &self.mqtt.username
    }

    pub fn get_mqtt_password(&self) -> &str {
        &self.mqtt.password
    }

    pub fn get_mqtt_topic(&self) -> &str {
        &self.mqtt.topic
    }

    // Customer getters
    pub fn get_customer_full_name(&self) -> &str {
        &self.customer.full_name
    }

    pub fn get_customer_address(&self) -> &str {
        &self.customer.address
    }

    pub fn get_customer_phone(&self) -> &str {
        &self.customer.phone
    }

    pub fn get_timezone_offset(&self) -> i32 {
        self.location.timezone_offset_hours
    }
} 