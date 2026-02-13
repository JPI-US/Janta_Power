// Runtime configuration manager for compile-time .env + runtime overrides.
//
// Priority: runtime_override > .env_value > hardcoded_default
// Runtime overrides can be persisted to NVS for persistence across reboots.

use esp_idf_svc::nvs::{EspNvs, NvsPartitionId};
use log::{error, info, warn};
use serde_json::Value;
use std::collections::HashMap;

const NVS_NAMESPACE: &str = "runtime_config";
const NVS_KEY_OVERRIDES: &str = "overrides";
const NVS_KEY_RUNTIME_MODE: &str = "runtime_mode";

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RuntimeMode {
    Normal,
    Admin,
}

impl RuntimeMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            RuntimeMode::Normal => "normal",
            RuntimeMode::Admin => "admin",
        }
    }

    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "normal" => Some(RuntimeMode::Normal),
            "admin" => Some(RuntimeMode::Admin),
            _ => None,
        }
    }
}

pub struct ConfigManager {
    // Runtime overrides (in-memory cache)
    overrides: HashMap<String, Value>,
    // Whether to persist overrides to NVS
    persist_nvs: bool,
}

impl ConfigManager {
    /// Create a new ConfigManager and load persisted overrides from NVS.
    pub fn new<T: NvsPartitionId>(nvs: &mut EspNvs<T>, persist_nvs: bool) -> Self {
        let mut manager = Self {
            overrides: HashMap::new(),
            persist_nvs,
        };
        manager.load_from_nvs(nvs);
        manager
    }

    /// Get a config value with priority: runtime_override > .env > default
    /// Returns the value as a JSON Value for flexibility.
    pub fn get(&self, key: &str) -> Option<Value> {
        // First check runtime overrides
        if let Some(val) = self.overrides.get(key) {
            return Some(val.clone());
        }

        // Then check compile-time .env constants (via generated constants)
        // Note: This requires accessing the constants module, which we'll handle via a helper
        None // Will be extended to check .env constants
    }

    /// Get a config value as a specific type, with fallback to default.
    pub fn get_with_default<T: ConfigValue>(&self, key: &str, default: T) -> T {
        if let Some(val) = self.get(key) {
            if let Ok(parsed) = T::from_value(&val) {
                return parsed;
            }
        }
        default
    }

    /// Set a runtime override. Optionally persists to NVS.
    pub fn set_runtime<T: NvsPartitionId>(
        &mut self,
        key: &str,
        value: Value,
        nvs: &mut EspNvs<T>,
    ) -> anyhow::Result<()> {
        info!("Setting runtime config override: {} = {:?}", key, value);
        self.overrides.insert(key.to_string(), value.clone());

        if self.persist_nvs {
            self.save_to_nvs(nvs)?;
        }

        Ok(())
    }

    /// Remove a runtime override (revert to .env/default).
    pub fn remove_runtime<T: NvsPartitionId>(
        &mut self,
        key: &str,
        nvs: &mut EspNvs<T>,
    ) -> anyhow::Result<()> {
        if self.overrides.remove(key).is_some() {
            info!("Removed runtime config override: {}", key);
            if self.persist_nvs {
                self.save_to_nvs(nvs)?;
            }
        }
        Ok(())
    }

    /// Clear all runtime overrides (reset to .env/defaults).
    pub fn reset_to_defaults<T: NvsPartitionId>(
        &mut self,
        nvs: &mut EspNvs<T>,
    ) -> anyhow::Result<()> {
        info!("Resetting all runtime config overrides");
        self.overrides.clear();

        if self.persist_nvs {
            self.save_to_nvs(nvs)?;
        }

        Ok(())
    }

    /// Get current runtime mode (admin/normal).
    pub fn get_runtime_mode<T: NvsPartitionId>(&self, nvs: &mut EspNvs<T>) -> RuntimeMode {
        if !self.persist_nvs {
            return RuntimeMode::Normal; // Default if persistence disabled
        }

        let mut buffer = [0u8; 16];
        if let Ok(Some(mode_str)) = nvs.get_str(NVS_KEY_RUNTIME_MODE, &mut buffer) {
            RuntimeMode::from_str(&mode_str).unwrap_or(RuntimeMode::Normal)
        } else {
            RuntimeMode::Normal
        }
    }

    /// Set runtime mode (admin/normal) and persist to NVS.
    pub fn set_runtime_mode<T: NvsPartitionId>(
        &mut self,
        mode: RuntimeMode,
        nvs: &mut EspNvs<T>,
    ) -> anyhow::Result<()> {
        info!("Setting runtime mode to: {:?}", mode);
        
        if self.persist_nvs {
            nvs.set_str(NVS_KEY_RUNTIME_MODE, mode.as_str())?;
        }

        Ok(())
    }

    /// Load overrides from NVS.
    fn load_from_nvs<T: NvsPartitionId>(&mut self, nvs: &mut EspNvs<T>) {
        if !self.persist_nvs {
            return;
        }

        let mut buffer = [0u8; 1024]; // Adjust size as needed
        if let Ok(Some(json_str)) = nvs.get_str(NVS_KEY_OVERRIDES, &mut buffer) {
            match serde_json::from_str::<HashMap<String, Value>>(&json_str) {
                Ok(overrides) => {
                    self.overrides = overrides;
                    info!("Loaded {} runtime config overrides from NVS", self.overrides.len());
                }
                Err(e) => {
                    warn!("Failed to parse runtime config overrides from NVS: {:?}", e);
                }
            }
        }
    }

    /// Save overrides to NVS.
    fn save_to_nvs<T: NvsPartitionId>(&mut self, nvs: &mut EspNvs<T>) -> anyhow::Result<()> {
        let json_str = serde_json::to_string(&self.overrides)?;
        nvs.set_str(NVS_KEY_OVERRIDES, &json_str)?;
        Ok(())
    }

    /// Get all current overrides (for debugging/get_config "all").
    pub fn get_all_overrides(&self) -> &HashMap<String, Value> {
        &self.overrides
    }
}

/// Trait for types that can be converted from JSON Value.
pub trait ConfigValue: Sized {
    fn from_value(v: &Value) -> Result<Self, String>;
}

impl ConfigValue for f64 {
    fn from_value(v: &Value) -> Result<Self, String> {
        match v {
            Value::Number(n) => n.as_f64().ok_or_else(|| "Not a valid f64".to_string()),
            _ => Err("Not a number".to_string()),
        }
    }
}

impl ConfigValue for f32 {
    fn from_value(v: &Value) -> Result<Self, String> {
        match v {
            Value::Number(n) => n.as_f64().map(|f| f as f32).ok_or_else(|| "Not a valid f32".to_string()),
            _ => Err("Not a number".to_string()),
        }
    }
}

impl ConfigValue for i64 {
    fn from_value(v: &Value) -> Result<Self, String> {
        match v {
            Value::Number(n) => n.as_i64().ok_or_else(|| "Not a valid i64".to_string()),
            _ => Err("Not a number".to_string()),
        }
    }
}

impl ConfigValue for bool {
    fn from_value(v: &Value) -> Result<Self, String> {
        match v {
            Value::Bool(b) => Ok(*b),
            _ => Err("Not a boolean".to_string()),
        }
    }
}

impl ConfigValue for String {
    fn from_value(v: &Value) -> Result<Self, String> {
        match v {
            Value::String(s) => Ok(s.clone()),
            _ => Err("Not a string".to_string()),
        }
    }
}
