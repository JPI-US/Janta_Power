use anyhow::Result;
use log::*;
use esp_idf_svc::{
    mqtt::client::{
    EspMqttClient, EventPayload, MqttClientConfiguration, QoS},
    tls::X509,
};
use std::{sync::{atomic::{AtomicBool, Ordering}, Arc, Mutex}, thread};
use std::ffi::CStr;
use std::time::Duration;
use std::collections::VecDeque;
pub struct Mqtt {
    client: EspMqttClient<'static>,
    connected: Arc<AtomicBool>,
    message_queue: Arc<Mutex<VecDeque<(String, Vec<u8>)>>>,
}

const ROOT_CA: &CStr = unsafe {
    CStr::from_bytes_with_nul_unchecked(
        concat!(include_str!("../AmazonRootCA1.pem"), "\0").as_bytes(),
    )
};

// Cert/key filenames are derived from `DEVICE_ID` in `.env` via this crate's
// `build.rs` (which re-exports it as a rustc-env var). Provisioning workflow:
// drop `tower_{DEVICE_ID}-certificate.pem.crt` + `tower_{DEVICE_ID}-private.pem.key`
// into this directory and set `DEVICE_ID` in `.env`; no source edits per tower.
const DEVICE_CERT: &CStr = unsafe {
    CStr::from_bytes_with_nul_unchecked(
        concat!(
            include_str!(concat!(
                "../tower_",
                env!("DEVICE_ID"),
                "-certificate.pem.crt"
            )),
            "\0"
        )
        .as_bytes(),
    )
};

const PRIVATE_KEY: &CStr = unsafe {
    CStr::from_bytes_with_nul_unchecked(
        concat!(
            include_str!(concat!(
                "../tower_",
                env!("DEVICE_ID"),
                "-private.pem.key"
            )),
            "\0"
        )
        .as_bytes(),
    )
};
impl Mqtt {
    /// Create a new TLS-secured MQTT client
    pub fn new_mqtt(broker_url: &str, client_id: &str) -> Result<Self> {
        let mqtt_config = MqttClientConfiguration {
            client_id: Some(client_id),

            // AWS IoT Core requirements
            server_certificate: Some(X509::pem(ROOT_CA)),
            client_certificate: Some(X509::pem(DEVICE_CERT)),
            private_key: Some(X509::pem(PRIVATE_KEY)),

            keep_alive_interval: Some(Duration::from_secs(60)),
            use_global_ca_store: false,
            ..Default::default()
        }; // New AWS config

        info!("Attempting to create MQTT client...");
        info!("Broker URL: {}", broker_url);

        let connected = Arc::new(AtomicBool::new(false));
        let connected_clone = connected.clone();
        let message_queue = Arc::new(Mutex::new(VecDeque::new()));

        let (mut client, mut connection) = EspMqttClient::new(
            broker_url,
            &mqtt_config,
        )?;
        info!("MQTT client created successfully!");

        thread::spawn(move || {
            while let Ok(event) = connection.next() {
                match event.payload() {
                    EventPayload::Connected(_) => {
                        info!("MQTT Connected");
                        connected_clone.store(true, Ordering::SeqCst);

                        // publish inside thread if needed
                    }
                    EventPayload::Disconnected => {
                        warn!("MQTT Disconnected, will queue messages temporarily...");
                        warn!("Retrying momentarilly...");
                        connected_clone.store(false, Ordering::SeqCst);
                        // trigger reconnect
                    }
                    EventPayload::Published(id) => info!("MQTT Publish Message {} confirmed", id),
                    EventPayload::Error(e) => error!("MQTT error: {:?}", e),
                    _ => {}
                }
            }
        });

        Ok(Self {
            client,
            connected,
            message_queue,
        })
    }

    // Expose the flag safely
    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    /// Wait for MQTT connection to be established (with timeout)
    pub fn wait_for_connection(&self, timeout_ms: u64) -> Result<()> {
        let start = std::time::Instant::now();
        while !self.connected.load(Ordering::SeqCst) {
            if start.elapsed().as_millis() > timeout_ms as u128 {
                return Err(anyhow::anyhow!("MQTT connection timeout after {}ms", timeout_ms));
            }
            thread::sleep(Duration::from_millis(100));
        }
        Ok(())
    }

    pub fn publish(&mut self, topic: &str, payload: &[u8]) -> Result<()> {
        if !self.is_connected() {
            return Err(anyhow::anyhow!("MQTT client not connected"));
        }
        info!("Attempting to publish message to topic...");
        self.client.publish(topic, QoS::AtLeastOnce, false, payload)?;
        info!("Initial message published successfully!");
        Ok(())
    }

    pub fn subscribe(&mut self, topic: &str) -> Result<()> {
        // Wait for connection before subscribing
        self.wait_for_connection(10000)?; // 10 second timeout
        info!("Subscribing to topic: {}", topic);
        self.client.subscribe(topic, QoS::AtMostOnce)?;
        info!("Successfully subscribed to: {}", topic);
        Ok(())
    }

    /// Poll for received messages. Returns the next message if available.
    pub fn try_receive(&self) -> Option<(String, Vec<u8>)> {
        if let Ok(mut queue) = self.message_queue.lock() {
            let queue_len = queue.len();
            if queue_len > 0 {
                info!("MQTT queue: {} messages pending, popping one", queue_len);
            }
            queue.pop_front()
        } else {
            warn!("Failed to lock MQTT message queue");
            None
        }
    }
}
