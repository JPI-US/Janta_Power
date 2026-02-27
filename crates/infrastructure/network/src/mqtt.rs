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

const CA_CERT: &CStr = unsafe{
    CStr::from_bytes_with_nul_unchecked(concat!(include_str!("../fullchain.pem"), "\0").as_bytes())
};
impl Mqtt {
    /// Create a new TLS-secured MQTT client
    pub fn new_mqtt(broker_url: &str, client_id: &str, user: &str, pass: &str) -> Result<Self> {


        let mqtt_config = MqttClientConfiguration {
            client_id: Some(client_id),
            username: Some(user),
            password: Some(pass),
            server_certificate: Some(X509::pem(CA_CERT)),
            keep_alive_interval: Some(Duration::from_secs(60)),
            ..Default::default()
        };

        info!("Attempting to create MQTT client...");
        info!("Broker URL: {}", broker_url);

        let connected = Arc::new(AtomicBool::new(false));
        let connected_clone = connected.clone();
        let message_queue = Arc::new(Mutex::new(VecDeque::new()));
        let message_queue_clone = message_queue.clone();

        let (client, mut connection) = EspMqttClient::new(
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
                    }
                    EventPayload::Disconnected => {
                        warn!("MQTT Disconnected");
                        connected_clone.store(false, Ordering::SeqCst);
                    }
                    EventPayload::Published(id) => info!("MQTT Publish Message {} confirmed", id),
                    EventPayload::Received { topic, data, .. } => {
                        if let Some(topic_str) = topic {
                            info!("MQTT Received: topic={}, payload_len={}", topic_str, data.len());
                            if let Ok(mut queue) = message_queue_clone.lock() {
                                queue.push_back((topic_str.to_string(), data.to_vec()));
                            }
                        } else {
                            warn!("MQTT Received message with no topic");
                        }
                    }
                    EventPayload::Error(e) => error!("MQTT error: {:?}", e),
                    _ => {}
                }
            }
        });

        Ok(Self {client, connected, message_queue})
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
