use std::{
    collections::VecDeque,
    ffi::CStr,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};

use anyhow::Result;
use esp_idf_svc::{
    mqtt::client::{Details, EspMqttClient, EventPayload, MqttClientConfiguration, QoS},
    tls::X509,
};
use log::*;

pub type MqttMessageQueue = Arc<Mutex<VecDeque<(String, Vec<u8>)>>>;

pub struct Mqtt {
    client: EspMqttClient<'static>,
    connected: Arc<AtomicBool>,
    message_queue: MqttMessageQueue,
    /// AWS IoT reserved topics (`$aws/...`, e.g. Jobs) are routed into this
    /// separate queue at ingestion time so a consumer polling for a job
    /// response (`wait_for_job_message`) can never pop a message meant for
    /// the device's own command channel (`try_receive`), or vice versa.
    job_message_queue: MqttMessageQueue,
    /// AWS IoT job topics are namespaced by Thing Name. In this fleet's setup
    /// the registered Thing Name equals the MQTT client_id, so this is just
    /// what was passed in as `client_id`.
    pub thing_name: String,
}

const ROOT_CA: &CStr = unsafe {
    CStr::from_bytes_with_nul_unchecked(
        concat!(include_str!("../../../../certs/AmazonRootCA1.pem"), "\0").as_bytes(),
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
                "../../../../certs/tower_",
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
                "../../../../certs/tower_",
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

            // Increased document buffer size
            buffer_size: 4096,
            ..Default::default()
        }; // New AWS config

        info!("Attempting to create MQTT client...");
        info!("Broker URL: {}", broker_url);

        let connected = Arc::new(AtomicBool::new(false));
        let connected_clone = connected.clone();
        let message_queue = Arc::new(Mutex::new(VecDeque::new()));
        let message_queue_clone = message_queue.clone();
        let job_message_queue = Arc::new(Mutex::new(VecDeque::new()));
        let job_message_queue_clone = job_message_queue.clone();

        let (client, mut connection) = EspMqttClient::new(broker_url, &mqtt_config)?;
        info!("MQTT client created successfully!");

        thread::spawn(move || {
            // Enqueue inbound messages so the main loop can drain them via
            // `try_receive()` (e.g. the remote command channel) or
            // `wait_for_job_message()` (AWS IoT Jobs)
            let enqueue = |topic: String, data: Vec<u8>| {
                let target = if topic.starts_with("$aws/") {
                    &job_message_queue_clone
                } else {
                    &message_queue_clone
                };
                if let Ok(mut queue) = target.lock() {
                    queue.push_back((topic, data));
                } else {
                    warn!("Failed to lock MQTT queue for received message");
                }
            };

            // A payload larger than the client's inbound buffer is delivered as
            // several `Received` events: the first carries the topic plus
            // `Details::InitialChunk`, every later one carries `topic: None`
            // and `Details::SubsequentChunk`
            let mut partial: Option<(String, Vec<u8>, usize)> = None;

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
                    EventPayload::Received {
                        topic,
                        data,
                        details,
                        ..
                    } => match details {
                        Details::Complete => {
                            if let Some(topic) = topic {
                                enqueue(topic.to_string(), data.to_vec());
                            } else {
                                warn!("MQTT received message without a topic");
                            }
                        }
                        Details::InitialChunk(chunk) => {
                            let Some(topic) = topic else {
                                warn!("MQTT received first chunk without a topic");
                                continue;
                            };
                            if partial.is_some() {
                                warn!("Discarding incomplete MQTT message, a new one started");
                            }
                            let mut buf = Vec::with_capacity(chunk.total_data_size);
                            buf.extend_from_slice(data);
                            if buf.len() >= chunk.total_data_size {
                                enqueue(topic.to_string(), buf);
                            } else {
                                info!(
                                    "Reassembling {}-byte MQTT message on {} ({} bytes so far)",
                                    chunk.total_data_size,
                                    topic,
                                    buf.len()
                                );
                                partial = Some((topic.to_string(), buf, chunk.total_data_size));
                            }
                        }
                        Details::SubsequentChunk(chunk) => {
                            let complete = match partial.as_mut() {
                                Some((_, buf, total)) => {
                                    buf.extend_from_slice(data);
                                    buf.len() >= *total
                                }
                                None => {
                                    warn!(
                                        "Discarding MQTT chunk at offset {} with no message in progress",
                                        chunk.current_data_offset
                                    );
                                    false
                                }
                            };
                            if complete {
                                if let Some((topic, buf, _)) = partial.take() {
                                    info!(
                                        "Reassembled {}-byte MQTT message on {}",
                                        buf.len(),
                                        topic
                                    );
                                    enqueue(topic, buf);
                                }
                            }
                        }
                    },
                    EventPayload::Error(e) => error!("MQTT error: {:?}", e),
                    _ => {}
                }
            }
        });

        Ok(Self {
            client,
            connected,
            message_queue,
            job_message_queue,
            thing_name: client_id.to_string(),
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
                return Err(anyhow::anyhow!(
                    "MQTT connection timeout after {}ms",
                    timeout_ms
                ));
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
        self.client
            .publish(topic, QoS::AtLeastOnce, false, payload)?;
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
    /// Only ever sees device topics (`tower/...`) — AWS IoT reserved topics
    /// (`$aws/...`) are routed to the separate job queue at ingestion time,
    /// so this can never steal a job response out from under
    /// `wait_for_job_message`.
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

    /// Block (up to `timeout`) waiting for the next incoming AWS IoT Jobs
    /// message (`$aws/...` topics only — see the job queue routing in
    /// `new_mqtt`). Unlike `try_receive`, this can never consume a message
    /// meant for the device command channel, since the two queues are fed
    /// independently based on topic prefix as messages arrive.
    pub fn wait_for_job_message(&self, timeout: Duration) -> Option<(String, Vec<u8>)> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if let Ok(mut queue) = self.job_message_queue.lock() {
                if let Some(msg) = queue.pop_front() {
                    return Some(msg);
                }
            }
            thread::sleep(Duration::from_millis(50));
        }
        None
    }
}
