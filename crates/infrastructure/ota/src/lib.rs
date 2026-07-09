//use std::io::{Read, Write};
use std::{result::Result::Ok, thread, time::Duration};

use anyhow::Context;
// use ota::OtaPartition; // hypothetical struct from ota crate
use anyhow::Result;
use base64::{engine::general_purpose, Engine as _};
use embedded_svc::http::client::{Client as HttpClient, Method};
use esp_idf_svc::{
    http::client::{Configuration as HttpConfiguration, EspHttpConnection},
    io::Error,
    nvs::{EspNvs, *},
    ota::EspOta,
    sys::esp_crt_bundle_attach,
};
use log::*;
use network::mqtt::Mqtt;
use semver::Version;
use serde_json::Value; // for Basic Auth header
use sha2::{Digest, Sha256};

pub struct OtaUpdater<'a> {
    current_version: Version,
    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    mqtt_client: &'a mut Mqtt,
    device_id: &'a str,
    client: HttpClient<EspHttpConnection>,
    username: Option<String>,
    password: Option<String>,
    #[allow(dead_code)] // TODO: Remove #[allow(dead_code)]
    default_headers: Vec<(&'static str, &'static str)>,
}

impl<'a> OtaUpdater<'a> {
    pub fn new_ota(
        current_version: Version,
        mqtt_client: &'a mut Mqtt,
        device_id: &'a str,
        username: Option<&str>,
        password: Option<&str>,
    ) -> Result<Self> {
        let config = EspHttpConnection::new(&HttpConfiguration {
            buffer_size: Some(1024),
            timeout: Some(Duration::from_secs(60)),
            crt_bundle_attach: Some(esp_crt_bundle_attach),
            use_global_ca_store: true,
            ..Default::default()
        })
        .context("Failed to create OTA Updater")?;

        let client = HttpClient::wrap(config);

        Ok(Self {
            current_version,
            mqtt_client,
            device_id,
            client,
            username: username.map(|s| s.to_string()),
            password: password.map(|s| s.to_string()),
            default_headers: vec![("User-Agent", "ESP32-Rust-Client/1.0")],
        })
    }

    // Build authorization header if username/password provided
    fn build_auth_header(&self) -> Option<(String, String)> {
        if let (Some(u), Some(p)) = (&self.username, &self.password) {
            let credentials = format!("{}:{}", u, p);
            let encoded = general_purpose::STANDARD.encode(credentials.as_bytes());
            Some(("authorization".into(), format!("Basic {}", encoded)))
        } else {
            None
        }
    }

    // Creates a new http client
    /* fn create_https_client(&self) -> Result<HttpClient<EspHttpConnection>> {
        let config = EspHttpConnection::new(&HttpConfiguration {
            buffer_size: Some(1024),
            timeout: Some(Duration::from_secs(60)),
            crt_bundle_attach: Some(esp_crt_bundle_attach),
            use_global_ca_store: true,
            ..Default::default()
        })?;
        Ok(HttpClient::wrap(config))
    }  */

    // Function for requesting the version text file from the server
    fn get_remote_version(&mut self, url: &str) -> Result<Value> {
        const MAX_RETRIES: usize = 3;
        const RETRY_DELAY: Duration = Duration::from_secs(2);

        for attempt in 1..=MAX_RETRIES {
            info!("Attempt {} to fetch remote version...", attempt);

            // Recreate the HTTP client for each attempt
            let mut headers = vec![("accept", "application/json")];
            if let Some((key, value)) = self.build_auth_header() {
                // Leak strings into static refs (safe in embedded static context)
                headers.push((
                    Box::leak(key.into_boxed_str()),
                    Box::leak(value.into_boxed_str()),
                ));
            }

            // Build GET request using existing client
            let request = match self.client.request(Method::Get, url, &headers) {
                Ok(r) => r,
                Err(e) => {
                    warn!("Failed to build GET request: {:?}", e);
                    thread::sleep(RETRY_DELAY);
                    continue;
                }
            };

            // Create a new request
            match request.submit() {
                Ok(mut response) => {
                    let status = response.status();
                    info!("HTTP status: {}", status);
                    if !(200..300).contains(&status) {
                        warn!("Non-success HTTP status: {}", status);
                        thread::sleep(RETRY_DELAY);
                        continue;
                    }

                    // Read response in a loop (streaming)
                    let mut buf = [0u8; 512];
                    let bytes_read = match response.read(&mut buf) {
                        Ok(n) => n,
                        Err(e) => {
                            warn!("Failed reading body: {:?}", e);
                            thread::sleep(RETRY_DELAY);
                            continue;
                        }
                    };
                    info!("Read {} bytes", bytes_read);

                    let body_str = std::str::from_utf8(&buf[..bytes_read])
                        .map_err(|e| anyhow::anyhow!("UTF-8 decode error: {e}"))?;

                    let json: Value = serde_json::from_str(body_str)
                        .map_err(|e| anyhow::anyhow!("JSON parse error: {e}"))?;

                    return Ok(json);
                }
                Err(e) => {
                    warn!("Request failed: {:?}", e);
                    thread::sleep(RETRY_DELAY);
                }
            }
        }
        Err(anyhow::anyhow!(
            "Failed to fetch remote version after {} attempts",
            MAX_RETRIES
        ))
    }

    pub fn run_version_compare<T: NvsPartitionId>(&mut self, nvs: &mut EspNvs<T>) -> Result<()> {
        // Retrieve remote version metadata for this tower.
        let metadata_url = format!(
            "https://firmware.jantaus.com/firmware/test2/metadata{}.json",
            self.device_id
        );
        let remote_json = self.get_remote_version(&metadata_url)?;

        // Extact the "version" field from JSON and verify its not empty
        let remote_version: Version = remote_json
            .get("version")
            .and_then(|v| v.as_str())
            .ok_or_else(|| anyhow::anyhow!("Missing or invalid 'version' field in remote JSON"))?
            .trim()
            .parse()?;

        // Extact the "size" field from JSON and verify its not empty
        let remote_size = remote_json
            .get("size")
            .and_then(|s| s.as_u64())
            .ok_or_else(|| anyhow::anyhow!("Missing or invalid 'size' field in remote JSON"))?;

        if remote_size == 0 {
            return Err(anyhow::anyhow!("'size' field is zero"));
        }

        // Extract download_url
        let remote_url = remote_json
            .get("download_url")
            .and_then(|u| u.as_str())
            .ok_or_else(|| {
                anyhow::anyhow!("Missing or invalid 'download_url' field in remote JSON")
            })?
            .trim()
            .to_string();

        if remote_url.is_empty() {
            return Err(anyhow::anyhow!("'download_url' field is empty"));
        }

        // Extract sha256 and validate sha256 length (must be 64 hex chars → 32 bytes)
        let remote_sha256 = remote_json
            .get("sha256")
            .and_then(|h| h.as_str())
            .ok_or_else(|| anyhow::anyhow!("Missing or invalid 'sha256' field in remote JSON"))?
            .trim()
            .to_string();

        if remote_sha256.len() != 64 {
            return Err(anyhow::anyhow!(
                "'sha256' must be exactly 64 hex characters"
            ));
        }

        if hex::decode(&remote_sha256).is_err() {
            return Err(anyhow::anyhow!("'sha256' is not valid hex"));
        }

        // TODO: Verify digital signature if present (strongly recommended!)

        info!("Here is the current remote version: {remote_version}");
        info!(
            "Here is the current firmware version: {}",
            self.current_version
        );

        info!("Here is the current download url: {remote_url}");
        info!("Here is the current sha256: {remote_sha256}");
        info!("Here is the current download size: {remote_size}");

        if remote_version > self.current_version {
            info!("New firmware version detected!");

            // Run firmware update
            info!("Waiting 5 seconds before running firmware download...");
            thread::sleep(Duration::from_secs(5));
            let flash_download = self.run_update(
                remote_url,
                remote_version.clone(),
                remote_sha256,
                remote_size,
            );

            match flash_download {
                Ok(_) => {
                    // Stash the version we're leaving so the post-reboot boot
                    // flow can publish `logs/firmware_update` with both versions.
                    nvs.set_str("prev_version", &self.current_version.to_string())?;

                    info!("Saving new version to nvs!");
                    nvs.set_str("version", &remote_version.to_string())?;
                    nvs.set_u8("first_boot", 1)?;

                    // Note: no MQTT publish here. The `firmware_update` success
                    // event is emitted from `main.rs` on the next boot, once the
                    // new firmware has actually booted and passed validation.
                    info!("Reebooting firmware in 3 seconds...");
                    thread::sleep(Duration::from_secs(3));
                    esp_idf_svc::hal::reset::restart();
                }
                Err(e) => {
                    info!("Firmware download failed: {:?}", e);
                    // Propagate so the caller publishes a single failure event
                    // via `network::telemetry::publish_json` + `FirmwareUpdateLog`.
                    return Err(anyhow::anyhow!("Firmware download failed: {:?}", e));
                }
            }
        } else {
            info!("Firmware already up to date: {}", self.current_version);
        }
        Ok(())
    }

    // Function for downloading the binary file
    fn run_update(
        &mut self,
        remote_url: String,
        remote_version: Version,
        remote_sha256: String,
        remote_size: u64,
    ) -> Result<()> {
        info!(
            "Attempting to download and installing new version {}",
            remote_version
        );

        //let mut response = self.get_firmware(&remote_url)?;
        // Stream firmware directly using existing client
        let mut headers = vec![("accept", "application/octet-stream")];
        if let Some((key, value)) = self.build_auth_header() {
            headers.push((
                Box::leak(key.into_boxed_str()),
                Box::leak(value.into_boxed_str()),
            ));
        }

        let request = self.client.request(Method::Get, &remote_url, &headers)?;
        let mut response = request.submit()?;
        let status = response.status();
        info!("HTTP status: {}", status);
        if !(200..300).contains(&status) {
            return Err(anyhow::anyhow!("Non-success HTTP status: {}", status));
        }

        // Gets an instance of OTA
        let mut ota = EspOta::new()?;
        info!("Obtained OTA instance");
        let mut hasher = Sha256::new(); // Create SHA256 hasher

        let find_running_slot = EspOta::get_running_slot(&ota)?;
        let update_partition = EspOta::get_update_slot(&ota)?;

        info!("This is the running boot slot {:?}", find_running_slot);
        info!(
            "This is the next boot slot where a new update will be saved {:?}",
            update_partition
        );

        // Initialise ota update
        info!("Waiting for 5 seconds before initiating OTA update");
        thread::sleep(Duration::from_secs(5));
        let mut update = Some(ota.initiate_update()?);
        info!("OTA update has been initialised");

        // Read and write chunks to flash
        let mut buf = [0u8; 4096];

        // Setting progress variable
        let mut progress: f64 = 0.0;

        loop {
            // Read from the ESP-IDF specific reader
            let bytes_read = match response.read(&mut buf) {
                Ok(0) => break, // Reached the end of the response body
                Ok(n) => n,
                Err(e) if e.kind() == esp_idf_svc::io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e.into()), // Propagate the error
            };
            info!("Writing {} bytes to flash", bytes_read);

            // Write chunk to OTA partition
            if let Some(u) = update.as_mut() {
                u.write(&buf[..bytes_read])?; // <-- use as_mut() and unwrap Option
            }

            //update.write(&buf[..bytes_read])?;            GPT SUGGEST1

            // Update SHA256
            hasher.update(&buf[..bytes_read]);

            // Progress info
            progress += (bytes_read as f64 / remote_size as f64) * 100.0;
            info!("Progress: {:.2}%", progress);
        }
        info!("OTA update written, verifying checksum…");

        // Finalize hash and compare with expected
        let calculated_sha = hasher.finalize().to_vec();

        // Convert the hex string from manifest into raw bytes
        let expected_sha = hex::decode(&remote_sha256)
            .map_err(|_| anyhow::anyhow!("Invalid SHA256 hex string in manifest"))?;

        if calculated_sha != expected_sha {
            if let Some(u) = update.take() {
                u.abort()?; // explicitly end OTA
            }
            return Err(anyhow::anyhow!("SHA256 mismatch"));

            /* error!("SHA256 mismatch, aborting update");
            update.abort()?; // discard bad image               GPT SUGGEST1
            return Err(anyhow::anyhow!("SHA256 mismatch")); */
        }

        info!("Firmware checksum validated successfully & OTA complete, rebooting...");

        // Finish writing OTA image
        if let Some(u) = update.take() {
            u.complete()?; // mark valid
        }

        //update.complete()?; // Mark firmware as valid         GPT SUGGEST1
        Ok(())
    }
}

/* info!("Starting http run...");
let mut client = Box::new(HttpsClient::new_https(Some("device5"), Some("device5"))?); */
