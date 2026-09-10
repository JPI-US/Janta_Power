use std::{result::Result::Ok, thread, time::Duration};

use anyhow::{Context, Result};
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
use serde_json::Value;
use sha2::{Digest, Sha256};

pub struct OtaUpdater<'a> {
    current_version: Version,
    mqtt_client: &'a mut Mqtt,
    client: HttpClient<EspHttpConnection>,
}

impl<'a> OtaUpdater<'a> {
    pub fn new_ota(current_version: Version, mqtt_client: &'a mut Mqtt) -> Result<Self> {
        let config = EspHttpConnection::new(&HttpConfiguration {
            buffer_size: Some(4096),    // response side
            buffer_size_tx: Some(4096), // request side — presigned S3 URLs are long
            timeout: Some(Duration::from_secs(60)),
            crt_bundle_attach: Some(esp_crt_bundle_attach),
            use_global_ca_store: true,
            ..Default::default()
        })?;

        let client = HttpClient::wrap(config);

        Ok(Self {
            current_version,
            mqtt_client,
            client,
        })
    }

    // Requests the next pending AWS IoT Job over MQTT and waits for the
    // response. Returns Ok(None) if there's no pending job (device already
    // up to date, or nothing has been targeted at it).
    fn get_pending_job(&mut self) -> Result<Option<(String, Value)>> {
        let thing_name = self.mqtt_client.thing_name.clone();
        let request_topic = format!("$aws/things/{}/jobs/$next/get", thing_name);
        let accepted_topic = format!("$aws/things/{}/jobs/$next/get/accepted", thing_name);
        let rejected_topic = format!("$aws/things/{}/jobs/$next/get/rejected", thing_name);

        self.mqtt_client.subscribe(&accepted_topic)?;
        self.mqtt_client.subscribe(&rejected_topic)?;

        // Give the broker a moment to register the subscriptions before we
        // publish the request — there's no SUBACK confirmation wired up in
        // Mqtt yet, so this is a stopgap. If you see missed responses,
        // this is the first thing to make more robust.
        thread::sleep(Duration::from_millis(500));

        self.mqtt_client.publish(&request_topic, b"{}")?;

        for _ in 0..30 {
            if let Some((topic, payload)) = self
                .mqtt_client
                .wait_for_job_message(Duration::from_secs(1))
            {
                if topic == accepted_topic {
                    info!(
                        "Raw job response payload: {}",
                        String::from_utf8_lossy(&payload)
                    );
                    let json: Value = serde_json::from_slice(&payload)
                        .map_err(|e| anyhow::anyhow!("Failed to parse job response: {e}"))?;

                    return match json.get("execution") {
                        Some(execution) => {
                            let job_id = execution
                                .get("jobId")
                                .and_then(|v| v.as_str())
                                .ok_or_else(|| anyhow::anyhow!("Missing jobId in job execution"))?
                                .to_string();
                            let job_document =
                                execution.get("jobDocument").cloned().ok_or_else(|| {
                                    anyhow::anyhow!("Missing jobDocument in job execution")
                                })?;
                            Ok(Some((job_id, job_document)))
                        }
                        // Empty {} response means: no pending job right now.
                        None => Ok(None),
                    };
                }
                if topic == rejected_topic {
                    warn!(
                        "Job request rejected: {}",
                        String::from_utf8_lossy(&payload)
                    );
                    return Ok(None);
                }
                // Message on some other AWS IoT topic — ignore and keep waiting.
            }
        }

        warn!("Timed out waiting for job response");
        Ok(None)
    }

    // Reports job execution status back to AWS IoT Jobs so the console and
    // any rollout/abort configuration can see progress.
    fn report_job_status(
        &mut self,
        job_id: &str,
        status: &str,
        reason: Option<&str>,
    ) -> Result<()> {
        let thing_name = self.mqtt_client.thing_name.clone();
        let topic = format!("$aws/things/{}/jobs/{}/update", thing_name, job_id);
        let body = match reason {
            Some(r) => serde_json::json!({ "status": status, "statusDetails": { "reason": r } }),
            None => serde_json::json!({ "status": status }),
        };
        self.mqtt_client
            .publish(&topic, body.to_string().as_bytes())?;
        Ok(())
    }

    pub fn run_version_compare<T: NvsPartitionId>(&mut self, nvs: &mut EspNvs<T>) -> Result<()> {
        let (job_id, job_document) = match self.get_pending_job()? {
            Some(v) => v,
            None => {
                info!(
                    "No pending job — firmware already up to date: {}",
                    self.current_version
                );
                return Ok(());
            }
        };

        // Extact the "version" field from the job document and verify its not empty
        let remote_version: Version = job_document
            .get("version")
            .and_then(|v| v.as_str())
            .ok_or_else(|| anyhow::anyhow!("Missing or invalid 'version' field in job document"))?
            .trim()
            .parse()?;

        // Extact the "size" field from the job document and verify its not empty
        let remote_size = job_document
            .get("size")
            .and_then(|s| s.as_u64())
            .ok_or_else(|| anyhow::anyhow!("Missing or invalid 'size' field in job document"))?;

        if remote_size == 0 {
            return Err(anyhow::anyhow!("'size' field is zero"));
        }

        // Extract download_url
        let remote_url = job_document
            .get("download_url")
            .and_then(|u| u.as_str())
            .ok_or_else(|| {
                anyhow::anyhow!("Missing or invalid 'download_url' field in job document")
            })?
            .trim()
            .to_string();

        if remote_url.is_empty() {
            return Err(anyhow::anyhow!("'download_url' field is empty"));
        }

        // "sha256" arrives as "sha256:<hex>" — strip the prefix if present.
        // Validate sha256 length (must be 64 hex chars → 32 bytes)
        let remote_checksum_raw = job_document
            .get("sha256")
            .and_then(|h| h.as_str())
            .ok_or_else(|| anyhow::anyhow!("Missing or invalid 'sha256' field in job document"))?
            .trim();
        let remote_sha256 = remote_checksum_raw
            .strip_prefix("sha256:")
            .unwrap_or(remote_checksum_raw)
            .to_string();

        if remote_sha256.len() != 64 {
            return Err(anyhow::anyhow!("'sha256' must decode to 64 hex characters"));
        }

        if hex::decode(&remote_sha256).is_err() {
            return Err(anyhow::anyhow!("'sha256' is not valid hex"));
        }

        // NOTE: no "sig" field present in this job document — this build is
        // checksum-only (detects corruption, not tampering). Fine as a
        // deliberate short-term choice at small fleet scale; add signature
        // verification here later if you want it.

        info!("Here is the current remote version: {remote_version}");
        info!(
            "Here is the current firmware version: {}",
            self.current_version
        );

        info!("Here is the current download url: {remote_url}");
        info!("Here is the current sha256: {remote_sha256}");
        info!("Here is the current download size: {remote_size}");

        if remote_version > self.current_version {
            info!("New firmware version detected — starting OTA (job {job_id})");
            self.report_job_status(&job_id, "IN_PROGRESS", None)?;

            info!("Waiting 5 seconds before running firmware download...");
            thread::sleep(Duration::from_secs(5));
            let flash_result = self.run_update(remote_url, remote_sha256, remote_size);

            match flash_result {
                Ok(_) => {
                    nvs.set_u8("first_boot", 1)?;

                    // Don't report SUCCEEDED yet — the new image hasn't
                    // booted, let alone passed validation. Stash the job id
                    // so the new firmware can confirm it after boot
                    // validation passes (see `confirm_pending_job`); if it
                    // never boots cleanly and rolls back instead, this job
                    // is left IN_PROGRESS rather than falsely marked as
                    // succeeded.
                    nvs.set_str("pending_job_id", &job_id)?;

                    info!("Reebooting firmware in 3 seconds...");
                    thread::sleep(Duration::from_secs(3));
                    esp_idf_svc::hal::reset::restart();
                }
                Err(e) => {
                    info!("Firmware download failed: {:?}", e);
                    self.report_job_status(&job_id, "FAILED", Some(&e.to_string()))?;
                    return Err(anyhow::anyhow!("Firmware download failed: {:?}", e));
                }
            }
        } else {
            info!(
                "Job present but firmware already up to date: {}",
                self.current_version
            );
            // Acknowledge the job so it doesn't stay QUEUED forever against this device.
            self.report_job_status(&job_id, "SUCCEEDED", Some("already up to date"))?;
        }

        Ok(())
    }

    // Downloads from a presigned S3 URL, which carries its own SigV4
    // signature in the query string. Device Basic Auth was part of the old
    // firmware-host setup and must not be sent here — an Authorization
    // header alongside the presigned signature makes S3 reject the request
    // with 400 ("Only one auth mechanism allowed").
    fn run_update(
        &mut self,
        remote_url: String,
        remote_sha256: String,
        remote_size: u64,
    ) -> Result<()> {
        info!("Attempting to download and install new firmware...");

        let headers = vec![("accept", "application/octet-stream")];
        let request = self.client.request(Method::Get, &remote_url, &headers)?;
        let mut response = request.submit()?;
        let status = response.status();
        info!("HTTP status: {}", status);
        if !(200..300).contains(&status) {
            return Err(anyhow::anyhow!("Non-success HTTP status: {}", status));
        }

        // Gets an instance of OTA
        let mut ota = EspOta::new().context("Failed to obtain OTA instance")?;
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
        let mut update = Some(
            ota.initiate_update()
                .context("Failed to initiate OTA update")?,
        );
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
        }

        info!("Firmware checksum validated successfully & OTA complete, rebooting...");

        // Finish writing OTA image
        if let Some(u) = update.take() {
            u.complete()?; // mark valid
        }

        Ok(())
    }
}

/// Reports SUCCEEDED for an OTA job that finished flashing on the previous
/// boot (see the `Ok(_)` branch of `run_version_compare`) but was only
/// confirmed to AWS IoT Jobs once this boot has actually passed validation.
/// Call once, right after `EspOta::mark_running_slot_valid()` succeeds.
/// No-op if there's no pending job recorded.
pub fn confirm_pending_job<T: NvsPartitionId>(mqtt: &mut Mqtt, nvs: &mut EspNvs<T>) -> Result<()> {
    let mut buf = [0u8; 64];
    let Some(job_id) = nvs.get_str("pending_job_id", &mut buf)? else {
        return Ok(());
    };
    let job_id = job_id.trim_end_matches('\0').to_string();

    let thing_name = mqtt.thing_name.clone();
    let topic = format!("$aws/things/{}/jobs/{}/update", thing_name, job_id);
    let body = serde_json::json!({ "status": "SUCCEEDED", "statusDetails": { "reason": "boot validated" } });
    mqtt.publish(&topic, body.to_string().as_bytes())?;

    nvs.remove("pending_job_id")?;
    info!("Confirmed OTA job {job_id} as SUCCEEDED after boot validation");

    Ok(())
}
