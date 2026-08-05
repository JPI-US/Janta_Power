pub mod wifi {
    use std::{
        net::{IpAddr, Ipv4Addr},
        thread,
        time::Duration,
    };

    use anyhow::anyhow;
    use esp_idf_svc::wifi::{
        AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi, PmfConfiguration,
        ScanMethod,
        /*  WifiWait*/
    };
    use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};
    use log::*;

    /// Represents Wi-Fi connection states
    #[derive(Debug, PartialEq)]
    pub enum WifiState {
        Disconnected,
        Connecting,
        Connected(std::net::IpAddr),
    }

    /// The main Wi-Fi service abstraction
    pub struct Wifi<'a> {
        inner: BlockingWifi<EspWifi<'a>>,
    }

    impl<'a> Wifi<'a> {
        /// Create and configure a new Wi-Fi manager
        pub fn new(
            modem: esp_idf_svc::hal::modem::Modem,
            sysloop: EspSystemEventLoop,
            nvs: EspDefaultNvsPartition,
            ssid: &str,
            pass: &str,
        ) -> anyhow::Result<Self> {
            let esp_wifi = EspWifi::new(modem, sysloop.clone(), Some(nvs))?;
            let mut blocking = BlockingWifi::wrap(esp_wifi, sysloop)?;

            blocking.set_configuration(&Configuration::Client(ClientConfiguration {
                ssid: {
                    let mut s = heapless::String::<32>::new();
                    s.push_str(ssid)
                        .map_err(|_| anyhow!("Wifi SSID expected 32 bytes"))?;
                    s
                },
                password: {
                    let mut p = heapless::String::<64>::new();
                    p.push_str(pass)
                        .map_err(|_| anyhow!("Wifi password expected 64 bytes"))?;
                    p
                },
                auth_method: AuthMethod::WPA2Personal,
                scan_method: ScanMethod::FastScan,
                pmf_cfg: PmfConfiguration::NotCapable,
                ..Default::default()
            }))?;

            blocking.start()?;

            Ok(Wifi { inner: blocking })
        }

        /// Configure and connect to a Wi-Fi network
        pub fn connect(&mut self) -> anyhow::Result<()> {
            self.inner.connect()?;
            self.inner.wait_netif_up()?;

            // Wait up to 10s for connection
            thread::sleep(Duration::from_secs(10));

            if !self.inner.is_connected()? {
                return Err(anyhow::anyhow!("WiFi connection timeout"));
            }

            Ok(())
        }

        pub fn state(&self) -> WifiState {
            if let Ok(true) = self.inner.is_connected() {
                if let Ok(ip_info) = self.inner.wifi().sta_netif().get_ip_info() {
                    let v4: Ipv4Addr = ip_info.ip;
                    return WifiState::Connected(IpAddr::V4(v4));
                }
                WifiState::Connecting
            } else {
                WifiState::Disconnected
            }
        }

        /// Best-effort reconnect: never propagates an error to the caller.
        /// A flaky AP/router can make `connect`/`wifi_wait_while` fail
        /// (e.g. ESP_ERR_TIMEOUT) repeatedly; callers run this every tracking
        /// loop iteration and must keep moving the tower regardless of Wi-Fi
        /// state, so failures are logged and swallowed here instead.
        /// `start()` is intentionally omitted — the driver stays started after
        /// a connection drop, so calling it again returns ESP_ERR_INVALID_STATE
        /// and short-circuits before connect() can run.
        pub fn reconnect_if_disconnected(&mut self) -> anyhow::Result<()> {
            if self.state() != WifiState::Disconnected {
                return Ok(());
            }

            // Driver is already started; just re-associate.
            if let Err(e) = self.inner.connect() {
                warn!("Wi-Fi reconnect: connect() failed: {:?}", e);
                return Ok(());
            }

            // 30 s — AWS IoT needs public DNS + full mTLS handshake before
            // MQTT can reconnect; 10 s is too tight over a real WAN link.
            if let Err(e) = self.inner.wifi_wait_while(
                || Ok(self.state() == WifiState::Disconnected),
                Some(Duration::from_secs(30)),
            ) {
                warn!("Wi-Fi reconnect: wait timed out: {:?}", e);
                return Ok(());
            }

            if matches!(self.state(), WifiState::Connected(_)) {
                info!("Successfully reconnected to Wi-Fi.");
            } else {
                warn!("Failed to reconnect to Wi-Fi within 30 seconds.");
            }

            Ok(())
        }

        /// Disconnect from Wi-Fi
        pub fn disconnect(&mut self) -> anyhow::Result<()> {
            self.inner.disconnect()?;
            Ok(())
        }
    }
}
