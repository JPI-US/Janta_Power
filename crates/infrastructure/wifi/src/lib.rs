pub mod wifi {
    use anyhow;
    use log::*;
    use esp_idf_svc::wifi::{
        AuthMethod,
        BlockingWifi, 
        ClientConfiguration,
        Configuration,
        EspWifi, 
        PmfConfiguration,
        ScanMethod,
        /*  WifiWait*/
    };
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use std::time::Duration;
    use std::net::{IpAddr, Ipv4Addr};
    use std::thread;

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
        /// Create a new Wi-Fi manager
        pub fn new(
            modem: esp_idf_svc::hal::modem::Modem,
            sysloop: EspSystemEventLoop,
            nvs: EspDefaultNvsPartition,
        ) -> anyhow::Result<Self> {
            let esp_wifi = EspWifi::new(modem, sysloop.clone(), Some(nvs))?;
            let blocking = BlockingWifi::wrap(esp_wifi, sysloop)?;
            Ok(Wifi { inner: blocking })
        }

        /// Configure and connect to a Wi-Fi network
        pub fn connect(&mut self, ssid: &str, pass: &str) -> anyhow::Result<()> {
            self.inner.set_configuration(&Configuration::Client(
                ClientConfiguration {
                    ssid: { 
                        let mut s = heapless::String::<32>::new();
                        s.push_str(ssid).unwrap();
                        s
                    },
                    password: { 
                        let mut p = heapless::String::<64>::new();
                        p.push_str(pass).unwrap();
                        p
                    },
                    auth_method: AuthMethod::WPA2Personal,
                    scan_method: ScanMethod::FastScan,
                    pmf_cfg: PmfConfiguration::NotCapable,
                    ..Default::default()
                },
            ))?;

            self.inner.start()?;
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
                    let v4: Ipv4Addr = ip_info.ip.into();
                    return WifiState::Connected(IpAddr::V4(v4));
                }
                WifiState::Connecting
            } else {
                WifiState::Disconnected
            }
        }

        pub fn reconnect_if_disconnected(&mut self) -> anyhow::Result<()>{
            // Check if the Wi-Fi is disconnected
            if self.state() == WifiState::Disconnected {
                // Attempt to reconnect
                self.inner.start()?;
                self.inner.connect()?;

                // Block for up to 10 seconds while waiting for the connection to establish
                self.inner.wifi_wait_while(
                    || Ok(self.state() == WifiState::Disconnected),
                    Some(Duration::from_secs(10)),
                )?;

                // Check if the connection was successful
                if matches!(self.state(), WifiState::Connected(_)) {
                    info!("Successfully reconnected to Wi-Fi.");
                } else {
                    warn!("Failed to reconnect to Wi-Fi within 30 seconds.");
                }
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
