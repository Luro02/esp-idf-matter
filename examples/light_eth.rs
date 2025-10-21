//! An example utilizing the `EspEthMatterStack` struct.
//! As the name suggests, this Matter stack assembly uses Ethernet as the main transport, as well as for commissioning.
//!
//! Notice thart we actually don't use Ethernet for real, as ESP32s don't have Ethernet ports out of the box.
//! Instead, we utilize Wifi, which - from the POV of Matter - is indistinguishable from Ethernet as long as the Matter
//! stack is not concerned with connecting to the Wifi network, managing its credentials etc. and can assume it "pre-exists".
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![allow(unexpected_cfgs)]
#![recursion_limit = "256"]

fn main() -> Result<(), anyhow::Error> {
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6))]
    {
        example::main()
    }

    #[cfg(not(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6)))]
    panic!("This example is only supported on ESP32, ESP32-S2, ESP32-S3, ESP32-C3 and ESP32-C6 chips. Please select a different example or target.");
}

#[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6))]
mod example {
    use core::pin::pin;

    use alloc::sync::Arc;

    use esp_idf_matter::eth::EspEthMatterStack;
    use esp_idf_matter::init_async_io;
    use esp_idf_matter::matter::dm::clusters::desc::{ClusterHandler as _, DescHandler};
    use esp_idf_matter::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
    use esp_idf_matter::matter::dm::clusters::on_off::test::TestOnOffDeviceLogic;
    use esp_idf_matter::matter::dm::clusters::on_off::{self, OnOffHandler, OnOffHooks};
    use esp_idf_matter::matter::dm::devices::test::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
    use esp_idf_matter::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
    use esp_idf_matter::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node};
    use esp_idf_matter::matter::utils::init::InitMaybeUninit;
    use esp_idf_matter::matter::{clusters, devices};
    use esp_idf_matter::netif::{EspMatterNetStack, EspMatterNetif};

    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::hal::task::block_on;
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    use esp_idf_svc::handle::RawHandle;
    use esp_idf_svc::io::vfs::MountedEventfs;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use esp_idf_svc::sys::{esp, esp_netif_create_ip6_linklocal};
    use esp_idf_svc::timer::EspTaskTimerService;
    use esp_idf_svc::wifi::{self, AsyncWifi, EspWifi};

    use log::{error, info};

    use rs_matter_stack::mdns::BuiltinMdns;
    use static_cell::StaticCell;

    extern crate alloc;

    const STACK_SIZE: usize = 20 * 1024; // Can go down to 15K for esp32c6
    const BUMP_SIZE: usize = 13500;

    const WIFI_SSID: &str = env!("WIFI_SSID");
    const WIFI_PASS: &str = env!("WIFI_PASS");

    pub fn main() -> Result<(), anyhow::Error> {
        esp_idf_svc::log::init_from_env();

        info!("Starting...");

        ThreadSpawnConfiguration::set(&ThreadSpawnConfiguration {
            name: Some(c"matter"),
            ..Default::default()
        })?;

        // Run in a higher-prio thread to avoid issues with `async-io` getting
        // confused by the low priority of the ESP IDF main task
        // Also allocate a very large stack (for now) as `rs-matter` futures do occupy quite some space
        let thread = std::thread::Builder::new()
            .stack_size(STACK_SIZE)
            .spawn(run)
            .unwrap();

        thread.join().unwrap()
    }

    #[inline(never)]
    #[cold]
    fn run() -> Result<(), anyhow::Error> {
        let result = block_on(matter());

        if let Err(e) = &result {
            error!("Matter aborted execution with error: {e:?}");
        }
        {
            info!("Matter finished execution successfully");
        }

        result
    }

    async fn matter() -> Result<(), anyhow::Error> {
        // Initialize the Matter stack (can be done only once),
        // as we'll run it in this thread
        let stack = MATTER_STACK
            .uninit()
            .init_with(EspEthMatterStack::init_default(
                &TEST_DEV_DET,
                TEST_DEV_COMM,
                &TEST_DEV_ATT,
            ));

        // Take some generic ESP-IDF stuff we'll need later
        let sysloop = EspSystemEventLoop::take()?;
        let nvs = EspDefaultNvsPartition::take()?;
        let peripherals = Peripherals::take()?;

        let mounted_event_fs = Arc::new(MountedEventfs::mount(3)?);
        init_async_io(mounted_event_fs.clone())?;

        // Configure and start the Wifi first
        let mut wifi = Box::new(AsyncWifi::wrap(
            EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs.clone()))?,
            sysloop.clone(),
            EspTaskTimerService::new()?,
        )?);
        wifi.set_configuration(&wifi::Configuration::Client(wifi::ClientConfiguration {
            ssid: WIFI_SSID.try_into().unwrap(),
            password: WIFI_PASS.try_into().unwrap(),
            ..Default::default()
        }))?;
        wifi.start().await?;
        wifi.connect().await?;

        // Matter needs an IPv6 address to work
        esp!(unsafe { esp_netif_create_ip6_linklocal(wifi.wifi().sta_netif().handle() as _) })?;

        wifi.wait_netif_up().await?;

        // Our "light" on-off cluster.
        // It will toggle the light state every 5 seconds
        let on_off = OnOffHandler::new_standalone(
            Dataver::new_rand(stack.matter().rand()),
            LIGHT_ENDPOINT_ID,
            TestOnOffDeviceLogic::new(true),
        );

        // Chain our endpoint clusters with the
        // (root) Endpoint 0 system clusters in the final handler
        let handler = EmptyHandler
            // Our on-off cluster, on Endpoint 1
            .chain(
                EpClMatcher::new(
                    Some(LIGHT_ENDPOINT_ID),
                    Some(TestOnOffDeviceLogic::CLUSTER.id),
                ),
                on_off::HandlerAsyncAdaptor(&on_off),
            )
            // Each Endpoint needs a Descriptor cluster too
            // Just use the one that `rs-matter` provides out of the box
            .chain(
                EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(DescHandler::CLUSTER.id)),
                Async(DescHandler::new(Dataver::new_rand(stack.matter().rand())).adapt()),
            );

        // Create the persister & load any previously saved state
        // `EspKvBlobStore` saves to a user-supplied ESP-IDF NVS partition
        // However, for this demo and for simplicity, we use a dummy persister that does nothing
        let persist = stack
            .create_persist_with_comm_window(rs_matter_stack::persist::DummyKvBlobStore)
            .await?;

        // Run the Matter stack with our handler
        // Using `pin!` is completely optional, but reduces the size of the final future
        let matter = pin!(stack.run_preex(
            // The Matter stack needs UDP sockets to communicate with other Matter devices
            EspMatterNetStack::new(),
            // The Matter stack need access to the netif on which we'll operate
            // Since we are pretending to use a wired Ethernet connection - yet -
            // we are using a Wifi STA - provide the Wifi netif here
            EspMatterNetif::new(wifi.wifi().sta_netif(), InterfaceTypeEnum::WiFi, sysloop),
            // The Matter stack needs an mDNS service to advertise itself
            BuiltinMdns,
            // The Matter stack needs a persister to store its state
            &persist,
            // Our `AsyncHandler` + `AsyncMetadata` impl
            (NODE, handler),
            // No user future to run
            (),
        ));

        // Run Matter
        matter.await?;

        Ok(())
    }

    /// The Matter stack is allocated statically to avoid
    /// program stack blowups.
    static MATTER_STACK: StaticCell<EspEthMatterStack<BUMP_SIZE, ()>> = StaticCell::new();

    /// Endpoint 0 (the root endpoint) always runs
    /// the hidden Matter system clusters, so we pick ID=1
    const LIGHT_ENDPOINT_ID: u16 = 1;

    /// The Matter Light device Node
    const NODE: Node = Node {
        id: 0,
        endpoints: &[
            EspEthMatterStack::<0, ()>::root_endpoint(),
            Endpoint {
                id: LIGHT_ENDPOINT_ID,
                device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
                clusters: clusters!(DescHandler::CLUSTER, TestOnOffDeviceLogic::CLUSTER),
            },
        ],
    };
}
