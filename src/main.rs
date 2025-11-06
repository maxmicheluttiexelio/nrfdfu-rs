use clap::Parser;
use serialport::available_ports;
use std::{error::Error, fs};

#[macro_use]
mod macros;
mod connection;
mod messages;
mod radio_manifest;
mod slip;

use crate::connection::BootloaderConnection;
use crate::radio_manifest::RadioManifestJSONObject;
use messages::*;

type Result<T> = std::result::Result<T, Box<dyn Error>>;

const USB_VID: u16 = 0x1915;
const USB_PID: u16 = 0x521f;

/// Bootloader protocol version we support.
const PROTOCOL_VERSION: u8 = 1;

fn main() {
    match run() {
        Ok(()) => {}
        Err(e) => {
            if e.downcast_ref::<PreviousErrors>().is_none() {
                eprintln!("error: {}", e);
            }
            std::process::exit(1);
        }
    }
}
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long, global = true, action = clap::ArgAction::Count, default_value = "0", help = "Set log level"
    )]
    verbose: u8,

    /// Directory with files to flash
    path: Option<std::path::PathBuf>,

    #[arg(
        short,
        long,
        global = true,
        default_value = "false",
        help = "Update all devices"
    )]
    all: bool,

    #[arg(
        short('x'),
        long,
        global = true,
        default_value = "false",
        help = "Send abort after updating a device"
    )]
    abort: bool,

    #[arg(
        short('i'),
        long,
        global = true,
        default_value = "false",
        help = "Read firmware images (before updating)"
    )]
    get_images: bool,

    #[arg(
        short('p'),
        long,
        global = true,
        default_value = "false",
        help = "Select a serial port"
    )]
    port: Option<String>,
}

/// Previous errors occurred and were printed.
///
/// This error is explicitly *not* shown in main (because the errors were printed already).
#[derive(Debug)]
struct PreviousErrors;

impl std::fmt::Display for PreviousErrors {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Previous errors occurred")
    }
}

impl std::error::Error for PreviousErrors {}

fn run() -> Result<()> {
    // We show info and higher levels by default, but allow overriding this via `RUST_LOG`.
    let args = Args::parse();

    set_logger(args.verbose);

    let image = if let Some(path) = args.path.as_ref() {
        let manifest_path = path.join("manifest.json");
        let manifest_contents = fs::read_to_string(&manifest_path)?;
        Some(serde_json::from_str::<RadioManifestJSONObject>(
            &manifest_contents,
        )?)
    } else {
        None
    };

    let matching_ports: Vec<_> = available_ports()?
       .into_iter()
       .filter(|port|{
            if let Some(chosen_port) = &args.port &&
                port.port_name != *chosen_port {
                return false;
            }
            match &port.port_type {
                serialport::SerialPortType::UsbPort(usb) => {
                    // macOS presents two serial devices - /dev/tty* for receiving
                    // incoming data and /dev/cu for dialling out. We can use
                    // either, but we only want one of them, so hide the /dev/tty
                    // devices.
                    #[cfg(target_os = "macos")]
                    if port.port_name.starts_with("/dev/tty") {
                        return false;
                    }
                    
                    usb.vid == USB_VID && usb.pid == USB_PID
                },
                _ => false,
            }
       })
       .collect();

    match matching_ports.len() {
        0 => {
            return Err(
                "no matching USB serial device found.\n       Remember to put the \
                                 device in bootloader mode by pressing the reset button!"
                    .into(),
            );
        }
        1 => (),
        _ => {
            if !args.all {
                eprintln!("Too many nRF devices found in DFU");
                std::process::exit(1);
            }
        }
    };

    let mut error_in_all = false;

    for port in matching_ports {
        let result = run_on_port(
            &port,
            args.path.as_ref(),
            image.clone(),
            args.get_images,
            args.abort,
        );
        if let Err(e) = result {
            eprintln!("error processing {}: {e}", &port.port_name);
            if !args.all {
                return Err(e);
            } else {
                error_in_all = true;
            }
        }
    }

    if error_in_all {
        eprintln!("At least one update failed");
        std::process::exit(1);
    }

    Ok(())
}

fn run_on_port(
    port: &serialport::SerialPortInfo,
    path: Option<&std::path::PathBuf>,
    image: Option<RadioManifestJSONObject>,
    get_images: bool,
    abort: bool,
) -> Result<()> {
    log::debug!("opening {} (type {:?})", port.port_name, port.port_type);
    let serial = match &port.port_type {
        serialport::SerialPortType::UsbPort(s) => s.serial_number.as_ref(),
        // Those don't get through filtering anyway
        _ => None,
    };
    println!(
        "Found port {} (serial: {})",
        port.port_name,
        serial
            .map(|s| format!("{:?}", s))
            .unwrap_or("unknown".into())
    );

    let mut conn = BootloaderConnection::wait_connection(port);
    // Disable receipt notification. USB is a reliable transport.
    conn.set_receipt_notification(0)?;

    let obj_select = conn.select_object_command();
    log::debug!("select object response: {:?}", obj_select);

    let version = conn.fetch_protocol_version()?;
    log::debug!("protocol version: {}", version);

    let hw_version = conn.fetch_hardware_version()?;
    log::debug!("hardware version: {:?}", hw_version);

    if get_images {
        let bootloader_version = conn.fetch_firmware_version(0)?;
        println!("* image 0: {}", bootloader_version);

        let primary_version = conn.fetch_firmware_version(1)?;
        println!("* image 1: {}", primary_version);

        if primary_version.type_ == Some(FirmwareType::Softdevice) {
            let secondary_version = conn.fetch_firmware_version(2)?;
            println!("* image 2: {:#?}", secondary_version);
        }
    }
    drop(conn);

    if let Some(path) = path
        && let Some(image) = image
    {
        for image_item in image.manifest.into_iter().flatten() {
            let mut conn = BootloaderConnection::wait_connection(port);
            // Disable receipt notification. USB is a reliable transport.
            conn.set_receipt_notification(0)?;

            let init_packet_path = path.join(image_item.dat_file);
            let init_packet_data = fs::read(&init_packet_path)?;
            let firmware_path = path.join(image_item.bin_file);
            let firmware_data = fs::read(&firmware_path)?;
            conn.send_init_packet(&init_packet_data)?;
            conn.send_firmware(&firmware_data)?;

            if abort {
                let abort_result = conn.abort();

                if abort_result.is_ok() {
                    log::warn!("Response received to Abort command (expected USB disconnect)");
                }
            }
        }
    }

    Ok(())
}

/// Initializes the logger using the provided log level.
fn set_logger(verbose: u8) {
    let log_level = match verbose {
        0 => log::LevelFilter::Off,
        1 => log::LevelFilter::Error,
        2 => log::LevelFilter::Warn,
        3 => log::LevelFilter::Info,
        4 => log::LevelFilter::Debug,
        _ => log::LevelFilter::Trace,
    };

    env_logger::builder()
        .filter_level(log_level)
        .parse_default_env()
        .init();
}
