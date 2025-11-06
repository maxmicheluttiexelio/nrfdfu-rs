use crate::messages::{
    AbortRequest, AbortResponse, CrcRequest, CrcResponse, CreateObjectRequest, ExecuteRequest,
    ExecuteResponse, FirmwareVersionRequest, FirmwareVersionResponse, GetMtuRequest,
    HardwareVersionRequest, HardwareVersionResponse, ObjectType, ProtocolVersionRequest, Request,
    SelectRequest, SelectResponse, SetPrnRequest, WriteRequest,
};
use crate::{PROTOCOL_VERSION, messages, slip};
use serialport::{SerialPort, SerialPortInfo};
use std::convert::TryInto;
use std::hash::Hasher;
use std::time::Duration;

pub struct BootloaderConnection {
    serial: Box<dyn SerialPort>,
    buf: Vec<u8>,
    mtu: u16,
}

impl BootloaderConnection {
    pub fn wait_connection(port: &SerialPortInfo) -> Self {
        loop {
            let port = loop {
                if let Ok(port) = serialport::new(&port.port_name, 115200)
                    .timeout(Duration::from_millis(5000))
                    .open()
                {
                    break port;
                }
                std::thread::sleep(Duration::from_millis(100));
            };
            if let Ok(c) = BootloaderConnection::new(port) {
                break c;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    pub fn new(serial: Box<dyn SerialPort>) -> crate::Result<Self> {
        let mut this = Self {
            serial,
            buf: Vec::new(),
            mtu: 0,
        };

        // We must check the protocol version before doing anything else, since any other command
        // might change if the version changes.
        let proto_version = this.fetch_protocol_version()?;
        if proto_version != PROTOCOL_VERSION {
            return Err(format!(
                "device reports protocol version {}, we only support {}",
                proto_version, PROTOCOL_VERSION
            )
            .into());
        }

        let mtu = this.fetch_mtu()?;
        log::debug!("MTU = {} Bytes", mtu);
        this.mtu = mtu;
        Ok(this)
    }

    /// send `req` and do not fetch any response
    fn request<R: Request>(&mut self, req: R) -> crate::Result<()> {
        let mut buf = vec![R::OPCODE as u8];
        req.write_payload(&mut buf)?;
        log::trace!("--> {:?}", buf);

        // Go through an intermediate buffer to avoid writing every byte individually.
        self.buf.clear();
        slip::encode_frame(&buf, &mut self.buf)?;
        self.serial
            .write_all(&self.buf)
            .map_err(|e| format!("error while writing to serial port: {}", e))?;
        self.serial.flush()?;

        Ok(())
    }

    /// send `req` and expect a response.
    /// aborts if no response is received within timeout window.
    fn request_response<R: Request>(&mut self, req: R) -> crate::Result<R::Response> {
        self.request(req)?;

        self.buf.clear();
        slip::decode_frame(&mut self.serial, &mut self.buf)
            .map_err(|e| format!("error while reading from serial port: {}", e))?;
        log::trace!("<-- {:?}", self.buf);

        messages::parse_response::<R>(&self.buf)
    }

    pub fn fetch_protocol_version(&mut self) -> crate::Result<u8> {
        let response = self.request_response(ProtocolVersionRequest);
        match response {
            Ok(version_response) => Ok(version_response.version),
            Err(e) => Err(e),
        }
    }

    pub fn fetch_hardware_version(&mut self) -> crate::Result<HardwareVersionResponse> {
        self.request_response(HardwareVersionRequest)
    }

    pub fn fetch_firmware_version(&mut self, image: u8) -> crate::Result<FirmwareVersionResponse> {
        self.request_response(FirmwareVersionRequest(image))
    }

    /// Sends the `.dat` file that's zipped into our firmware DFU .zip(?)
    /// modeled after `pc-nrfutil`s `dfu_transport_serial::send_init_packet()`
    pub fn send_init_packet(&mut self, data: &[u8]) -> crate::Result<()> {
        log::info!("Sending init packet...");
        let select_response = self.select_object_command()?;
        log::debug!("Object selected: {:?}", select_response);

        let data_size = data.len() as u32;

        log::debug!("Creating Command...");
        self.create_command_object(data_size)?;
        log::debug!("Command created");

        log::debug!("Streaming Data: len: {}", data_size);
        self.stream_object_data(data)?;

        let received_crc = self.get_crc()?.crc;
        self.check_crc(data, received_crc, 0)?;

        self.execute()?;

        Ok(())
    }

    /// Sends the firmware image at `bin_path`.
    /// This is done in chunks to avoid exceeding our MTU  and involves periodic CRC checks.
    pub fn send_firmware(&mut self, image: &[u8]) -> crate::Result<()> {
        log::info!("Sending firmware image of size {}...", image.len());

        log::debug!("Selecting Object: type Data");
        let select_response = self.select_object_data()?;
        log::debug!("Object selected: {:?}", select_response);

        let max_size = select_response.max_size;
        let mut prev_chunk_crc: u32 = 0;

        for chunk in image.chunks(max_size.try_into().unwrap()) {
            let curr_chunk_sz: u32 = chunk.len().try_into().unwrap();
            self.create_data_object(curr_chunk_sz)?;
            log::debug!("Streaming Data: len: {}", curr_chunk_sz);

            self.stream_object_data(chunk)?;

            let received_crc = self.get_crc()?;
            log::debug!("crc response: {:?}", received_crc);
            prev_chunk_crc = self.check_crc(chunk, received_crc.crc, prev_chunk_crc)?;

            self.execute()?;
        }

        log::info!("Done.");
        Ok(())
    }

    fn check_crc(&self, data: &[u8], received_crc: u32, initial: u32) -> crate::Result<u32> {
        let mut digest = crc32fast::Hasher::new_with_initial(initial);
        digest.write(data);
        let expected_crc = digest.finalize();

        if expected_crc == received_crc {
            log::debug!("crc passed.");
            Ok(expected_crc)
        } else {
            let err_msg = format!(
                "crc failed: expected {} - received {}",
                expected_crc, received_crc
            );
            log::debug!("{}", err_msg);
            Err(err_msg.into())
        }
    }

    /// Sends a
    /// Request Type: `Select`
    /// Parameters:   `Object type = Command`
    pub fn select_object_command(&mut self) -> crate::Result<SelectResponse> {
        self.request_response(SelectRequest(ObjectType::Command))
    }

    /// Sends a
    /// Request Type: `Select`
    /// Parameters:   `Object type = Data`
    fn select_object_data(&mut self) -> crate::Result<SelectResponse> {
        self.request_response(SelectRequest(ObjectType::Data))
    }

    /// Sends a
    /// Request Type: `Create`
    /// Parameters:   `Object type = Command`
    ///               `size`
    pub fn create_command_object(&mut self, size: u32) -> crate::Result<()> {
        self.request_response(CreateObjectRequest {
            obj_type: ObjectType::Command,
            size,
        })?;
        Ok(())
    }

    /// Sends a
    /// Request Type: `Create`
    /// Parameters:   `Object type = Data`
    ///               `size`
    pub fn create_data_object(&mut self, size: u32) -> crate::Result<()> {
        // Note: Data objects cannot be created if no init packet has been sent. This results in an
        // `OperationNotPermitted` error.
        self.request_response(CreateObjectRequest {
            obj_type: ObjectType::Data,
            size,
        })?;
        Ok(())
    }

    pub fn set_receipt_notification(&mut self, every_n_packets: u16) -> crate::Result<()> {
        log::debug!("Setting receipt notification to {every_n_packets}...");
        self.request_response(SetPrnRequest(every_n_packets))?;
        Ok(())
    }

    fn fetch_mtu(&mut self) -> crate::Result<u16> {
        log::debug!("Fetching maximum transmission unit...");
        Ok(self.request_response(GetMtuRequest)?.0)
    }

    fn stream_object_data(&mut self, data: &[u8]) -> crate::Result<()> {
        // On the wire, the write request contains the opcode byte, and is then SLIP-encoded,
        // potentially doubling the size, and adding a frame terminator, so the chunk size has
        // to be smaller than the MTU.
        let max_chunk_size = usize::from((self.mtu - 1) / 2 - 1);

        for chunk in data.chunks(max_chunk_size) {
            // TODO: this also needs to take into account the receipt response. In our case we turn
            // it off, so there's nothing to do here.
            self.request(WriteRequest {
                request_payload: chunk,
            })?;
        }

        Ok(())
    }

    fn get_crc(&mut self) -> crate::Result<CrcResponse> {
        self.request_response(CrcRequest)
    }

    // tell the target to execute whatever request setup we sent them before
    fn execute(&mut self) -> crate::Result<ExecuteResponse> {
        self.request_response(ExecuteRequest)
    }

    /// Sends the Abort command, causing a reboot without reflashing.
    pub fn abort(&mut self) -> crate::Result<AbortResponse> {
        self.request_response(AbortRequest)
    }
}
