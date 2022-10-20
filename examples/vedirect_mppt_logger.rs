use std::time::UNIX_EPOCH;

use clap::Parser;
use serialport::SerialPort;
use vedirect::{Events, VEError};

struct Listener;

impl Events<vedirect::MPPT> for Listener {
    fn on_complete_block(&mut self, block: vedirect::MPPT) {
        println!(
            "{};{};{};{};{};{};{};{};{};{};{};{};{};{};{:?}",
            std::time::SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs(),
            block.channel1_voltage,
            block.battery_current,
            block.panel_voltage,
            block.panel_power,
            block.load_current,
            block.load_output_state,
            block.yield_total,
            block.yield_today,
            block.yield_yesterday,
            block.max_power_today,
            block.max_power_yesterday,
            block.day_sequence,
            block.firmware,
            block.tracker_mode,
        );
    }

    fn on_parse_error(&mut self, _error: VEError, _parse_buf: &[u8]) {}
}

pub fn record(mut port: Box<dyn SerialPort>) -> anyhow::Result<()> {
    let mut buf: Vec<u8> = vec![0; 1024];
    let mut listener = Listener {};
    let mut parser = vedirect::Parser::new(&mut listener);
    loop {
        let r = port.read(buf.as_mut_slice())?;
        let _ = parser.feed(&buf[..r]);
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serial device, on linux e.g. /dev/ttyUSB0
    #[arg(short, long)]
    device: String,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let port = serialport::new(args.device, 19_200)
        .data_bits(serialport::DataBits::Eight)
        .timeout(core::time::Duration::from_secs(2))
        .open()
        .expect("Failed to open vedirect serial port");
    record(port)?;
    Ok(())
}
