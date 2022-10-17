use clap::Parser;
use serialport::SerialPort;
use vedirect::{Events, VEError};

struct Listener;

impl Events<vedirect::MPPT> for Listener {
    fn on_complete_block(&mut self, block: vedirect::MPPT) {
        // println!("Mapped data {:#?}", &block);
        println!(
            "{};{};{};{};{};{};{};{};{};{};{};{};{};{:?}",
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
        /*
                   assert_eq!(data.channel1_voltage, 12.54);
                  assert_eq!(data.battery_current, 0.04);
                  assert_eq!(data.panel_voltage, 18.54);
                  assert_eq!(data.panel_power, 5);
                  assert_eq!(data.load_current, 0.3);
                  assert_eq!(data.load_output_state, true);
                  assert_eq!(data.yield_total, 144);
                  assert_eq!(data.yield_today, 1);
                  assert_eq!(data.yield_yesterday, 4);
                  assert_eq!(data.max_power_today, 6);
                  assert_eq!(data.max_power_yesterday, 14);
                  assert_eq!(data.day_sequence, 16);
                  assert_eq!(data.firmware, 159);
                  assert_eq!(data.tracker_mode, TrackerOperationMode::MPPTrackerActive);

        */
    }

    fn on_parse_error(&mut self, _error: VEError, _parse_buf: &[u8]) {}
}

pub fn record(mut port: Box<dyn SerialPort>) -> anyhow::Result<()> {
    let mut buf: Vec<u8> = vec![0; 1024];
    let mut listener = Listener {};
    let mut parser = vedirect::Parser::new(&mut listener);
    loop {
        let r = port.read(buf.as_mut_slice())?;
        parser.feed(&buf[..r]); // <- FIX ME: handle errors
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
