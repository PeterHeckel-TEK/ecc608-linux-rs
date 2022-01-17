use bytes::BytesMut;
use std::{thread, time::Duration};

use crate::{Result, Error};

use bytes::BufMut;
use serialport::{ClearBuffer, SerialPort};
use crate::{constants::{ATCA_SWI_CMD_SIZE_MAX, WAKE_DELAY}};

use i2c_linux::{I2c, ReadFlags};
use std::fs::File;

pub(crate) const RECV_RETRY_WAIT: Duration = Duration::from_millis(50);
pub(crate) const RECV_RETRIES: u8 = 2;

pub(crate) enum TransportProtocol { 
    I2c,
    Swi,
}
pub(crate) struct EccTransport
{
    swi_port: Option<Box<dyn SerialPort>>, 
    i2c_port: Option<I2c<File>>,
    i2c_address: u16,
    pub(crate) protocol: TransportProtocol,
}

impl EccTransport
{
    pub fn from_path(path: &str, address: u16) -> Result<Self> {
        
        if path.contains("/dev/tty"){
            let swi_port = serialport::new(path, 230_400)
            .data_bits(serialport::DataBits::Seven)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .timeout(Duration::from_millis(50))
            .open().unwrap_or_else(|e| {
                eprintln!("Failed to open serial port. Error: {}", e);
                ::std::process::exit(1);
            });
            
            Ok(Self {
                i2c_port: None,
                i2c_address: address,
                swi_port: Some(swi_port),
                protocol: TransportProtocol::Swi
            })    
        }
        else {
            let mut i2c = I2c::from_path(path)?;
            i2c.smbus_set_slave_address(address, false)?;
            
            Ok(Self { 
                swi_port: None,
                i2c_port: Some(i2c),
                i2c_address: address,
                protocol: TransportProtocol::I2c
            })
        }
    }

    pub fn send_wake(&mut self) -> Result {
        match self.protocol {
            TransportProtocol::I2c =>{
                let _ = self.send_i2c_buf(0, &[0]);
                Ok(())
            }
            TransportProtocol::Swi => {
                if let Err(_err) = self.swi_port.as_mut().unwrap().set_baud_rate(115_200)
                {
                    return Err(Error::timeout());
                }
                
                let _ = self.swi_port.as_mut().unwrap().write(&[0]);
                
                thread::sleep(WAKE_DELAY);
                let _ = self.swi_port.as_mut().unwrap().set_baud_rate(230_400);
                let _ = self.swi_port.as_mut().unwrap().clear(ClearBuffer::All);
                Ok(()) 
            },
        }
    }

    pub fn send_sleep(&mut self) {
        match self.protocol {
            TransportProtocol::I2c => {
                let _ = self.send_i2c_buf(self.i2c_address, &[1]);
            } 
            TransportProtocol::Swi => {
                let mut sleep_msg = BytesMut::new();
                sleep_msg.put_u8(0xCC);
                let sleep_encoded = self.encode_uart_to_swi(&sleep_msg);
        
                let _ = self.swi_port.as_mut().unwrap().write(&sleep_encoded);
                thread::sleep( Duration::from_micros(300));
            },
        }
    }

    pub fn send_recv_buf(&mut self, delay: Duration, buf: &mut BytesMut) -> Result {
        match self.protocol {
            TransportProtocol::I2c => {
                self.send_i2c_buf(self.i2c_address, &buf[..])?;
                thread::sleep(delay);
                self.recv_i2c_buf(buf)
            },
            TransportProtocol::Swi => {
                let _ = self.swi_port.as_mut().unwrap().clear(ClearBuffer::All);
                let swi_msg = self.encode_uart_to_swi(buf);
                self.send_swi_buf(&swi_msg)?;
                thread::sleep(delay);
                self.recv_swi_buf(buf)
            },
        }
    }

    fn send_i2c_buf(&mut self, address: u16, buf: &[u8]) -> Result {
        let write_msg = i2c_linux::Message::Write {
            address,
            data: buf,
            flags: Default::default(),
        };

        self.i2c_port.as_mut().unwrap().i2c_transfer(&mut [write_msg])?;
        Ok(())
    }

    fn recv_i2c_buf(&mut self, buf: &mut BytesMut) -> Result {
        unsafe { buf.set_len(1) };
        buf[0] = 0xff;
        for _retry in 0..RECV_RETRIES {
            let msg = i2c_linux::Message::Read {
                address: self.i2c_address,
                data: &mut buf[0..1],
                flags: Default::default(),
            };
            if let Err(_err) = self.i2c_port.as_mut().unwrap().i2c_transfer(&mut [msg]) {
            } else {
                break;
            }
            thread::sleep(RECV_RETRY_WAIT);
        }
        let count = buf[0] as usize;
        if count == 0xff {
            return Err(Error::timeout());
        }
        unsafe { buf.set_len(count) };
        let read_msg = i2c_linux::Message::Read {
            address: self.i2c_address,
            data: &mut buf[1..count],
            flags: ReadFlags::NO_START,
        };
        self.i2c_port.as_mut().unwrap().i2c_transfer(&mut [read_msg])?;
        Ok(())
    }

    fn send_swi_buf(&mut self, buf: &[u8]) -> Result {
        
        let send_size = self.swi_port.as_mut().unwrap().write(buf)?;

        //Each byte takes ~45us to transmit, so we must wait for the transmission to finish before proceeding
        let uart_tx_time = Duration::from_micros( (buf.len() * 45) as u64); 
        thread::sleep(uart_tx_time);
        //Because Tx line is linked with Rx line, all sent msgs are returned on the Rx line and must be cleared from the buffer
        let mut clear_rx_line = BytesMut::new();
        clear_rx_line.resize(send_size, 0);
        let _ = self.swi_port.as_mut().unwrap().read_exact( &mut clear_rx_line );

        Ok(())
    }

    fn recv_swi_buf(&mut self, buf: &mut BytesMut) -> Result {
        let mut encoded_msg = BytesMut::new();
        encoded_msg.resize(ATCA_SWI_CMD_SIZE_MAX as usize,0);
        
        let mut transmit_flag = BytesMut::new();
        transmit_flag.put_u8(0x88);
        let encoded_transmit_flag = self.encode_uart_to_swi(&transmit_flag );
        
        let _ = self.swi_port.as_mut().unwrap().clear(ClearBuffer::All);

        for retry in 0..RECV_RETRIES {
            self.swi_port.as_mut().unwrap().write(&encoded_transmit_flag)?;
            thread::sleep(Duration::from_micros(40_000) );
            let read_response = self.swi_port.as_mut().unwrap().read(&mut encoded_msg);
            
            match read_response {
                Ok(cnt) if cnt == 8 => { //If the buffer is empty except for the transmit flag, wait & try again
                },
                Ok(cnt) if cnt > 16 => {
                    break;
                },
                _ if retry != RECV_RETRIES => continue,
                _  => return Err(Error::Timeout) 
            }
            
            thread::sleep(RECV_RETRY_WAIT);
        }

        //TODO: Fix Sizes. Will do after first testing
        let mut decoded_message = BytesMut::new();
        decoded_message.resize((ATCA_SWI_CMD_SIZE_MAX) as usize, 0);   

        self.decode_swi_to_uart(&encoded_msg, &mut decoded_message);

        let msg_size = decoded_message[1];

        if msg_size as u16 > ATCA_SWI_CMD_SIZE_MAX/8{
            return Err(Error::Timeout)
        }

        buf.resize(msg_size as usize, 0);

        // Remove the transmit flag at the beginning & the excess buffer space at the end
        let _transmit_flag = decoded_message.split_to(1);
        decoded_message.truncate(msg_size as usize);

        buf.copy_from_slice(&decoded_message);

        Ok(())
    }

    fn encode_uart_to_swi(&mut self, uart_msg: &BytesMut ) -> BytesMut {
        
        let mut bit_field = BytesMut::new();
        bit_field.reserve(uart_msg.len() * 8 );
    
        for byte in uart_msg.iter() {
            for bit_index in 0..8 {
                if ( ((1 << bit_index ) & byte) >> bit_index ) == 0 {
                    bit_field.put_u8(0xFD); 
                } else {
                    bit_field.put_u8(0xFF);
                }
            }
        }
        bit_field
    }
    
    fn decode_swi_to_uart(&mut self, swi_msg: &BytesMut, uart_msg: &mut BytesMut ) {
    
        uart_msg.clear();
        assert!( (swi_msg.len() % 8) == 0);
        uart_msg.resize( &swi_msg.len() / 8, 0 );
    
        let mut i = 0; 
        for byte in uart_msg.iter_mut() {
            let bit_slice= &swi_msg[i..i+8];
            
            for bit in bit_slice.iter(){
                if *bit == 0x7F || *bit == 0x7E {
                    *byte ^= 1;
                }
                *byte = byte.rotate_right(1);
            }
            i += 8;
        }
    }

}