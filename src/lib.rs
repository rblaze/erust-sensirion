#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

pub mod commands;

use core::fmt;

use embedded_hal::i2c::I2c;

const ADDR: u8 = 0x59;

pub struct SGP40<I2C> {
    i2c: I2C,
}

#[derive(Clone, Copy, Hash, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Sgp40Error<I2cError> {
    InvalidResponse,
    InvalidCrc,
    I2c(I2cError),
}

impl<E> From<E> for Sgp40Error<E> {
    fn from(err: E) -> Self {
        Self::I2c(err)
    }
}

impl<E> embedded_hal::i2c::Error for Sgp40Error<E>
where
    E: embedded_hal::i2c::Error,
{
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Self::I2c(err) => err.kind(),
            _ => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<E> fmt::Display for Sgp40Error<E>
where
    E: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidResponse => write!(f, "invalid response"),
            Self::InvalidCrc => write!(f, "invalid CRC"),
            Self::I2c(err) => err.fmt(f),
        }
    }
}

impl<E: fmt::Debug + fmt::Display> core::error::Error for Sgp40Error<E> {}

impl<I2C: I2c> SGP40<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Performs sensor self-test.
    /// Returns true if successful, false if failed.
    pub fn self_test(&mut self) -> Result<bool, Sgp40Error<I2C::Error>> {
        let mut result = [0u8; 3];
        self.i2c
            .write_read(ADDR, &commands::CMD_EXECUTE_SELF_TEST, &mut result)?;

        Self::check_crc(&result)?;

        match result[0] {
            0xd4 => Ok(true),
            0x4b => Ok(false),
            _ => Err(Sgp40Error::InvalidResponse),
        }
    }

    // https://sensirion.com/media/documents/296373BB/6203C5DF/Sensirion_Gas_Sensors_Datasheet_SGP40.pdf
    // Section 4.6
    fn crc(data: &[u8; 2]) -> u8 {
        let mut crc = 0xff;

        for byte in data {
            crc ^= byte;

            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ 0x31;
                } else {
                    crc <<= 1;
                }
            }
        }

        crc
    }

    fn check_crc(data: &[u8; 3]) -> Result<(), Sgp40Error<I2C::Error>> {
        if Self::crc(&[data[0], data[1]]) != data[2] {
            Err(Sgp40Error::InvalidCrc)
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{Error, ErrorType};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum DummyError {}

    impl Error for DummyError {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            unimplemented!()
        }
    }

    struct DummyBus {}

    impl ErrorType for DummyBus {
        type Error = DummyError;
    }

    impl I2c for DummyBus {
        fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            unimplemented!()
        }
    }

    #[test]
    fn test_crc() {
        assert_eq!(SGP40::<DummyBus>::check_crc(&[0xbe, 0xef, 0x92]), Ok(()));
        assert_eq!(
            SGP40::<DummyBus>::check_crc(&[0xbe, 0x01, 0x92]),
            Err(Sgp40Error::InvalidCrc)
        );
    }
}
