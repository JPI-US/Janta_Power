//! SSD1306-compatible 128x64 OLED over I2C.
//!
//! Written for the HS96L03W2C03 module (LCSC C5248080), whose datasheet does not
//! name its controller but is unambiguous about it: 128x64 pixels, slave address
//! `0111100`, a `0x00`/`0x40` control byte before commands and data, and an
//! initialisation sequence that is the standard SSD1306 one.
//!
//! No framebuffer is kept. A full frame is 1 KB, and holding one would put that on
//! the caller's stack for a driver whose only job here is drawing test patterns —
//! so each page is computed as it is sent.
//!
//! **The module's VCC is rated 2.8-3.3 V absolute maximum.** Driving it from 5 V is
//! outside that, and the datasheet warns of permanent damage. The panel may still
//! respond; that is not evidence the supply is within spec.

#![no_std]

use embedded_hal::i2c::I2c;

/// Default 7-bit address. The datasheet offers `0111100` or `0111101`, selected by
/// the D/C# pin — which a four-pin module does not expose, so it is fixed.
pub const DEFAULT_ADDRESS: u8 = 0x3C;

pub const WIDTH: usize = 128;
pub const HEIGHT: usize = 64;

/// Rows are addressed in pages of eight, one bit per pixel within a byte.
pub const PAGES: usize = HEIGHT / 8;

/// Columns a *clear* writes, which is more than the panel shows.
///
/// Controllers in this family ship with wider RAM than the glass — an SH1106 has
/// 132 columns behind a 128-column panel, offset by two — so clearing only the
/// visible 128 can leave a strip at one edge holding whatever was last drawn there.
/// That is exactly how a border pattern survives being cleared.
///
/// Writing past the end is safe on a part that really does have 128: the column
/// pointer wraps within the page and rewrites zeros over zeros. Only correct
/// because clearing is idempotent — a real pattern must never be written this way,
/// or the wrap would overwrite its left edge.
const CLEAR_COLUMNS: usize = 132;

/// Prefix byte marking the rest of a write as commands.
const CONTROL_COMMAND: u8 = 0x00;
/// Prefix byte marking the rest of a write as display data.
const CONTROL_DATA: u8 = 0x40;

/// A test pattern, chosen for what each one can reveal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pattern {
    /// Every pixel lit. Finds dead pixels, and a row or column that never drives.
    AllOn,
    /// Every pixel dark. Finds a pixel stuck on, which `AllOn` cannot show.
    Off,
    /// 8x8 blocks. Finds addressing faults: a swapped page or column shows as a
    /// misaligned grid rather than as nothing at all.
    Checkerboard,
    /// Alternating columns. Isolates a segment driver fault to a column.
    VerticalStripes,
    /// A one-pixel frame at the edges. Confirms the full extent is reachable, which
    /// a pattern covering the middle can pass without proving.
    Border,
}

impl Pattern {
    /// Every pattern, in the order a walkthrough should use them.
    pub const ALL: &'static [(&'static str, Pattern)] = &[
        ("ALL_ON", Pattern::AllOn),
        ("CHECKERBOARD", Pattern::Checkerboard),
        ("STRIPES", Pattern::VerticalStripes),
        ("BORDER", Pattern::Border),
        ("OFF", Pattern::Off),
    ];

    pub fn from_name(name: &str) -> Option<Pattern> {
        Self::ALL
            .iter()
            .find(|(candidate, _)| candidate.eq_ignore_ascii_case(name))
            .map(|(_, pattern)| *pattern)
    }

    pub fn name(self) -> &'static str {
        Self::ALL
            .iter()
            .find(|(_, pattern)| *pattern == self)
            .map(|(name, _)| *name)
            .unwrap_or("UNKNOWN")
    }

    /// The byte for one column of one page: eight vertically stacked pixels, LSB at
    /// the top of the page.
    fn column_byte(self, page: usize, column: usize) -> u8 {
        match self {
            Pattern::AllOn => 0xFF,
            Pattern::Off => 0x00,
            // Eight-pixel blocks, so the parity flips every page and every 8 columns.
            Pattern::Checkerboard => {
                if (page + column / 8) % 2 == 0 {
                    0xFF
                } else {
                    0x00
                }
            }
            Pattern::VerticalStripes => {
                if column % 2 == 0 {
                    0xFF
                } else {
                    0x00
                }
            }
            Pattern::Border => {
                let first_or_last_column = column == 0 || column == WIDTH - 1;
                if first_or_last_column {
                    return 0xFF;
                }
                // Top row lives in bit 0 of page 0; bottom row in bit 7 of the last.
                let mut byte = 0x00;
                if page == 0 {
                    byte |= 0x01;
                }
                if page == PAGES - 1 {
                    byte |= 0x80;
                }
                byte
            }
        }
    }
}

pub struct Ssd1306<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Ssd1306<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C) -> Self {
        Self::with_address(i2c, DEFAULT_ADDRESS)
    }

    pub fn with_address(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Does the panel acknowledge its address?
    ///
    /// Unlike an addressable LED, this part answers — so its presence is genuinely
    /// verifiable, and only "are the right pixels lit" needs a person.
    pub fn present(&mut self) -> bool {
        self.i2c.write(self.address, &[]).is_ok()
    }

    fn command(&mut self, bytes: &[u8]) -> Result<(), I2C::Error> {
        // Small enough to build on the stack; the longest command here is two bytes.
        let mut buffer = [CONTROL_COMMAND; 8];
        let length = bytes.len().min(buffer.len() - 1);
        buffer[1..=length].copy_from_slice(&bytes[..length]);
        self.i2c.write(self.address, &buffer[..=length])
    }

    /// The initialisation sequence from the module's datasheet, in its order.
    ///
    /// Kept verbatim rather than reduced to what looks necessary: the values encode
    /// this panel's multiplex ratio, COM pin wiring and charge-pump arrangement, and
    /// a "tidier" sequence is how a display ends up mirrored or blank on one board
    /// and fine on another.
    ///
    /// Leaves display RAM untouched, so the caller must draw something immediately
    /// after — [`clear`](Self::clear) if nothing else. Clearing here would double
    /// the cost of every command: on a 10 kHz bus a full-screen write is about a
    /// second, and this runs on the thread that also steps the motor.
    pub fn init(&mut self) -> Result<(), I2C::Error> {
        self.command(&[0xAE])?; // display off
        self.command(&[0x00])?; // low column address
        self.command(&[0x10])?; // high column address
        self.command(&[0x40])?; // start line 0
        self.command(&[0x81, 0xCF])?; // contrast
        self.command(&[0xA1])?; // segment remap: 0xA0 mirrors horizontally
        self.command(&[0xC8])?; // COM scan direction: 0xC0 mirrors vertically
        self.command(&[0xA6])?; // normal, not inverted
        self.command(&[0xA8, 0x3F])?; // multiplex ratio, 1/64 duty
        self.command(&[0xD3, 0x00])?; // no display offset
        self.command(&[0xD5, 0x80])?; // clock divide / oscillator
        self.command(&[0xD9, 0xF1])?; // pre-charge period
        self.command(&[0xDA, 0x12])?; // COM pins hardware configuration
        self.command(&[0xDB, 0x30])?; // VCOMH deselect level
        self.command(&[0x20, 0x02])?; // page addressing mode
        // Charge pump on. The panel makes its own display voltage, which is why the
        // module needs no separate high-voltage supply pin.
        self.command(&[0x8D, 0x14])?;
        self.command(&[0xAF]) // display on
    }

    pub fn display_on(&mut self) -> Result<(), I2C::Error> {
        self.command(&[0xAF])
    }

    pub fn display_off(&mut self) -> Result<(), I2C::Error> {
        self.command(&[0xAE])
    }

    /// Blank the panel, including any columns behind the glass.
    ///
    /// Distinct from `write_pattern(Pattern::Off)`, which writes only what is
    /// visible. See [`CLEAR_COLUMNS`].
    pub fn clear(&mut self) -> Result<(), I2C::Error> {
        for page in 0..PAGES {
            self.command(&[0xB0 | page as u8])?;
            self.command(&[0x00])?;
            self.command(&[0x10])?;
            let mut buffer = [0x00u8; CLEAR_COLUMNS + 1];
            buffer[0] = CONTROL_DATA;
            self.i2c.write(self.address, &buffer)?;
        }
        Ok(())
    }

    /// Draw a pattern, one page at a time.
    ///
    /// Per page rather than as one 1 KB transfer: the I2C driver has a finite
    /// transaction buffer, and 129 bytes is comfortably inside any of them.
    pub fn write_pattern(&mut self, pattern: Pattern) -> Result<(), I2C::Error> {
        for page in 0..PAGES {
            self.command(&[0xB0 | page as u8])?; // page start
            self.command(&[0x00])?; // column low nibble
            self.command(&[0x10])?; // column high nibble

            let mut buffer = [CONTROL_DATA; WIDTH + 1];
            for column in 0..WIDTH {
                buffer[column + 1] = pattern.column_byte(page, column);
            }
            self.i2c.write(self.address, &buffer)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pattern_names_round_trip() {
        for (name, pattern) in Pattern::ALL {
            assert_eq!(Pattern::from_name(name), Some(*pattern));
            assert_eq!(pattern.name(), *name);
        }
        assert_eq!(Pattern::from_name("all_on"), Some(Pattern::AllOn));
        assert_eq!(Pattern::from_name("SPIRAL"), None);
    }

    #[test]
    fn all_on_and_off_are_complements() {
        for page in 0..PAGES {
            for column in 0..WIDTH {
                assert_eq!(Pattern::AllOn.column_byte(page, column), 0xFF);
                assert_eq!(Pattern::Off.column_byte(page, column), 0x00);
            }
        }
    }

    #[test]
    fn checkerboard_alternates_every_eight_columns_and_every_page() {
        // A misaligned grid is what an addressing fault looks like, so the geometry
        // is worth pinning rather than eyeballing on hardware.
        assert_eq!(Pattern::Checkerboard.column_byte(0, 0), 0xFF);
        assert_eq!(Pattern::Checkerboard.column_byte(0, 7), 0xFF);
        assert_eq!(Pattern::Checkerboard.column_byte(0, 8), 0x00);
        assert_eq!(Pattern::Checkerboard.column_byte(1, 0), 0x00);
        assert_eq!(Pattern::Checkerboard.column_byte(1, 8), 0xFF);
    }

    #[test]
    fn border_touches_all_four_edges_and_nothing_else() {
        // Left and right columns fully lit.
        assert_eq!(Pattern::Border.column_byte(3, 0), 0xFF);
        assert_eq!(Pattern::Border.column_byte(3, WIDTH - 1), 0xFF);
        // Top row is bit 0 of page 0, bottom row is bit 7 of the last page.
        assert_eq!(Pattern::Border.column_byte(0, 64), 0x01);
        assert_eq!(Pattern::Border.column_byte(PAGES - 1, 64), 0x80);
        // Everything between is dark, or the frame would not be a frame.
        assert_eq!(Pattern::Border.column_byte(3, 64), 0x00);
    }

    #[test]
    fn vertical_stripes_alternate_every_column() {
        assert_eq!(Pattern::VerticalStripes.column_byte(0, 0), 0xFF);
        assert_eq!(Pattern::VerticalStripes.column_byte(0, 1), 0x00);
        assert_eq!(Pattern::VerticalStripes.column_byte(7, 126), 0xFF);
    }
}
