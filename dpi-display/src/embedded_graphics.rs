use embedded_graphics::pixelcolor::raw::RawU24;
use embedded_graphics::prelude::*;

pub enum DrawError {
    BufferNotAvailable,
}

impl core::fmt::Display for DrawError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DrawError::BufferNotAvailable => write!(
                f,
                "Display buffer not available. Remember to call display.get_buffer() before drawing."
            ),
        }
    }
}

impl core::fmt::Debug for DrawError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        <Self as core::fmt::Display>::fmt(self, f)
    }
}

impl DrawTarget for super::Display {
    type Color = super::Pixel;
    type Error = DrawError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let buffer = self.maybe_get_buffer();
        if buffer.is_none() {
            return Err(DrawError::BufferNotAvailable);
        }
        let buffer = buffer.unwrap();
        for Pixel(coord, color) in pixels.into_iter() {
            // Check if the pixel coordinates are out of bounds
            if let (x @ 0..=super::WIDTH, y @ 0..=super::HEIGHT) =
                (coord.x as usize, coord.y as usize)
            {
                // Calculate the index in the framebuffer.
                buffer.set_pixel(x, y, color);
            }
        }

        Ok(())
    }
}

impl OriginDimensions for super::Display {
    fn size(&self) -> Size {
        Size::new(super::WIDTH as u32, super::HEIGHT as u32)
    }
}

impl PixelColor for super::Pixel {
    type Raw = RawU24;
}
