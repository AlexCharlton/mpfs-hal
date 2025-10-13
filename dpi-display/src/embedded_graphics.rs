use embedded_graphics::pixelcolor::{
    raw::RawU24, Bgr555, Bgr565, Bgr666, Bgr888, BinaryColor, Rgb555, Rgb565, Rgb666, Rgb888,
};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;

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
            if let (x @ 0..super::WIDTH, y @ 0..super::HEIGHT) =
                (coord.x as usize, coord.y as usize)
            {
                // Calculate the index in the framebuffer.
                buffer.set_pixel(x, y, color);
            }
        }

        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let buffer = self.maybe_get_buffer();
        if buffer.is_none() {
            return Err(DrawError::BufferNotAvailable);
        }
        let buffer = buffer.unwrap();

        let self_area = Rectangle::new(
            Point::zero(),
            Size::new(super::WIDTH as u32, super::HEIGHT as u32),
        );
        let target_area = self_area.intersection(area);
        if let Some(bottom_right) = target_area.bottom_right() {
            let mut x = target_area.top_left.x;
            let mut y = target_area.top_left.y;
            for color in colors {
                if x >= bottom_right.x {
                    x = target_area.top_left.x;
                    y += 1;
                } else if y >= bottom_right.y {
                    break;
                }
                buffer.set_pixel(x as usize, y as usize, color);
                x += 1;
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let buffer = self.maybe_get_buffer();
        if buffer.is_none() {
            return Err(DrawError::BufferNotAvailable);
        }
        buffer.unwrap().as_slice().fill(color);
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

// To embedded-graphics Colors
impl From<super::Pixel> for Rgb888 {
    fn from(color: super::Pixel) -> Self {
        Rgb888::new(color.r, color.g, color.b)
    }
}

impl From<super::Pixel> for Rgb666 {
    fn from(color: super::Pixel) -> Self {
        Rgb666::new(color.r, color.g, color.b)
    }
}

impl From<super::Pixel> for Rgb565 {
    fn from(color: super::Pixel) -> Self {
        Rgb565::new(color.r, color.g, color.b)
    }
}

impl From<super::Pixel> for Rgb555 {
    fn from(color: super::Pixel) -> Self {
        Rgb555::new(color.r, color.g, color.b)
    }
}

impl From<super::Pixel> for Bgr888 {
    fn from(color: super::Pixel) -> Self {
        Bgr888::new(color.b, color.g, color.r)
    }
}

impl From<super::Pixel> for Bgr666 {
    fn from(color: super::Pixel) -> Self {
        Bgr666::new(color.b, color.g, color.r)
    }
}

impl From<super::Pixel> for Bgr565 {
    fn from(color: super::Pixel) -> Self {
        Bgr565::new(color.b, color.g, color.r)
    }
}

impl From<super::Pixel> for Bgr555 {
    fn from(color: super::Pixel) -> Self {
        Bgr555::new(color.b, color.g, color.r)
    }
}

// From embedded-graphics Colors
impl From<Rgb888> for super::Pixel {
    fn from(color: Rgb888) -> Self {
        super::Pixel {
            r: color.r(),
            g: color.g(),
            b: color.b(),
        }
    }
}

impl From<Rgb565> for super::Pixel {
    fn from(color: Rgb565) -> Self {
        super::Pixel {
            r: color.r(),
            g: color.g(),
            b: color.b(),
        }
    }
}

impl From<Rgb666> for super::Pixel {
    fn from(color: Rgb666) -> Self {
        super::Pixel {
            r: color.r(),
            g: color.g(),
            b: color.b(),
        }
    }
}

impl From<Rgb555> for super::Pixel {
    fn from(color: Rgb555) -> Self {
        super::Pixel {
            r: color.r(),
            g: color.g(),
            b: color.b(),
        }
    }
}

impl From<Bgr888> for super::Pixel {
    fn from(color: Bgr888) -> Self {
        super::Pixel {
            r: color.r(),
            g: color.g(),
            b: color.b(),
        }
    }
}

impl From<Bgr565> for super::Pixel {
    fn from(color: Bgr565) -> Self {
        super::Pixel {
            r: color.r(),
            g: color.g(),
            b: color.b(),
        }
    }
}

impl From<Bgr666> for super::Pixel {
    fn from(color: Bgr666) -> Self {
        super::Pixel {
            r: color.r(),
            g: color.g(),
            b: color.b(),
        }
    }
}

impl From<Bgr555> for super::Pixel {
    fn from(color: Bgr555) -> Self {
        super::Pixel {
            r: color.r(),
            g: color.g(),
            b: color.b(),
        }
    }
}

impl From<BinaryColor> for super::Pixel {
    fn from(color: BinaryColor) -> Self {
        let value = if color.is_on() { 255 } else { 0 };
        super::Pixel {
            r: value,
            g: value,
            b: value,
        }
    }
}
