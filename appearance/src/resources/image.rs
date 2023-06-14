use glam::*;

#[derive(Clone, Debug)]
pub struct Image {
    pub data: Vec<u8>,
    pub dimensions: IVec2,
    pub channel_count: i32,
    inv_dimensions: Vec2
}

impl Image {
    pub fn new(data: Vec<u8>, dimensions: &IVec2, channel_count: i32) -> Self {
        Image {
            data,
            dimensions: *dimensions,
            channel_count,
            inv_dimensions: Vec2::new(1.0 / dimensions.x as f32, 1.0 / dimensions.y as f32)
        }
    }

    pub fn sample_pixel(&self, tex_coord: &Vec2) -> Vec4 {
        let (x, y) = (tex_coord.x, tex_coord.y);
        let tl = self.get_pixel(x - self.inv_dimensions.x, y - self.inv_dimensions.y);
        let bl = self.get_pixel(x - self.inv_dimensions.x, y + self.inv_dimensions.y);
        let br = self.get_pixel(x + self.inv_dimensions.x, y + self.inv_dimensions.y);
        let tr = self.get_pixel(x + self.inv_dimensions.x, y - self.inv_dimensions.y);
        
        let x = x * self.dimensions.x as f32;
        let y = y * self.dimensions.y as f32;
        let dx = x - ((x as i32) as f32);
        let dy = y - ((y as i32) as f32);

        let bottom = bl.lerp(br, dx);
        let top = tl.lerp(tr, dx);
        top.lerp(bottom, dy)
    }

    pub fn get_pixel(&self, x: f32, y: f32) -> Vec4 {
        let x = ((x * self.dimensions.x as f32) as usize) % (self.dimensions.x - 1) as usize;
        let y = ((y * self.dimensions.y as f32) as usize) % (self.dimensions.y - 1) as usize;

        match self.channel_count {
            4 => {
                let data: &Vec<(u8, u8, u8, u8)> = unsafe { std::mem::transmute(&self.data) };
                let pixel = &data[y * (self.dimensions.x as usize) + x];
    
                Vec4::new(pixel.0 as f32 / 255.99, pixel.1 as f32 / 255.99, pixel.2 as f32 / 255.99, pixel.3 as f32 / 255.99)
            },
            3 => {
                let data: &Vec<(u8, u8, u8)> = unsafe { std::mem::transmute(&self.data) };
                let pixel = &data[y * (self.dimensions.x as usize) + x];
    
                Vec4::new(pixel.0 as f32 / 255.99, pixel.1 as f32 / 255.99, pixel.2 as f32 / 255.99, 0.0)
            },
            2 => {
                let data: &Vec<(u8, u8)> = unsafe { std::mem::transmute(&self.data) };
                let pixel = &data[y * (self.dimensions.x as usize) + x];
    
                Vec4::new(pixel.0 as f32 / 255.99, pixel.1 as f32 / 255.99, 0.0, 0.0)
            },
            1 => {
                let data: &Vec<u8> = unsafe { std::mem::transmute(&self.data) };
                let pixel = &data[y * (self.dimensions.x as usize) + x];
    
                Vec4::new(*pixel as f32 / 255.99, 0.0, 0.0, 0.0)
            }
            _ => panic!("Failed to get pixel. (Unsupported channel count)")
        }
    }
}