use ndarray::{Array2, Axis};
use std::path::Path;
use std::fs::File;
use std::io::Write;
use std::ffi::OsStr;
use std::cmp;

mod vec3;
use vec3::*;

mod ray;
use ray::*;

const XAXIS: Axis = Axis(1);
const YAXIS: Axis = Axis(0);

struct Image(Array2<Vec3>);

impl Image {
    fn display(&self, filename: &Path) {
	let mut result = vec![b'P', b'6', b'\n'];
	result.extend_from_slice(self.0.len_of(XAXIS).to_string().as_bytes());
	result.push(b' ');
	result.extend_from_slice(self.0.len_of(YAXIS).to_string().as_bytes());
	result.extend_from_slice(b"\n255\n");
	for colour in self.0.iter() {
	    for value in [colour.x, colour.y, colour.z] {
		result.push((value * 255.0) as u8)
	    }
	}
	let mut file = File::create(filename).unwrap();
	file.write_all(&result);
    }
}

fn hit_sphere(center: Point3, radius: f64, ray: &Ray) -> Option<f64> {
    let offset = center - ray.origin;
    let a = ray.direction.length_squared();
    let b = -2.0 * ray.direction.dot(offset);
    let c = offset.length_squared() - radius*radius;
    let discriminant = b*b - 4.0*a*c;
    if discriminant < 0.0 {
	None
    } else {
	// least intersection is the important one
	Some((-b - discriminant.sqrt()) / (2.0*a))
    }
}

fn ray_colour(ray: &Ray) -> Colour3 {
    if let Some(t) = hit_sphere(Point3::new(0.0, 0.0, -1.0), 0.5, ray) {
	let normal = (ray.at(t) - Point3::new(0.0, 0.0, -1.0)).normalized();
	return 0.5*Colour3::new(normal.x+1.0, normal.y+1.0, normal.z+1.0);
    }
    
    let unit_direction = ray.direction.normalized();
    let a = 0.5 * (unit_direction.y + 1.0);
    (1.0-a)*Colour3::new(1.0, 1.0, 1.0) + a*Colour3::new(0.5, 0.7, 1.0)
}

fn main() {
    let aspect_ratio: f64 = 16.0 / 9.0;
    let image_width = 640;
    let image_height = ((image_width as f64 / aspect_ratio) as usize).max(1);

    let focal_length: f64 = 1.0;
    let viewport_height: f64 = 2.0;
    let viewport_width = viewport_height * (image_width as f64 / image_height as f64);
    let camera: Point3 = Point3::default();

    let viewport_u = Vec3::new(viewport_width, 0.0, 0.0);
    let viewport_v = Vec3::new(0.0, -viewport_height, 0.0);
    let pixel_delta_u = viewport_u / image_width as f64;
    let pixel_delta_v = viewport_v / image_height as f64;
    let viewport_upper_left = Vec3::new(0.0, 0.0, focal_length) - (viewport_u + viewport_v) / 2.0;
    let pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);
    
    let image = Image(Array2::from_shape_fn((image_height, image_width), |(j, i)| {
	if i == 0 {
	    println!("Remaining scanlines: {}", image_height - j);
	}
	let pixel_direction = pixel00_loc + i as f64 * pixel_delta_u + j as f64 * pixel_delta_v;
	let ray = Ray::new(camera, pixel_direction);

	ray_colour(&ray)
    }));
    image.display(Path::new("output.ppm"));
}
