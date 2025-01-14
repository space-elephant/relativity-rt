#[deny(unsafe_op_in_unsafe_fn)]

use ndarray::{Array2, Axis};
use std::path::Path;
use std::fs::File;
use std::io::Write;
use std::ffi::OsStr;
use std::cmp;
use rand::Rng;
use std::ops::Add;
use std::rc::Rc;

mod vec3;
use vec3::*;

mod ray;
use ray::*;

mod objects;
use objects::*;

mod material;
use material::*;

mod boundingbox;
use boundingbox::*;

mod referenceframe;
use referenceframe::*;

mod colour;
use colour::*;

const XAXIS: Axis = Axis(1);
const YAXIS: Axis = Axis(0);

struct Image(Array2<Colour3>);

impl Image {
    fn display(&self, filename: &Path) {
	let mut result = vec![b'P', b'6', b'\n'];
	result.extend_from_slice(self.0.len_of(XAXIS).to_string().as_bytes());
	result.push(b' ');
	result.extend_from_slice(self.0.len_of(YAXIS).to_string().as_bytes());
	result.extend_from_slice(b"\n255\n");
	
	for colour in self.0.iter() {
	    let colour = colour.add_gamma();
	    for value in [colour.x, colour.y, colour.z] {
		result.push((value * 255.0) as u8)
	    }
	}
	let mut file = File::create(filename).unwrap();
	file.write_all(&result).unwrap();
    }
}

fn ray_colour(ray: Ray, world: &Bvh, depth: u32) -> Colour {
    let mut result = Colour::new([
	// wavelengths should be ideally randomized
	ColourSample::new(RED, 0.0),
	ColourSample::new(GREEN, 0.0),
	ColourSample::new(BLUE, 0.0),
    ]);
    
    let mut stack = Vec::with_capacity(256);
    stack.push(Request {
	ray,
	colour: Colour::new([
	    ColourSample::new(RED, 1.0),
	    ColourSample::new(GREEN, 1.0),
	    ColourSample::new(BLUE, 1.0),
	]),
	depth,
    });

    while let Some(Request{ray, colour, depth}) = stack.pop() {
	if depth == 0 {
	    // assume it's just black
	} else if let Some(record) = world.hit(ray, 0.001..f64::INFINITY) {
	    if let Some((attenuation, ray)) = record.material.reflect(&record) {
		let colour = attenuation.attenuate(colour);
		//ray_colour(ray, world, depth - 1));
		stack.push(Request{ray, colour, depth: depth - 1});
	    } else {
		return Colour::default();
	    }
	} else {
	    let unit_direction = ray.direction.normalized();
	    let a = 0.5 * (unit_direction.y + 1.0);
	    //let temperature = (1.0-a)*4000.0 + a*10000.0;
	    //let origin = Incandescant::new(temperature, 1.0).attenuate(colour);
	    let origin = Sky::new((1.0-a) * 5300e-9 + a * 400e-9);
	    //println!("{:?}", origin);
	    result.add(&origin.attenuate(colour));
	    //(1.0-a)*Colour3::new(1.0, 1.0, 1.0) + a*Colour3::new(0.5, 0.7, 1.0)
	}
    }
    
    result
}

struct Camera {
    image_width: usize,
    image_height: usize,
    position: Point3,
    direction: Vec3,
    samples_per_pixel: u32,
    max_depth: u32,
    vfov: f64,
}

impl Camera {
    fn new() -> Camera {
	let aspect_ratio: f64 = 16.0 / 9.0;
	let image_width = 640;
	let image_height = ((image_width as f64 / aspect_ratio) as usize).max(1);
	let samples_per_pixel = 64;
	let max_depth = 32;
	let vfov = 36.87_f64.to_radians() * 2.0;

	let position = Point3::new(-0.5, 1.0, 0.5);
	let target = Point3::new(0.0, 0.0, -1.0);
	let direction = position - target;
	
	Camera {
	    image_width,
	    image_height,
	    position,
	    direction,
	    samples_per_pixel,
	    max_depth,
	    vfov,
	}
    }
    
    fn render(&self, world: Bvh) -> Image {
	let focal_length: f64 = self.direction.length();
	let h = (self.vfov / 2.0).tan();
	let viewport_height: f64 = 2.0 * h * focal_length;
	let viewport_width = viewport_height * (self.image_width as f64 / self.image_height as f64);

	let vup = Vec3::new(0.0, 1.0, 0.0);
	let w = self.direction / focal_length;
	
	let u = vup.cross(w).normalized();
	let v = w.cross(u);

	let viewport_u = u * viewport_width;
	let viewport_v = -v * viewport_height;
	let viewport_upper_left = -self.direction - (viewport_u + viewport_v) / 2.0;

	let inverse_width = 1.0 / self.image_width as f64;
	let inverse_height = 1.0 / self.image_height as f64;
	
	Image(Array2::from_shape_fn((self.image_height, self.image_width), |(j, i)| {
	    if i == 0 {
		println!("Remaining scanlines: {}", self.image_height - j);
	    }
	    let mut rng = rand::thread_rng();

	    let mut total = Vec::with_capacity(self.samples_per_pixel as usize);
	    for _ in 0..self.samples_per_pixel {
		let pixel_direction = viewport_upper_left + (i as f64 + rng.gen::<f64>()) * inverse_width * viewport_u + (j as f64 + rng.gen::<f64>()) * inverse_height * viewport_v;
		let ray = Ray::new(self.position, pixel_direction);
		total.push(ray_colour(ray, &world, self.max_depth));
	    }
	    torgb(&total)
	}))
    }
}

fn main() {
    /*let elements = vec![
	Primitive::from(SmokeSphere::new(Colour3::new(0.1, 0.1, 0.1), 1.0, Point3::new(-1.0, 0.0, -1.0), 0.5)),
	Primitive::from(Sphere::new(Rc::new(Lambertian::new(Colour3::new(0.1, 0.5, 0.1))), Point3::new(0.0, 0.0, -2.0), 0.2)),
	Primitive::from(Sphere::new(Rc::new(Metal::new(Colour3::new(0.73, 0.45, 0.2), 0.05)), Point3::new(1.0, 0.0, -1.0), 0.5)),
	Primitive::from(Sphere::new(Rc::new(Dielectric::new(1.5)), Point3::new(0.0, 0.0, -1.0), 0.5)),
	Primitive::from(Sphere::new(Rc::new(Dielectric::new(1.0 / 1.5)), Point3::new(0.0, 0.0, -1.0), 0.45)),
	Primitive::from(Sphere::new(Rc::new(Lambertian::new(Colour3::new(0.5, 0.5, 0.5))), Point3::new(0.0, -100.5, -1.0), 100.0)),
    ];*/

    let elements = vec![
	Primitive::from(SmokeSphere::new(ReflectionSpectrum::Grey(Grey::new(1.0)), 1.0, Point3::new(-1.0, 0.0, -1.0), 0.5)),
	Primitive::from(Sphere::new(Rc::new(Lambertian::new(ReflectionSpectrum::Grey(Grey::new(0.1)))), Point3::new(0.0, 0.0, -2.0), 0.2)),
	Primitive::from(Sphere::new(Rc::new(Metal::new(ReflectionSpectrum::Grey(Grey::new(0.85)), 0.05)), Point3::new(1.0, 0.0, -1.0), 0.5)),
	Primitive::from(Sphere::new(Rc::new(Dielectric::new(1.5)), Point3::new(0.0, 0.0, -1.0), 0.5)),
	Primitive::from(Sphere::new(Rc::new(Dielectric::new(1.0 / 1.5)), Point3::new(0.0, 0.0, -1.0), 0.45)),
	Primitive::from(Sphere::new(Rc::new(Lambertian::new(ReflectionSpectrum::Grey(Grey::new(0.5)))), Point3::new(0.0, -100.5, -1.0), 100.0)),
    ];

    //let world = Box::new(Group::new());
    let world = Bvh::new(elements);
    let camera = Camera::new();
    let image = camera.render(world);
    image.display(Path::new("output.ppm"));
}
