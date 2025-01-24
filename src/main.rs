#[deny(unsafe_op_in_unsafe_fn)]

use ndarray::{Array2, Axis};
use std::path::Path;
use std::fs::File;
use std::io::Write;
use rand::Rng;
use std::ops::Add;
use std::thread;
use std::sync::Arc;

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

mod relativity;
use relativity::*;

const XAXIS: Axis = Axis(1);
const YAXIS: Axis = Axis(0);

const TIMESCALE: f64 = 0.2;

// an array of floats representing an image
struct Image(Array2<Colour3>);

impl Image {
    // convert floats to chars and then write them to a file
    fn display(&self, filename: &Path) {
	// make the header
	let mut result = vec![b'P', b'6', b'\n'];
	result.extend_from_slice(self.0.len_of(XAXIS).to_string().as_bytes());
	result.push(b' ');
	result.extend_from_slice(self.0.len_of(YAXIS).to_string().as_bytes());
	result.extend_from_slice(b"\n255\n");

	// convert each pixel to u8 and write out
	for colour in self.0.iter() {
	    let colour = colour.add_gamma();
	    for value in [colour.x, colour.y, colour.z] {
		result.push((value * 255.0) as u8)
	    }
	}
	let mut file = File::create(filename).unwrap();
	file.write_all(&result).unwrap();
    }

    // adjust automatic exposure, ensuring all values are in the range [0, 1]
    fn normalize(&mut self) {
	// Welford's algorithm from
	// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
	let mut count: f64 = 0.0;
	let mut mean: f64 = 0.0;
	let mut m2: f64 = 0.0;

	for colour in self.0.iter() {
	    let brightness = colour.max_component();
	    count += 1.0;
	    let delta = brightness - mean;
	    mean += delta / count;
	    let delta2 = brightness - mean;
	    m2 += delta * delta2;
	}

	let stdev = (m2 / count).sqrt();

	// max is mu + 1.5 sigma
	let adjust = 1.0 / (mean + 1.5 * stdev);

	// adjust, but if any channel goes over 1.0 then reduce all of the values
	for colour in self.0.iter_mut() {
	    *colour *= adjust;
	    if colour.max_component() > 1.0 {
		*colour /= colour.max_component();
	    }
	}
    }
}

// fires a ray through a scene, branching as necessary
// the last entry in maxraystable is all later values; for performance, use 1
fn ray_colour(ray: Ray, colour: Colour, world: &Bvh, maxdepth: u32, maxraystable: &[u32]) -> Colour {
    let mut result = Colour::new([
	// wavelengths should be ideally randomized
	ColourSample::new(RED, 0.0),
	ColourSample::new(GREEN, 0.0),
	ColourSample::new(BLUE, 0.0),
    ]);

    // don't duplicate the entire call stack, just the Request
    let mut stack = Vec::with_capacity(256);
    stack.push(Request {
	ray,
	colour,
	depth: 0,
    });

    while let Some(Request{ray, colour, depth}) = stack.pop() {
	if depth >= maxdepth {
	    // assume it's just black
	} else {
	    if let Some(record) = world.hit(ray, 0.001..f64::INFINITY) {
		// we hit the world, find the reflection rays
		let maxrays = maxraystable[(depth as usize).min(maxraystable.len() - 1)];
		let reflection = record.material.reflect(&record, maxrays);
		let invrays: f64 = 1.0 / reflection.len() as f64;
		for (attenuation, ray) in reflection {
		    // create the attenuation for the ray, remembering that it is going back in time
		    let colour = attenuation.attenuate(colour).times_intensity(invrays);
		    stack.push(Request{ray, colour, depth: depth + 1});
		}
	    } else {
		// hit the sky, where higher is "cooler" (that is to say, bluer, at higher temp.)
		let unit_direction = ray.direction.normalized();
		let a = 0.5 * (unit_direction.y + 1.0);
		let temperature = (1.0-a)*4000.0 + a*10000.0;
		let origin = Incandescant::new(temperature, 1.0);
		result.add(&origin.attenuate(colour));
	    }
	}
    }
    
    result
}

// the thing that generates the image; velocity is strictly less than 1 (c = 1)
struct Camera {
    image_width: usize,
    image_height: usize,
    position: Point3,
    direction: Vec3,
    samples_per_pixel: Arc<[u32]>,
    max_depth: u32,
    vfov: f64,
    velocity: Vec3,
}

impl Camera {
    // create a camera; some values are hardcoded here
    fn new(velocity: Vec3, position: Point3) -> Camera {
	let aspect_ratio: f64 = 16.0 / 9.0;
	let image_width = 640;
	let image_height = ((image_width as f64 / aspect_ratio) as usize).max(1);
	let samples_per_pixel = Arc::new([64, 1]);
	let max_depth = 8;
	let vfov = 36.87_f64.to_radians() * 2.0;

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
	    velocity,
	}
    }

    // motion, time is in arbitrary units
    fn step(&mut self, timestep: f64) {
	self.position += timestep * self.velocity;
    }

    // generate an image using multiple threads
    fn render(&self, world: &Bvh) -> Image {
	let cpus = num_cpus::get();

	// create several threads, each of which creates a number of complete pixels
	// scope needed to allow the threads to copy references from the parent
	let mut pixels = Vec::with_capacity(cpus);
	thread::scope(|s| {
	    let mut threads = Vec::with_capacity(cpus);

	    // move the threadindex into thread scope
	    // other objects have copy semantics, so they aren't moved
	    for threadindex in 0..cpus {
		threads.push(s.spawn(move || {
		    self.render_thread(world, threadindex, cpus)
		}));
	    }

	    // get the data and store on parent
	    for thread in threads {
		pixels.push(thread.join().unwrap());
	    }
	});

	// transpose the pixels so that adjacent pixels are from adjacent threads
	let mut thread = 0;
	let mut index = 0;
	Image(Array2::from_shape_simple_fn((self.image_height, self.image_width), || {
	    let result = pixels[thread][index];
	    thread += 1;
	    if thread >= cpus {
		thread = 0;
		index += 1;
	    }
	    result
	}))
    }

    // runs in the threads, generating pixels with a fixed stride
    fn render_thread(&self, world: &Bvh, threadindex: usize, cpus: usize) -> Vec<Colour3> {
	// generate constants across the frame (which could be done in parent, but it would mean shuffling more data around)
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
	
	let pixels = self.image_width * self.image_height;
	let mut rng = rand::thread_rng();

	// store only our pixels, step but the number of threads being used
	let mut result = Vec::with_capacity(pixels / cpus + 1);

	// store, for each pixel, the set of samples
	let mut total = Vec::with_capacity(self.samples_per_pixel[0] as usize);

	for pixel in (0..pixels).skip(threadindex).step_by(cpus) {
	    let j = pixel / self.image_width;
	    let i = pixel % self.image_width;

	    total.clear();
	    for _ in 0..self.samples_per_pixel[0] {
		// interpolate ray from camera parameters
		let pixel_direction = (viewport_upper_left + (i as f64 + rng.gen::<f64>()) * inverse_width * viewport_u + (j as f64 + rng.gen::<f64>()) * inverse_height * viewport_v).normalized();
		let mut ray = Ray::new(self.position, pixel_direction);
		// uses three wavelengths only, since torgb can't do anything else currently
		let mut colour = Colour::new([
		    ColourSample::new(RED, 1.0),
		    ColourSample::new(GREEN, 1.0),
		    ColourSample::new(BLUE, 1.0),
		]);
		// transforms from self.velocity to "stationary", so velocity is reversed
		lorentz(&mut ray.direction, &mut colour, -self.velocity);
		// get the colour of the new ray
		total.push(ray_colour(ray, colour, &world, self.max_depth, &self.samples_per_pixel[1..]));
	    }
	    result.push(torgb(&total))
	}

	result
    }
}

fn main() {
    // create the scene, ideally would be loaded from a configuration file
    // materials can be used multiple times
    let concrete = Arc::new(Lambertian::new(ReflectionSpectrum::Grey(Grey::new(0.5))));

    let mut elements = vec![
	Primitive::from(Sphere::new(Arc::new(Lambertian::new(ReflectionSpectrum::Grey(Grey::new(0.2)))), Point3::new(0.0, 0.2, -4.0), 0.2)),
	Primitive::from(Sphere::new(Arc::new(Metal::new(ReflectionSpectrum::Copper(Copper::new()), 0.1)), Point3::new(0.0, 1.9, -4.0), 0.5)),
	Primitive::from(Sphere::new(Arc::new(Dielectric::new(1.5)), Point3::new(0.0, 0.9, -4.0), 0.5)),
	Primitive::from(Sphere::new(Arc::new(Dielectric::new(1.0 / 1.5)), Point3::new(0.0, 0.9, -4.0), 0.35)),
	Primitive::from(SmokeSphere::new(ReflectionSpectrum::Grey(Grey::new(0.0)), 0.4, Point3::new(0.0, 0.9, -4.0), 0.35)),

	Primitive::from(PlaneSeg::new(Arc::new(Lambertian::new(ReflectionSpectrum::Grey(Grey::new(0.2)))), Point3::new(-10.0, 0.0, -10.0), Vec3::new(0.0, 0.0, 60.0), Vec3::new(20.0, 0.0, 0.0), PlaneSegType::Parallelogram)),
    ];

    // create a composite structure that can be used multiple times
    let mut building1 = vec![
	// a building
	Primitive::from(PlaneSeg::new(concrete.clone(), Point3::new(-1.0, 0.0, -1.0), Vec3::new(-2.0, 0.0, 0.0), Vec3::new(0.0, 4.0, 0.0), PlaneSegType::Parallelogram)),
	Primitive::from(PlaneSeg::new(concrete.clone(), Point3::new(-1.0, 0.0, -3.0), Vec3::new(-2.0, 0.0, 0.0), Vec3::new(0.0, 4.0, 0.0), PlaneSegType::Parallelogram)),
	Primitive::from(PlaneSeg::new(concrete.clone(), Point3::new(-1.0, 4.0, -1.0), Vec3::new(-2.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -2.0), PlaneSegType::Parallelogram)),
	Primitive::from(PlaneSeg::new(concrete.clone(), Point3::new(-1.0, 0.0, -1.0), Vec3::new(0.0, 0.0, -2.0), Vec3::new(0.0, 4.0, 0.0), PlaneSegType::Parallelogram)),
	Primitive::from(PlaneSeg::new(concrete.clone(), Point3::new(-3.0, 0.0, -1.0), Vec3::new(0.0, 0.0, -2.0), Vec3::new(0.0, 4.0, 0.0), PlaneSegType::Parallelogram)),
    ];
    let mut building2 = building1.clone();
    translate_many(&mut building2, Vec3::new(0.0, 0.0, -4.0));
    let mut building3 = building1.clone();
    translate_many(&mut building3, Vec3::new(0.0, 0.0, -8.0));
    
    let mut building4 = building1.clone();
    translate_many(&mut building4, Vec3::new(4.0, 0.0, 0.0));
    let mut building5 = building1.clone();
    translate_many(&mut building5, Vec3::new(4.0, 0.0, -4.0));
    let mut building6 = building1.clone();
    translate_many(&mut building6, Vec3::new(4.0, 0.0, -8.0));

    elements.append(&mut building1);
    elements.append(&mut building2);
    elements.append(&mut building3);
    
    elements.append(&mut building4);
    elements.append(&mut building5);
    elements.append(&mut building6);

    // add ivy trim
    let mut z = -1.0;
    while z > -11.0 {
	for x in [-0.95, 0.95] {
	    let offset = Vec3::random_unit() * 0.1;
	    let u = Vec3::random_unit();
	    let v = Vec3::random_on_hemisphere(u);
	    elements.push(Primitive::from(PlaneSeg::new(Arc::new(Lambertian::new(ReflectionSpectrum::Plant(Plant::new()))), Point3::new(x, 0.05, z) + offset, u * 0.1, v * 0.1, PlaneSegType::Triangle)));
	}
	z -= 0.05;
    }

    // render a still image, and then a video with a turn in the middle of it
    let world = Bvh::new(elements);

    let mut camera = Camera::new(Vec3::new(0.0, 0.0, 0.0), Point3::new(-0.5, 3.0, 3.0));

    println!("start");
    let mut image = camera.render(&world);
    image.normalize();
    image.display(Path::new("still000.ppm"));
    
    let mut framenum = 0;
    
    camera.velocity = Vec3::new(0.0, 0.0, -0.5);
    while camera.position.z > -3.0 {
	println!("moving {} at {:?}", framenum, camera.position);
	let mut image = camera.render(&world);
	image.normalize();
	image.display(Path::new(&format!("vid001frame{:0>3}.ppm", framenum)));
	camera.step(TIMESCALE);
	framenum += 1;
    }

    println!("{:?}", camera.direction);

    while camera.direction.z > camera.direction.x * 2.0 {
	println!("turning {} at {:?}", framenum, camera.position);
	let mut image = camera.render(&world);
	image.normalize();
	image.display(Path::new(&format!("vid001frame{:0>3}.ppm", framenum)));
	camera.direction.z -= camera.direction.length() * 0.2 * TIMESCALE;
	camera.step(TIMESCALE);
	framenum += 1;
    }
    
    while camera.position.z > -11.0 {
	println!("moving final {} at {:?}", framenum, camera.position);
	let mut image = camera.render(&world);
	image.normalize();
	image.display(Path::new(&format!("vid001frame{:0>3}.ppm", framenum)));
	camera.step(TIMESCALE);
	framenum += 1;
    }

    println!("done");
}
