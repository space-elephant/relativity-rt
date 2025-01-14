use std::rc::Rc;
use crate::ray::*;
use crate::vec3::*;
use crate::material::*;
use crate::boundingbox::*;
use crate::colour::*;
use enum_dispatch::enum_dispatch;

pub type Range = std::ops::Range<f64>;

pub fn in_range(t: f64, range: Range) -> Option<f64> {
    if range.contains(&t) {
	Some(t)
    } else {
	None
    }
}

pub struct Request {
    pub ray: Ray,
    pub colour: Colour,
    pub depth: u32,
}

#[derive(Clone, Debug)]
pub struct HitRecord {
    pub ray: Ray,
    pub point: Point3,
    pub normal: Vec3,
    pub t: f64,
    pub backface: bool,
    pub material: Rc<dyn Material>,
}

impl HitRecord {
    pub fn with_ray(point: Point3, mut normal: Vec3, t: f64, ray: Ray, material: Rc<dyn Material>) -> Self {
	let backface = ray.direction.dot(normal) > 0.0;
	if backface {
	    normal = -normal
	}
	HitRecord {
	    ray,
	    point,
	    normal,
	    t,
	    backface,
	    material,
	}
    }
}

#[enum_dispatch]
pub trait Object {
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord>;
    fn boundingbox(&self) -> Boundingbox;
}

#[derive(Debug)]
#[enum_dispatch(Object)]
pub enum Primitive {
    Sphere(Sphere),
    SmokeSphere(SmokeSphere),
}

#[derive(Debug)]
pub struct Sphere {
    center: Point3,
    radius: f64,
    material: Rc<dyn Material>,
}

impl Sphere {
    pub fn new(material: Rc<dyn Material>, center: Point3, radius: f64) -> Sphere {
	Sphere {
	    center,
	    radius,
	    material,
	}
    }
}

impl Object for Sphere {
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord> {
	let offset = ray.origin - self.center;
	let a = ray.direction.length_squared();
	let h = ray.direction.dot(offset);
	let c = offset.length_squared() - self.radius*self.radius;
	let discriminant = h*h - a*c;
	if discriminant < 0.0 {
	    None
	} else {
	    // least intersection is the important one
	    let factor = discriminant.sqrt();
	    
	    let t = (-h - factor) / a;
	    if range.contains(&t) {
		return Some(HitRecord {
		    ray,
		    point: ray.at(t),
		    normal: (ray.at(t) - self.center) / self.radius,
		    t,
		    backface: false,
		    material: self.material.clone(),
		})
	    }
	    
	    let t = (-h + factor) / a;
	    if range.contains(&t) {
		return Some(HitRecord {
		    ray,
		    point: ray.at(t),
		    normal: -(ray.at(t) - self.center) / self.radius,
		    t,
		    backface: true,
		    material: self.material.clone(),
		})
	    }
	    None
	}
    }
    
    fn boundingbox(&self) -> Boundingbox {
	let shift = Vec3::new(self.radius, self.radius, self.radius);
	Boundingbox::new(self.center - shift, self.center + shift)
    }
}

#[derive(Debug)]
pub struct Group {
    objects: Vec<Primitive>,
}

impl Group {
    pub fn new(objects: Vec<Primitive>) -> Self {
	Group {
	    objects,
	}
    }
}

impl Object for Group {
    fn hit(&self, ray: Ray, mut range: Range) -> Option<HitRecord> {
	let mut best: Option<HitRecord> = None;
	for object in &self.objects {
	    if let Some(collision) = object.hit(ray, range.clone()) {
		range.end = collision.t;
		best = Some(collision);
	    }
	}
	best
    }
    
    fn boundingbox(&self) -> Boundingbox {
	let mut result = Default::default();
	for object in &self.objects {
	    result += object.boundingbox();
	}
	result
    }
}

#[derive(Debug)]
pub struct SmokeSphere {
    center: Point3,
    radius: f64,
    material: Rc<dyn Material>,
    neg_inv_density: f64,
}

impl SmokeSphere {
    pub fn new(colour: ReflectionSpectrum, density: f64, center: Point3, radius: f64) -> SmokeSphere {
	SmokeSphere {
	    center,
	    radius,
	    material: Rc::new(Isotropic::new(colour)),
	    neg_inv_density: -1.0 / density,
	}
    }
}

impl Object for SmokeSphere {
    fn hit(&self, ray: Ray, range: Range) -> Option<HitRecord> {
	let offset = ray.origin - self.center;
	let a = ray.direction.length_squared();
	let h = ray.direction.dot(offset);
	let c = offset.length_squared() - self.radius*self.radius;
	let discriminant = h*h - a*c;
	if discriminant < 0.0 {
	    None
	} else {
	    // least intersection is the important one
	    let factor = discriminant.sqrt();
	    
	    let mut t1 = (-h - factor) / a;
	    if t1 >= range.end {
		return None;
	    }
	    if t1 < range.start {
		t1 = range.start;
	    }

	    let collision = t1 + self.neg_inv_density * rand::random::<f64>().log10();
	    
	    
	    let t2 = (-h + factor) / a;
	    if collision <= t2 && range.contains(&collision) {
		return Some(HitRecord {
		    ray,
		    point: ray.at(collision),
		    normal: (ray.at(t1) - self.center) / self.radius,
		    t: collision,
		    backface: false,
		    material: self.material.clone(),
		})
	    }
	    None
	}
    }
    
    fn boundingbox(&self) -> Boundingbox {
	let shift = Vec3::new(self.radius, self.radius, self.radius);
	Boundingbox::new(self.center - shift, self.center + shift)
    }
}
