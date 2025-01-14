use std::fmt::Debug;
use crate::ray::*;
use crate::vec3::*;
use crate::objects::*;
use crate::colour::*;

pub trait Material: Debug {
    // self will be contained by record
    fn reflect(&self, record: &HitRecord) -> Option<(ReflectionSpectrum, Ray)>;
}

#[derive(Debug)]
pub struct Lambertian {
    albedo: ReflectionSpectrum,
}

impl Lambertian {
    pub fn new(albedo: ReflectionSpectrum) -> Self {
	Lambertian {
	    albedo,
	}
    }
}

impl Material for Lambertian {
    fn reflect(&self, record: &HitRecord) -> Option<(ReflectionSpectrum, Ray)> {
	let mut direction = record.normal + Vec3::random_unit();
	if direction.near_zero() {
	    direction = record.normal;
	}
	Some((self.albedo, Ray::new(record.point, direction)))
    }
}

#[derive(Debug)]
pub struct Metal {
    albedo: ReflectionSpectrum,
    fuzz: f64,
}

impl Metal {
    pub fn new(albedo: ReflectionSpectrum, fuzz: f64) -> Self {
	Metal {
	    albedo,
	    fuzz
	}
    }
}

impl Material for Metal {
    fn reflect(&self, record: &HitRecord) -> Option<(ReflectionSpectrum, Ray)> {
	let reflected = record.ray.direction.reflect(record.normal);
	let direction = reflected.normalized() + self.fuzz * Vec3::random_unit();
	Some((self.albedo, Ray::new(record.point, direction)))
    }
}

#[derive(Debug)]
pub struct Dielectric {
    refractive_index: f64,
}

impl Dielectric {
    pub fn new(refractive_index: f64) -> Self {
	Dielectric {
	    refractive_index,
	}
    }

    fn reflectance(cos: f64, refractive_index: f64) -> f64 {
	let r0 = (1.0 - refractive_index) / (1.0 + refractive_index);
	let r0 = r0*r0;
	r0 + (1.0-r0) * (1.0 - cos).powi(5)
    }
}

impl Material for Dielectric {
    fn reflect(&self, record: &HitRecord) -> Option<(ReflectionSpectrum, Ray)> {
	let rel_refractive_index = if record.backface {self.refractive_index} else {1.0 / self.refractive_index};
	let unit_direction = record.ray.direction.normalized();
	
	if let Some(direction) = unit_direction.refract(record.normal, rel_refractive_index) {
	    let cos = -unit_direction.dot(record.normal);
	    let probability = Self::reflectance(cos, rel_refractive_index);
	    if probability <= rand::random() {
		return Some((ReflectionSpectrum::Grey(Grey::new(1.0)), Ray::new(record.point, direction)));
	    }
	}
	let direction = unit_direction.reflect(record.normal);
	Some((ReflectionSpectrum::Grey(Grey::new(1.0)), Ray::new(record.point, direction)))
    }
}

#[derive(Debug)]
pub struct Isotropic {
    albedo: ReflectionSpectrum,
}

impl Isotropic {
    pub fn new(albedo: ReflectionSpectrum) -> Self {
	Isotropic {
	    albedo,
	}
    }
}

impl Material for Isotropic {
    fn reflect(&self, record: &HitRecord) -> Option<(ReflectionSpectrum, Ray)> {
	let direction = Vec3::random_unit();
	Some((self.albedo, Ray::new(record.point, direction)))
    }
}
