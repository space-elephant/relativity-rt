use std::fmt::Debug;
use crate::ray::*;
use crate::vec3::*;
use crate::objects::*;
use crate::colour::*;

pub trait Material: Debug + Send + Sync {
    // self will be contained by record
    // reflectionspectrum does NOT account for attenuation of having multiple rays
    // maxrays >= 1
    fn reflect(&self, record: &HitRecord, maxrays: u32) -> Vec<(ReflectionSpectrum, Ray)>;
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
    fn reflect(&self, record: &HitRecord, maxrays: u32) -> Vec<(ReflectionSpectrum, Ray)> {
	let mut result = Vec::with_capacity(maxrays as usize);
	for _ in 0..maxrays {
	    let mut direction = record.normal + Vec3::random_unit();
	    if direction.near_zero() {
		direction = record.normal;
	    }
	    result.push((self.albedo, Ray::new(record.point, direction)));
	}
	result
    }
}

#[derive(Debug)]
pub struct CleanMetal {
    albedo: ReflectionSpectrum,
}

impl CleanMetal {
    pub fn new(albedo: ReflectionSpectrum) -> Self {
	CleanMetal {
	    albedo,
	}
    }
}

impl Material for CleanMetal {
    fn reflect(&self, record: &HitRecord, _maxrays: u32) -> Vec<(ReflectionSpectrum, Ray)> {
	let reflected = record.ray.direction.reflect(record.normal);
	vec![((self.albedo), Ray::new(record.point, reflected))]
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
	    fuzz,
	}
    }
}

impl Material for Metal {
    fn reflect(&self, record: &HitRecord, maxrays: u32) -> Vec<(ReflectionSpectrum, Ray)> {
	let mut result = Vec::with_capacity(maxrays as usize);
	for _ in 0..maxrays {
	    let reflected = record.ray.direction.reflect(record.normal);
	    let direction = reflected.normalized() + self.fuzz * Vec3::random_unit();
	    result.push((self.albedo, Ray::new(record.point, direction)));
	}
	result
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
    fn reflect(&self, record: &HitRecord, maxrays: u32) -> Vec<(ReflectionSpectrum, Ray)> {
	let rel_refractive_index = if record.backface {self.refractive_index} else {1.0 / self.refractive_index};
	let unit_direction = record.ray.direction.normalized();

	let reflected = unit_direction.reflect(record.normal);
	
	if let Some(refracted) = unit_direction.refract(record.normal, rel_refractive_index) {
	    let cos = -unit_direction.dot(record.normal);
	    let probability = Self::reflectance(cos, rel_refractive_index);
	    
	    if maxrays == 1 {
		if probability <= rand::random() {
		    vec![(ReflectionSpectrum::Grey(Grey::new(1.0)), Ray::new(record.point, refracted))]
		} else {
		    vec![(ReflectionSpectrum::Grey(Grey::new(1.0)), Ray::new(record.point, reflected))]
		}
	    } else {
		// external their average has to equal 1, so multiply by 2
		vec![
		    (ReflectionSpectrum::Grey(Grey::new(2.0 - 2.0 * probability)), Ray::new(record.point, refracted)),
		    (ReflectionSpectrum::Grey(Grey::new(2.0 * probability)), Ray::new(record.point, reflected)),
		]
	    }
	} else {
	    // total internal reflection
	    vec![(ReflectionSpectrum::Grey(Grey::new(1.0)), Ray::new(record.point, reflected))]
	}
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
    fn reflect(&self, record: &HitRecord, maxrays: u32) -> Vec<(ReflectionSpectrum, Ray)> {
	let mut result = Vec::with_capacity(maxrays as usize);
	for _ in 0..maxrays {
	    let direction = Vec3::random_unit();
	    result.push((self.albedo, Ray::new(record.point, direction)));
	}
	result
    }
}
