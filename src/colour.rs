use enum_dispatch::enum_dispatch;
use crate::vec3::*;

// incandescance constaints
const KB: f64 = 1.380649e-23;// J/K
const H: f64 = 6.62607015e-34;// J/Hz
const C: f64 = 299792458.0;// m/s

// current camera takes only these wavelengths
pub const RED: f64 = 640e-9;
pub const GREEN: f64 = 545e-9;
pub const BLUE: f64 = 450e-9;

// the colour of one ray, a series of a number of samples (in this case 3)
#[derive(Clone, Copy, Debug)]
pub struct Colour([ColourSample; 3]);

// the empty colour, with no intensity so wavelength doesn't matter
impl Default for Colour {
    fn default() -> Colour {
	Colour([ColourSample{wavelength: 0.0, intensity: 0.0}; 3])
    }
}

impl Colour {
    // uses indices, since src is a transformed version of self
    pub fn add(&mut self, src: &Colour) {
	for (dest, src) in self.0.iter_mut().zip(src.0) {
	    dest.intensity += src.intensity;
	}
    }

    // utility functions to build and edit
    pub fn new(samples: [ColourSample; 3]) -> Colour {
	Colour(samples)
    }

    pub fn mul_wavelength(&mut self, factor: f64) {
	// careful to modify and not copy
	for sample in &mut self.0 {
	    sample.wavelength *= factor;
	}
    }

    pub fn times_intensity(mut self, factor: f64) -> Colour {
	for sample in &mut self.0 {
	    sample.intensity *= factor;
	}
	self
    }

    pub fn empty(mut self) -> Colour {
	for mut dest in self.0 {
	    dest.intensity = 0.0;
	}
	self
    }
}

// generate a Colour3 (a Vec3, as in rtweekend) from a number of samples of colours
pub fn torgb(blocks: &[Colour]) -> Colour3 {
    // results in unnatural camera response curve; can be improved
    let mut result = Colour3::default();
    for block in blocks {
	for sample in block.0 {
	    match sample.wavelength {
		// three frequencies only
		RED => result.x += sample.intensity,
		GREEN => result.y += sample.intensity,
		BLUE => result.z += sample.intensity,
		_ => (),// not in visible range
	    }
	}
    }
    result / (blocks.len() as f64)
}

// pure struct for each sample
#[derive(Clone, Copy, Debug)]
pub struct ColourSample {
    wavelength: f64,// in meters
    intensity: f64,
}

impl ColourSample {
    pub fn new(wavelength: f64, intensity: f64) -> ColourSample {
	ColourSample {
	    wavelength,
	    intensity,
	}
    }
}

// used for both emmision spectra and reflection spectra
// reflection spectra usually have reflectances no more than 1
// but see dielectric refraction
#[enum_dispatch]
pub trait Spectrum {
    fn reflectance(&self, wavelength: f64) -> f64;
    fn attenuate(&self, mut colour: Colour) -> Colour {
	for sample in &mut colour.0 {
	    sample.intensity *= self.reflectance(sample.wavelength);
	}
	colour
    }
}

// having seperate tagged unions allows them to be smaller, since incandescant spectra of reflection makes no sense
#[derive(Copy, Clone, Debug)]
#[enum_dispatch(Spectrum)]
pub enum EmmisionSpectrum {
    Incandescant(Incandescant),
    Sky(Sky),
}

// "warmer" colours have lower temperature when incandescing
#[derive(Copy, Clone, Debug)]
pub struct Incandescant {
    temperature: f64,// in kelvins
    attenuation: f64,
}

impl Incandescant {
    pub fn new(temperature: f64, attenuation: f64) -> Incandescant {
	Incandescant {
	    temperature,
	    attenuation,
	}
    }
}

// all values use SI units (even c, which is 1 elsewhere)
impl Spectrum for Incandescant {
    fn reflectance(&self, wavelength: f64) -> f64 {
	let f = C / wavelength;
	let exponential = ((H*f)/(KB*self.temperature)).exp() - 1.0;
	self.attenuation * (2.0 * H * f.powi(3)) / (C.powi(2) * exponential)
    }
}

// not used in the current version: based on rtweekend
#[derive(Copy, Clone, Debug)]
pub struct Sky {
    wavelength_3db: f64,
}

impl Sky {
    pub fn new(wavelength_3db: f64) -> Sky {
	Sky {
	    wavelength_3db
	}
    }
}

// filter, this one's just meant to look interesting, mainly for debugging
impl Spectrum for Sky {
    fn reflectance(&self, wavelength: f64) -> f64 {
	if wavelength > self.wavelength_3db {
	    1.0
	} else {
	    (wavelength / self.wavelength_3db).powi(2)
	}
    }
}

// most objects are white/grey in the visible range, but absorb signifigant amounts of UV light
// this includes plants, so really only the copper is actually correct
#[derive(Copy, Clone, Debug)]
#[enum_dispatch(Spectrum)]
pub enum ReflectionSpectrum {
    Grey(Grey),
    Copper(Copper),
    Plant(Plant),
}

// simplest possible spectrum, designed as a placeholder
#[derive(Copy, Clone, Debug)]
pub struct Grey {
    brightness: f64,
}

impl Grey {
    pub fn new(brightness: f64) -> Grey {
	Grey {
	    brightness
	}
    }
}

impl Spectrum for Grey {
    fn reflectance(&self, _wavelength: f64) -> f64 {
	self.brightness
    }
}

// a filter, steeper right near the 3dB point
// long wavelengths are less attenuated
#[derive(Copy, Clone, Debug)]
pub struct Copper;

impl Copper {
    pub fn new() -> Copper {
	Copper
    }
}

impl Spectrum for Copper {
    fn reflectance(&self, wavelength: f64) -> f64 {
	if wavelength < 262.5e-9 {
	    0.3
	} else if wavelength < 525e-9 {
	    // 3dB per octave
	    0.6 / 525e-9 * wavelength
	} else if wavelength < 660.6105e-9 {
	    // 6dB per octave
	    0.6 / 525e-9_f64.powi(2) * wavelength.powi(2)
	} else {
	    0.9
	}
    }
}

// based on the reflection spectrum of chlorophyll
// actualy plants will look darker in ultraviolet
// uses a quartic polynomial, generated externally
#[derive(Copy, Clone, Debug)]
pub struct Plant;

impl Plant {
    pub fn new() -> Plant {
	Plant
    }
}

impl Spectrum for Plant {
    fn reflectance(&self, wavelength: f64) -> f64 {
	let wavelength = wavelength * 1e9;// wavelength in nanometers
	let squared = wavelength * wavelength;
	let cubed = squared * wavelength;
	let fourth = squared * squared;
	// was modeled as 0 - 100, make sure to adjust exponents before editting
	(1.0 - (- 4.425573e-9 * fourth
		+ 9.805887e-6 * cubed
		- 0.8033462e-2 * squared
		+ 2.882434 * wavelength
		- 381.5536)).min(0.9)
    }
}
