use enum_dispatch::enum_dispatch;
use crate::vec3::*;

const KB: f64 = 1.380649e-23;// J/K
const H: f64 = 6.62607015e-34;// J/Hz
const C: f64 = 299792458.0;// m/s

pub const RED: f64 = 640e-9;
pub const GREEN: f64 = 545e-9;
pub const BLUE: f64 = 450e-9;

pub const WARNING: Colour = Colour([
    ColourSample{wavelength: RED, intensity: 1.0},
    ColourSample{wavelength: GREEN, intensity: 0.0},
    ColourSample{wavelength: BLUE, intensity: 1.0},
]);

#[derive(Clone, Copy, Debug)]
pub struct Colour([ColourSample; 3]);

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

pub fn torgb(blocks: &[Colour]) -> Colour3 {
    // results in unnatural camera response curve; can be improved
    let mut result = Colour3::default();
    for block in blocks {
	for sample in block.0 {
	    match sample.wavelength {
		RED => result.x += sample.intensity,
		GREEN => result.y += sample.intensity,
		BLUE => result.z += sample.intensity,
		_ => (),// not in visible range
	    }
	}
    }
    result / (blocks.len() as f64)
}

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

#[enum_dispatch]
pub trait Spectrum {
    fn reflectance(&self, wavelength: f64) -> f64;
    fn attenuate(&self, mut colour: Colour) -> Colour {
	//println!("takes {:?}", colour);
	for sample in &mut colour.0 {
	    sample.intensity *= self.reflectance(sample.wavelength);
	}
	//println!(" to get {:?}", colour);
	colour
    }
}

#[derive(Copy, Clone, Debug)]
#[enum_dispatch(Spectrum)]
pub enum EmmisionSpectrum {
    Incandescant(Incandescant),
    Sky(Sky),
}

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

impl Spectrum for Incandescant {
    fn reflectance(&self, wavelength: f64) -> f64 {
	let f = C / wavelength;
	let exponential = ((H*f)/(KB*self.temperature)).exp() - 1.0;
	self.attenuation * (2.0 * H * f.powi(3)) / (C.powi(2) * exponential)
    }
}

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

impl Spectrum for Sky {
    fn reflectance(&self, wavelength: f64) -> f64 {
	if wavelength > self.wavelength_3db {
	    1.0
	} else {
	    (wavelength / self.wavelength_3db).powi(2)
	}
    }
}

#[derive(Copy, Clone, Debug)]
#[enum_dispatch(Spectrum)]
pub enum ReflectionSpectrum {
    Grey(Grey),
    Copper(Copper),
    Plant(Plant),
}

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
	    // 6dB per octave
	    0.6 / 525e-9 * wavelength
	} else if wavelength < 660.6105e-9 {
	    0.6 / 525e-9_f64.powi(2) * wavelength.powi(2)
	} else {
	    0.9
	}
    }
}

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
