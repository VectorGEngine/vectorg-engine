#![allow(dead_code)]

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
extern crate vectorg_engine_f64 as vectorg_engine;
extern crate vectorg_engine_testbed_3d_f64 as vectorg_engine_testbed_3d;

use std::cmp::Ordering;
use vectorg_engine_testbed_3d::{Testbed, TestbedApp};

mod debug_serialized3;

#[cfg_attr(target_arch = "wasm32", wasm_bindgen(start))]
pub fn main() {
    let mut builders: Vec<(_, fn(&mut Testbed))> =
        vec![("(Debug) serialized", debug_serialized3::init_world)];

    // Lexicographic sort, with stress tests moved at the end of the list.
    builders.sort_by(|a, b| match (a.0.starts_with('('), b.0.starts_with('(')) {
        (true, true) | (false, false) => a.0.cmp(b.0),
        (true, false) => Ordering::Greater,
        (false, true) => Ordering::Less,
    });

    let testbed = TestbedApp::from_builders(builders);
    testbed.run()
}
