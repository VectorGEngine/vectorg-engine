use vectorg_engine_testbed_3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    crate::dynamic_trimesh3::do_init_world(testbed, true);
}
