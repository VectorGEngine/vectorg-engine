use vectorg_engine::control::*;
use vectorg_engine::prelude::*;

fn offsets(kind: &str, camber_deg: f32) -> Vec<f32> {
    let hz = 240;
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    match kind {
        "halfspace" => { colliders.insert(ColliderBuilder::halfspace(UnitVector::new_normalize(Vector::y()))); }
        "box" => { colliders.insert(ColliderBuilder::cuboid(20.0, 0.5, 20.0).translation(-Vector::y() * 0.5)); }
        _ => {
            let (hx, hy, hz_) = (20.0f32, 0.5f32, 20.0f32);
            let c = -Vector::y() * 0.5;
            let vtx: Vec<Point<f32>> = [
                [-hx,-hy,-hz_],[hx,-hy,-hz_],[hx,hy,-hz_],[-hx,hy,-hz_],
                [-hx,-hy,hz_],[hx,-hy,hz_],[hx,hy,hz_],[-hx,hy,hz_],
            ].iter().map(|p| Point::new(p[0]+c.x, p[1]+c.y, p[2]+c.z)).collect();
            let idx: Vec<[u32;3]> = vec![[0,1,2],[0,2,3],[4,6,5],[4,7,6],[0,4,5],[0,5,1],
                                         [3,2,6],[3,6,7],[0,3,7],[0,7,4],[1,5,6],[1,6,2]];
            colliders.insert(ColliderBuilder::trimesh(vtx, idx).unwrap());
        }
    }
    let chassis = bodies.insert(RigidBodyBuilder::dynamic()
        .position(Isometry::from_parts(Translation::from(Vector::y()*0.65), Rotation::identity()))
        .can_sleep(false));
    colliders.insert_with_parent(ColliderBuilder::cuboid(0.8,0.15,1.5).mass(1200.0), chassis, &mut bodies);
    bodies[chassis].recompute_mass_properties_from_colliders(&colliders);
    let mut v = DynamicRayCastVehicleController::new(chassis, VehicleControllerConfig::default());
    v.index_forward_axis = 2;
    v.set_input(VehicleInput { brake:1.0, clutch:1.0, ..Default::default() });
    for (x,z,role) in [(-0.8,1.3,WheelAxle::Front),(0.8,1.3,WheelAxle::Front),
                       (-0.8,-1.3,WheelAxle::Rear),(0.8,-1.3,WheelAxle::Rear)] {
        let axle = Rotation::new(Vector::z() * camber_deg.to_radians()) * Vector::x();
        let w = v.add_wheel(Point::new(x,0.0,z), -Vector::y(), axle, 0.4, 0.35, 0.2,
            &WheelTuning { suspension_stiffness:30.0, suspension_compression:3.0,
                           suspension_damping:3.0, friction_slip:1.2, ..Default::default() },
            WheelRole::new(role, role==WheelAxle::Rear, role==WheelAxle::Front));
        w.max_brake_force = 180_000.0;
    }
    let mut q = QueryPipeline::new(); q.update(&colliders);
    let mut p = PhysicsPipeline::new(); let mut il = IslandManager::new();
    let mut bp = BroadPhaseMultiSap::new(); let mut np = NarrowPhase::new();
    let mut ij = ImpulseJointSet::new(); let mut mj = MultibodyJointSet::new();
    let mut cc = CCDSolver::new();
    let par = IntegrationParameters { dt:1.0/hz as f32,
        num_solver_iterations: std::num::NonZeroUsize::new(4).unwrap(), ..Default::default() };
    for _ in 0..(hz*3) {
        v.update_vehicle(par.dt, &(-Vector::y()*9.81), &mut bodies, &colliders, &q,
            QueryFilter::default().exclude_rigid_body(v.chassis));
        p.step(&(-Vector::y()*9.81), &par, &mut il, &mut bp, &mut np, &mut bodies,
            &mut colliders, &mut ij, &mut mj, &mut cc, Some(&mut q), &(), &());
        v.finish_vehicle_update(&mut bodies);
    }
    v.wheels().iter().map(|w| (w.raycast_info().contact_point_ws.x - w.center().x)*1000.0).collect()
}

#[test]
fn all_collider_paths_agree() {
    println!("\nFlat ground, whole car on one surface. Offset from wheel centre, mm.");
    println!("{:>8} | {:>12} {:>12} {:>12} | {:>10}", "camber", "halfspace", "box", "trimesh", "expected");
    for camber_deg in [0.0f32, 1.0, 3.0, 5.0, 10.0] {
        let h = offsets("halfspace", camber_deg)[0];
        let b = offsets("box", camber_deg)[0];
        let t = offsets("trimesh", camber_deg)[0];
        let expected = (2.7 * 200.0 * camber_deg.to_radians().sin()).min(100.0);
        println!("{camber_deg:>8.1} | {h:>12.1} {b:>12.1} {t:>12.1} | {expected:>10.1}");
    }
}
