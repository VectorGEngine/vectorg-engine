use vectorg_engine::control::*;
use vectorg_engine::prelude::*;

/// Drive the left wheels up a ramp and watch every wheel's contact point.
/// `trimesh` picks a triangle-mesh ramp instead of a box collider.
fn run(trimesh: bool) {
    let hz = 240;
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    colliders.insert(ColliderBuilder::halfspace(UnitVector::new_normalize(Vector::y())));
    // A low slab on the left side, top at y = 0.06.
    if trimesh {
        let (hx, hy, hz_) = (1.0f32, 0.03f32, 6.0f32);
        let c = Vector::new(-1.2, 0.03, 0.0);
        let v: Vec<Point<f32>> = [
            [-hx, -hy, -hz_], [hx, -hy, -hz_], [hx, hy, -hz_], [-hx, hy, -hz_],
            [-hx, -hy, hz_], [hx, -hy, hz_], [hx, hy, hz_], [-hx, hy, hz_],
        ].iter().map(|p| Point::new(p[0] + c.x, p[1] + c.y, p[2] + c.z)).collect();
        let idx: Vec<[u32; 3]> = vec![
            [0,1,2],[0,2,3],[4,6,5],[4,7,6],[0,4,5],[0,5,1],
            [3,2,6],[3,6,7],[0,3,7],[0,7,4],[1,5,6],[1,6,2],
        ];
        colliders.insert(ColliderBuilder::trimesh(v, idx).unwrap());
    } else {
        colliders.insert(
            ColliderBuilder::cuboid(1.0, 0.03, 6.0).translation(Vector::new(-1.2, 0.03, 0.0)),
        );
    }
    let chassis = bodies.insert(
        RigidBodyBuilder::dynamic()
            .position(Isometry::from_parts(Translation::from(Vector::y() * 0.65), Rotation::identity()))
            .can_sleep(false),
    );
    colliders.insert_with_parent(ColliderBuilder::cuboid(0.8, 0.15, 1.5).mass(1200.0), chassis, &mut bodies);
    bodies[chassis].recompute_mass_properties_from_colliders(&colliders);
    let mut v = DynamicRayCastVehicleController::new(chassis, VehicleControllerConfig::default());
    v.index_forward_axis = 2;
    v.set_input(VehicleInput { brake: 1.0, clutch: 1.0, ..Default::default() });
    for (x, z, role) in [(-0.8, 1.3, WheelAxle::Front), (0.8, 1.3, WheelAxle::Front),
                         (-0.8, -1.3, WheelAxle::Rear), (0.8, -1.3, WheelAxle::Rear)] {
        let w = v.add_wheel(Point::new(x, 0.0, z), -Vector::y(), Vector::x(), 0.4, 0.35, 0.2,
            &WheelTuning { suspension_stiffness: 30.0, suspension_compression: 3.0,
                           suspension_damping: 3.0, friction_slip: 1.2, ..Default::default() },
            WheelRole::new(role, role == WheelAxle::Rear, role == WheelAxle::Front));
        w.max_brake_force = 180_000.0;
    }
    let mut q = QueryPipeline::new(); q.update(&colliders);
    let mut p = PhysicsPipeline::new(); let mut il = IslandManager::new();
    let mut bp = BroadPhaseMultiSap::new(); let mut np = NarrowPhase::new();
    let mut ij = ImpulseJointSet::new(); let mut mj = MultibodyJointSet::new();
    let mut cc = CCDSolver::new();
    let par = IntegrationParameters { dt: 1.0 / hz as f32,
        num_solver_iterations: std::num::NonZeroUsize::new(4).unwrap(), ..Default::default() };
    let mut tick = |bodies: &mut RigidBodySet, colliders: &mut ColliderSet,
                    q: &mut QueryPipeline, v: &mut DynamicRayCastVehicleController| {
        v.update_vehicle(par.dt, &(-Vector::y() * 9.81), bodies, colliders, q,
            QueryFilter::default().exclude_rigid_body(v.chassis));
        p.step(&(-Vector::y() * 9.81), &par, &mut il, &mut bp, &mut np, bodies, colliders,
            &mut ij, &mut mj, &mut cc, Some(q), &(), &());
        v.finish_vehicle_update(bodies, colliders, q,
            QueryFilter::default().exclude_rigid_body(v.chassis));
    };
    for _ in 0..hz { tick(&mut bodies, &mut colliders, &mut q, &mut v); }

    println!("\n--- {} ramp: sliding the car left onto the slab ---",
        if trimesh { "TRIMESH" } else { "BOX" });
    println!("{:>9} {:>8} | {:>38}", "car x", "roll", "contact offset from wheel centre, mm");
    for step in 0..=14 {
        let target = -0.05 * step as f32;
        for _ in 0..(hz / 12) {
            let x = bodies[chassis].translation().x;
            let vy = bodies[chassis].linvel().y;
            bodies[chassis].set_linvel(Vector::new((target - x) * 3.0, vy, 0.0), true);
            tick(&mut bodies, &mut colliders, &mut q, &mut v);
        }
        let up = bodies[chassis].position().rotation * Vector::y();
        let roll = up.x.atan2(up.y).to_degrees();
        let offs: Vec<String> = v.wheels().iter().map(|w| {
            if !w.raycast_info().is_in_contact { return "   air".to_string(); }
            // status 4 = accepted; also show the contact normal's tilt so we can
            // tell a face contact from an edge one.
            let n = w.raycast_info().contact_normal_ws;
            format!("{:>6.1}/n{:>5.2}", (w.raycast_info().contact_point_ws.x - w.center().x) * 1000.0, n.x)
        }).collect();
        println!("{:>9.2} {roll:>7.2}d | FL{} FR{} RL{} RR{}",
            bodies[chassis].translation().x, offs[0], offs[1], offs[2], offs[3]);
    }
}

#[test]
fn ramp_contact_points() { run(false); run(true); }
