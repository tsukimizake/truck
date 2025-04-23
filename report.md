I tried to investigate the code with this small testcase. It has a coplanar face on z=1.

```rust
#[test]
fn adjacent_cubes_or() {
    let v = builder::vertex(Point3::origin());
    let e = builder::tsweep(&v, Vector3::unit_x());
    let f = builder::tsweep(&e, Vector3::unit_y());
    let cube: Solid = builder::tsweep(&f, Vector3::unit_z());

    let v = builder::vertex(Point3::new(0.5, 0.5, 1.0));
    let w = builder::tsweep(&v, Vector3::unit_x());
    let f = builder::tsweep(&w, Vector3::unit_y());
    let cube2: Solid = builder::tsweep(&f, Vector3::unit_z());

    let result = crate::or(&cube, &cube2, 0.05);
    match &result {
        Some(_) => eprintln!("Result is Some"),
        None => eprintln!("Result is None"),
    }

    assert!(
        result.is_some(),
        "Boolean OR of adjacent cubes should succeed"
    );
}
```

There seems to be two function to fix.
- create_loops_store
- signed_crossing_faces
