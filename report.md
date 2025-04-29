I tried to investigate the boolean operation with coplanar faces.



I can use english to some extent, but it's too complex for me to write this report in English. As the author of this project seems to be also Japanese, I would like to write this report in Japanese. If there is anybody interested, please use chatgpt to translate it.



最も単純なテストケースとして以下のようなz=1で接するboxのorで実験しました。

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

これを実行してまず失敗したのはdivide_facesの
```rust
        let op = pre_faces.iter_mut().find(|face| face[0].poly.include(pt))?;
```
がNoneを返すというものでした。


