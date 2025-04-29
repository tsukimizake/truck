I tried to debug the boolean operations with coplanar faces for this two month or so. Now I gave up fixing it, but at least I could make or operation of adjacent cubes work.
I'll write up what I did. I hope it helps @ytanimura .

I can use english to some extent, but it's too complex and cumbersome for me to write this report in English. As the author of this project seems to be also Japanese, I would like to write it in Japanese. If there is anybody interested, please use llm to translate it.

最も単純なテストケースとして以下のようなz=1で接し、xy軸をずらすことで他の面は同一平面上にない2つのboxのorで実験しました。

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

この原因はcreate_loops_storeにおいて上のケースでのz=1のような共有面の隣で、相手Shellの辺が交線と見なされてLoopsStoreに追加されてしまうというもののようでした。
追加したadjacent_cubes_or testにおいては以下のような形のLoopsStoreが発生していたので、finalize_adjacent_to_coplanar_facesで`(0.5, 0.5, 1) -> (1, 0.5, 1)` と `(1, 0.5, 1) -> (0.5, 0.5, 1)`のような自明なループを削除するというパッチアップをまずは試しています。

```
LoopsStore[1]:
  Loop[0] with status: Or
  Edges in loop:
  Edge: (0.5, 0.5, 1) -> (1, 0.5, 1)
  Edge: (1, 0.5, 1) -> (1.5, 0.5, 1)
  Edge: (1.5, 0.5, 1) -> (1.5, 0.5, 2)
  Edge: (1.5, 0.5, 2) -> (0.5, 0.5, 2)
  Edge: (0.5, 0.5, 2) -> (0.5, 0.5, 1)
  Loop[1] with status: And
  Edges in loop:
  Edge: (0.5, 0.5, 1) -> (1, 0.5, 1)
  Edge: (1, 0.5, 1) -> (0.5, 0.5, 1)

```

ただ、この対処は全く完全ではなく、例えばcoplanar_faces_or testでは以下のようなLoopsStoreが発生してnon simple wireとなります。
```rust
LoopsStore[4]:
  Loop[0] with status: Or
  Edges in loop:
  Edge: (0, 0.5, 0) -> (0, 0.5, 1)
  Edge: (0, 0.5, 1) -> (0, 0, 1)
  Edge: (0, 0, 1) -> (0, 0, 0)
  Edge: (0, 0, 0) -> (0, 0.5, 0)
  Edge: (0, 0.5, 0) -> (0, 1, 0)
  Edge: (0, 1, 0) -> (0, 1, 1)
  Edge: (0, 1, 1) -> (0, 0.5, 1)
  Edge: (0, 0.5, 1) -> (0, 0.5, 0)
  Edge: (0, 0.5, 0) -> (0, 0, 0)
  Edge: (0, 0, 0) -> (0, 0.5, 0)
```
また、現状のfinalize_adjacent_to_coplanar_facesでは共有面がカーブしている場合にも捕捉できないと思われます。

add_edgeによる面の分割やShapeOpsStatus選択のロジックが読み解ききれず手を付けられていませんが、そのあたりで面上に辺があるケースを弾くのが正道かもしれません。


