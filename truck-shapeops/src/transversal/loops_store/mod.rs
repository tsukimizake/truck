#![allow(clippy::many_single_char_names)]

use std::fmt::Debug;

use super::*;
use rustc_hash::FxHashMap as HashMap;
use truck_base::cgmath64::*;
use truck_geometry::prelude::*;
use truck_meshalgo::prelude::*;
use truck_topology::{Vertex, *};

type PolylineCurve = truck_meshalgo::prelude::PolylineCurve<Point3>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapesOpStatus {
    Unknown,
    And,
    Or,
}

impl ShapesOpStatus {
    fn not(self) -> Self {
        match self {
            Self::Unknown => Self::Unknown,
            Self::And => Self::Or,
            Self::Or => Self::And,
        }
    }
}

#[derive(Clone, Debug)]
pub struct BoundaryWire<P, C> {
    wire: Wire<P, C>,
    status: ShapesOpStatus,
}

impl<P, C> BoundaryWire<P, C> {
    #[inline(always)]
    pub fn new(wire: Wire<P, C>, status: ShapesOpStatus) -> Self { Self { wire, status } }
    #[inline(always)]
    pub fn status(&self) -> ShapesOpStatus { self.status }
    #[inline(always)]
    pub fn invert(&mut self) {
        self.wire.invert();
        self.status = self.status.not();
    }
    #[inline(always)]
    pub fn inverse(&self) -> Self {
        Self {
            wire: self.wire.inverse(),
            status: self.status.not(),
        }
    }
}

impl ShapesOpStatus {
    fn from_is_curve<C, S0, S1>(curve: &IntersectionCurve<C, S0, S1>) -> Option<ShapesOpStatus>
    where
        C: ParametricCurve3D + BoundedCurve,
        S0: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3>,
        S1: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3>, {
        let (t0, t1) = curve.range_tuple();
        let t = (t0 + t1) / 2.0;
        let (_, pt0, pt1) = curve.search_triple(t, 100)?;
        let der = curve.leader().der(t);
        let normal0 = curve.surface0().normal(pt0[0], pt0[1]);
        let normal1 = curve.surface1().normal(pt1[0], pt1[1]);
        match normal0.cross(der).dot(normal1) > 0.0 {
            true => Some(ShapesOpStatus::Or),
            false => Some(ShapesOpStatus::And),
        }
    }
}

impl<P, C> std::ops::Deref for BoundaryWire<P, C> {
    type Target = Wire<P, C>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target { &self.wire }
}

impl<P, C> std::ops::DerefMut for BoundaryWire<P, C> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.wire }
}

#[derive(Clone, Debug)]
pub struct Loops<P, C>(Vec<BoundaryWire<P, C>>);
#[derive(Clone, Debug)]
pub struct LoopsStore<P, C>(Vec<Loops<P, C>>);

impl<P, C> std::ops::Deref for Loops<P, C> {
    type Target = Vec<BoundaryWire<P, C>>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target { &self.0 }
}

impl<P, C> std::ops::DerefMut for Loops<P, C> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.0 }
}

impl<P, C> std::ops::Deref for LoopsStore<P, C> {
    type Target = Vec<Loops<P, C>>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target { &self.0 }
}

impl<P, C> std::ops::DerefMut for LoopsStore<P, C> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.0 }
}

impl<P, C> FromIterator<BoundaryWire<P, C>> for Loops<P, C> {
    #[inline(always)]
    fn from_iter<I: IntoIterator<Item = BoundaryWire<P, C>>>(iter: I) -> Self {
        Self(Vec::from_iter(iter))
    }
}

impl<'a, P, C, S> From<&'a Face<P, C, S>> for Loops<P, C> {
    #[inline(always)]
    fn from(face: &'a Face<P, C, S>) -> Loops<P, C> {
        face.absolute_boundaries()
            .iter()
            .map(|wire| BoundaryWire::new(wire.clone(), ShapesOpStatus::Unknown))
            .collect()
    }
}

impl<'a, P: 'a, C: 'a, S: 'a> FromIterator<&'a Face<P, C, S>> for LoopsStore<P, C> {
    fn from_iter<I: IntoIterator<Item = &'a Face<P, C, S>>>(iter: I) -> Self {
        Self(iter.into_iter().map(Loops::from).collect())
    }
}

impl<'a, P, C> IntoIterator for &'a LoopsStore<P, C> {
    type Item = <&'a Vec<Loops<P, C>> as IntoIterator>::Item;
    type IntoIter = <&'a Vec<Loops<P, C>> as IntoIterator>::IntoIter;
    fn into_iter(self) -> Self::IntoIter { self.0.iter() }
}

#[derive(Clone, Debug, Copy, PartialEq)]
enum ParameterKind {
    Front,
    Back,
    Inner(f64),
}

impl ParameterKind {
    fn try_new(t: f64, (t0, t1): (f64, f64)) -> Option<ParameterKind> {
        if t0.near(&t) {
            Some(ParameterKind::Front)
        } else if t1.near(&t) {
            Some(ParameterKind::Back)
        } else if t0 < t && t < t1 {
            Some(ParameterKind::Inner(t))
        } else {
            None
        }
    }
}

impl<P: Copy, C: Clone> Loops<P, C> {
    fn search_parameter(&self, pt: P) -> Option<(usize, usize, ParameterKind)>
    where C: BoundedCurve<Point = P> + SearchParameter<D1, Point = P> {
        self.iter()
            .enumerate()
            .flat_map(move |(i, wire)| wire.iter().enumerate().map(move |(j, edge)| (i, j, edge)))
            .find_map(|(i, j, edge)| {
                let curve = edge.curve();
                curve.search_parameter(pt, None, 1).and_then(|t| {
                    let kind = ParameterKind::try_new(t, curve.range_tuple())?;
                    Some((i, j, kind))
                })
            })
    }

    fn change_vertex(
        &mut self,
        old_vertex: &Vertex<P>,
        new_vertex: &Vertex<P>,
        emap: &mut HashMap<EdgeID<C>, Edge<P, C>>,
    ) {
        self.iter_mut()
            .flat_map(|wire| wire.iter_mut())
            .for_each(|edge| {
                let mut new_edge = if edge.absolute_front() == old_vertex {
                    emap.entry(edge.id()).or_insert_with(|| {
                        Edge::new(new_vertex, edge.absolute_back(), edge.curve())
                    })
                } else if edge.absolute_back() == old_vertex {
                    emap.entry(edge.id()).or_insert_with(|| {
                        Edge::new(edge.absolute_front(), new_vertex, edge.curve())
                    })
                } else {
                    return;
                }
                .clone();
                if !edge.orientation() {
                    new_edge.invert();
                }
                // Remove the edge from the HashMap when it is no longer there because ID reassignment will occur.
                if edge.count() == 1 {
                    emap.remove(&edge.id());
                }
                *edge = new_edge;
            })
    }

    fn swap_edge_into_wire(&mut self, edge_id: EdgeID<C>, new_wire: &Wire<P, C>) {
        self.iter_mut().for_each(|wire| {
            let mut iter = wire.iter().enumerate();
            if let Some((idx, edge)) = iter.find(|(_, edge)| edge.id() == edge_id) {
                if edge.orientation() {
                    wire.swap_edge_into_wire(idx, new_wire.clone());
                } else {
                    wire.swap_edge_into_wire(idx, new_wire.inverse());
                }
            }
        });
    }

    #[inline(always)]
    fn add_independent_loop(&mut self, r#loop: BoundaryWire<P, C>) {
        self.push(r#loop.inverse());
        self.push(r#loop);
    }

    fn add_edge(
        &mut self,
        edge0: Edge<P, C>,
        status: ShapesOpStatus,
    ) -> [Option<(usize, usize)>; 2] {
        let a = self.iter().enumerate().find_map(|(i, wire)| {
            wire.iter().enumerate().find_map(|(j, edge)| {
                if edge.front() == edge0.back() {
                    Some((i, j))
                } else {
                    None
                }
            })
        });
        let b = self.iter().enumerate().find_map(|(i, wire)| {
            wire.iter().enumerate().find_map(|(j, edge)| {
                if edge.front() == edge0.front() {
                    Some((i, j))
                } else {
                    None
                }
            })
        });
        if let Some((wire_index0, edge_index0)) = a {
            self[wire_index0].rotate_left(edge_index0);
            self[wire_index0].push_front(edge0.clone());
            self[wire_index0].push_back(edge0.inverse());
        }
        match (a, b) {
            (Some((wire_index0, edge_index0)), Some((wire_index1, edge_index1))) => {
                if wire_index0 == wire_index1 {
                    let len = self[wire_index0].len() - 2;
                    let edge_index1 = (len + edge_index1 - edge_index0) % len + 1;
                    let new_wire = self[wire_index0].split_off(edge_index1);
                    self[wire_index0].status = status;
                    self.push(BoundaryWire::new(new_wire, status.not()));
                } else {
                    let mut new_wire0 = self[wire_index1].clone();
                    let mut new_wire1 = new_wire0.split_off(edge_index1);
                    new_wire0.append(&mut self[wire_index0]);
                    new_wire0.append(&mut new_wire1);
                    self[wire_index0] = new_wire0;
                    self.swap_remove(wire_index1);
                }
            }
            (None, Some((wire_index1, edge_index1))) => {
                self[wire_index1].rotate_left(edge_index1);
                self[wire_index1].push_front(edge0.inverse());
                self[wire_index1].push_back(edge0);
            }
            (None, None) => self.push(BoundaryWire::new(
                vec![edge0.inverse(), edge0].into(),
                ShapesOpStatus::Unknown,
            )),
            _ => {}
        }
        [a, b]
    }
}

impl<P: Copy + Tolerance, C: Clone> LoopsStore<P, C> {
    #[inline(always)]
    fn change_vertex(
        &mut self,
        old_vertex: &Vertex<P>,
        new_vertex: &Vertex<P>,
        emap: &mut HashMap<EdgeID<C>, Edge<P, C>>,
    ) {
        self.iter_mut()
            .for_each(|loops| loops.change_vertex(old_vertex, new_vertex, emap));
    }

    #[inline(always)]
    fn swap_edge_into_wire(&mut self, edge_id: EdgeID<C>, new_wire: &Wire<P, C>) {
        self.iter_mut()
            .for_each(|loops| loops.swap_edge_into_wire(edge_id, new_wire))
    }

    fn add_polygon_vertex(
        &mut self,
        loops_index: usize,
        v: &Vertex<P>,
        emap: &mut HashMap<EdgeID<C>, Edge<P, C>>,
    ) -> Option<(usize, usize, ParameterKind)>
    where
        C: Cut<Point = P> + SearchParameter<D1, Point = P>,
    {
        let pt = v.point();
        let (wire_index, edge_index, kind) = self[loops_index].search_parameter(pt)?;
        match kind {
            ParameterKind::Front => {
                let old_vertex = self[loops_index][wire_index][edge_index]
                    .absolute_front()
                    .clone();
                self.change_vertex(&old_vertex, v, emap);
            }
            ParameterKind::Back => {
                let old_vertex = self[loops_index][wire_index][edge_index]
                    .absolute_back()
                    .clone();
                self.change_vertex(&old_vertex, v, emap);
            }
            ParameterKind::Inner(t) => {
                let edge = self[loops_index][wire_index][edge_index].absolute_clone();
                let edge_id = edge.id();
                let (edge0, edge1) = edge.cut_with_parameter(v, t)?;
                let new_wire: Wire<_, _> = vec![edge0, edge1].into();
                self.swap_edge_into_wire(edge_id, &new_wire);
            }
        }
        Some((wire_index, edge_index, kind))
    }
}

impl<C> LoopsStore<Point3, C> {
    fn add_geom_vertex<S>(
        &mut self,
        (loops_index, wire_index, edge_index): (usize, usize, usize),
        v: &Vertex<Point3>,
        kind: ParameterKind,
        another_surface: &S,
        emap: &mut HashMap<EdgeID<C>, Edge<Point3, C>>,
    ) -> Option<()>
    where
        C: Cut<Point = Point3, Vector = Vector3> + SearchNearestParameter<D1, Point = Point3>,
        S: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3>,
    {
        match kind {
            ParameterKind::Front => {
                let old_vertex = self[loops_index][wire_index][edge_index]
                    .absolute_front()
                    .clone();
                v.set_point(old_vertex.point());
                self.change_vertex(&old_vertex, v, emap);
            }
            ParameterKind::Back => {
                let old_vertex = self[loops_index][wire_index][edge_index]
                    .absolute_back()
                    .clone();
                v.set_point(old_vertex.point());
                self.change_vertex(&old_vertex, v, emap);
            }
            ParameterKind::Inner(_) => {
                let curve = self[loops_index][wire_index][edge_index].curve();
                let (pt, t, _) =
                    curve_surface_projection(&curve, None, another_surface, None, v.point(), 100)?;
                v.set_point(pt);
                let edge = self[loops_index][wire_index][edge_index].absolute_clone();
                let edge_id = edge.id();
                let (edge0, edge1) = edge.cut_with_parameter(v, t)?;
                let new_wire: Wire<_, _> = vec![edge0, edge1].into();
                self.swap_edge_into_wire(edge_id, &new_wire);
            }
        }
        Some(())
    }
}

fn curve_surface_projection<C, S>(
    curve: &C,
    curve_hint: Option<f64>,
    surface: &S,
    surface_hint: Option<(f64, f64)>,
    point: Point3,
    trials: usize,
) -> Option<(Point3, f64, Point2)>
where
    C: ParametricCurve3D + SearchNearestParameter<D1, Point = Point3>,
    S: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3>,
{
    if trials == 0 {
        return None;
    }
    let t = curve.search_nearest_parameter(point, curve_hint, 10)?;
    let pt0 = curve.subs(t);
    let (u, v) = surface.search_nearest_parameter(point, surface_hint, 10)?;
    let pt1 = surface.subs(u, v);
    if point.near(&pt0) && point.near(&pt1) && pt0.near(&pt1) {
        Some((point, t, Point2::new(u, v)))
    } else {
        let l = curve.der(t);
        let n = surface.normal(u, v);
        let t0 = (pt1 - pt0).dot(n) / l.dot(n);
        curve_surface_projection(
            curve,
            Some(t),
            surface,
            Some((u, v)),
            pt0 + t0 * l,
            trials - 1,
        )
    }
}

fn create_independent_loop<P, C, D>(mut poly_curve0: C) -> Wire<P, D>
where
    C: Cut<Point = P>,
    D: From<C>, {
    let (t0, t1) = poly_curve0.range_tuple();
    let t = (t0 + t1) / 2.0;
    let poly_curve1 = poly_curve0.cut(t);
    let v0 = Vertex::new(poly_curve0.front());
    let v1 = Vertex::new(poly_curve1.front());
    let edge0 = Edge::new(&v0, &v1, poly_curve0.into());
    let edge1 = Edge::new(&v1, &v0, poly_curve1.into());
    wire![edge0, edge1]
}

#[allow(dead_code)]
pub struct LoopsStoreQuadruple<C> {
    pub geom_loops_store0: LoopsStore<Point3, C>,
    pub poly_loops_store0: LoopsStore<Point3, PolylineCurve>,
    pub geom_loops_store1: LoopsStore<Point3, C>,
    pub poly_loops_store1: LoopsStore<Point3, PolylineCurve>,
}

/// Check if two surfaces are coplanar within tolerance
/// TODO may not complete!
fn are_surfaces_coplanar<S>(surface0: &S, surface1: &S, tol: f64) -> bool
where S: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> {
    // Sample points on the parameter domains of both surfaces
    let sample_points = [(0.25, 0.25), (0.5, 0.5), (0.75, 0.75)];

    // Check if normals are parallel (or anti-parallel) at sample points
    for &(u, v) in &sample_points {
        let normal0 = surface0.normal(u, v);
        let normal1 = surface1.normal(u, v);

        // Normalize the normals
        let normal0 = normal0 / normal0.magnitude();
        let normal1 = normal1 / normal1.magnitude();

        // Check if normals are parallel or anti-parallel
        let dot_product = normal0.dot(normal1).abs();
        if (dot_product - 1.0).abs() > tol {
            return false;
        }

        // Sample a point on each surface
        let point0 = surface0.subs(u, v);
        let point1 = surface1.subs(u, v);

        // Project point1 onto the plane defined by point0 and normal0
        let displacement = point1 - point0;
        let distance = displacement.dot(normal0).abs();

        // Check if the distance is within tolerance
        if distance > tol {
            return false;
        }
    }

    true
}

pub fn print_edge<C>(edge: &Edge<Point3, C>) {
    let front_point = edge.absolute_front().point();
    let back_point = edge.absolute_back().point();
    if edge.orientation() {
        println!(
            "  Edge: ({}, {}, {}) -> ({}, {}, {})",
            front_point.x, front_point.y, front_point.z, back_point.x, back_point.y, back_point.z,
        );
    } else {
        println!(
            "  Edge: ({}, {}, {}) -> ({}, {}, {})",
            back_point.x, back_point.y, back_point.z, front_point.x, front_point.y, front_point.z
        );
    }
}

pub fn print_wire<C>(wire: &Wire<Point3, C>) {
    println!(
        "Wire with {} edges simple: {}:",
        wire.len(),
        wire.is_simple()
    );
    for edge in wire.iter() {
        print_edge(edge);
    }
}

pub fn print_face<C, S>(face: &Face<Point3, C, S>) {
    for wire in face.boundaries() {
        print_wire(&wire);
    }
}

pub fn print_loops<C>(loops: &Loops<Point3, C>) {
    for (i, loop_) in loops.iter().enumerate() {
        println!("Loop[{}] with status: {:?}", i, loop_.status());
        println!("Edges in loop:");
        for edge in loop_.edge_iter() {
            print_edge(edge);
        }
    }
}
pub fn print_shell<C, S>(shell: &Shell<Point3, C, S>) {
    for (i, face) in shell.face_iter().enumerate() {
        println!("Face[{}]:", i);
        print_face(face);
    }
}

pub fn print_loops_store<C>(loops_store: &LoopsStore<Point3, C>)
where C: ParametricCurve3D + Debug {
    for (i, loops) in loops_store.iter().enumerate() {
        println!("LoopsStore[{}]:", i);
        for (j, loop_) in loops.iter().enumerate() {
            println!("  Loop[{}] with status: {:?}", j, loop_.status());
            println!("  Edges in loop:");
            for edge in loop_.edge_iter() {
                print_edge(edge);
            }
        }
    }
}

fn finalize_coplanar_faces<C, S>(
    geom_loops_store0: &mut LoopsStore<Point3, C>,
    geom_loops_store1: &mut LoopsStore<Point3, C>,
    ori0: bool,
    ori1: bool,
    face_index0: usize,
    face_index1: usize,
) -> ()
where
    S: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + Debug,
    C: Debug + Clone,
{
    println!("i: {}, j: {}", face_index0, face_index1);
    if ori0 == ori1 {
        // orientation が同じなら共通部分(And) を片方だけ残す
        let and_loops_0: Vec<_> = geom_loops_store0[face_index0]
            .iter()
            .filter(|l| l.status() == ShapesOpStatus::And)
            .cloned()
            .collect();

        let mut new_loops1 = Vec::new();
        for loop1 in geom_loops_store1[face_index1].iter().cloned() {
            let duplicated = and_loops_0.iter().any(|loop0| {
                loop0.len() == loop1.len()
                    && loop0
                        .iter()
                        .zip(loop1.iter())
                        .all(|(e0, e1)| e0.id() == e1.id())
            });
            if !duplicated {
                new_loops1.push(loop1);
            }
        }

        geom_loops_store1[face_index1].clear();
        geom_loops_store1[face_index1].extend(new_loops1);
    } else {
        // orientationが逆なら共通部分は内部なのでaddしない
        let mut removed_indexes0 = vec![];
        geom_loops_store0[face_index0]
            .iter()
            .enumerate()
            .for_each(|(idx, loop_)| {
                if loop_.status() == ShapesOpStatus::And {
                    removed_indexes0.push(idx);
                }
            });
        removed_indexes0.iter().for_each(|&idx| {
            geom_loops_store0[face_index0].swap_remove(idx);
        });

        let mut removed_indexes1 = vec![];
        geom_loops_store1[face_index1]
            .iter()
            .enumerate()
            .for_each(|(idx, loop_)| {
                if loop_.status() == ShapesOpStatus::And {
                    removed_indexes1.push(idx);
                }
            });
        removed_indexes1.iter().for_each(|&idx| {
            geom_loops_store1[face_index1].swap_remove(idx);
        });
    }
}

fn finalize_adjacent_to_coplanar_faces<C, S>(
    geom_loops_store: &mut LoopsStore<Point3, C>,
    face_index: usize,
) -> ()
where
    S: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + Debug,
    C: Debug,
{
    // Z=1面のcoplanar面の辺が(0.5, 1, 1) と (1, 1, 1) 間にあると隣のY=1面ではこのようになり、
    // divide_facesで失敗する
    // Loop[0] with status: And
    //   Edges in loop:
    //   Edge: (0.5, 1, 1) -> (1, 1, 1)
    //   Edge: (1, 1, 1) -> (0.5, 1, 1)
    // Loop[1] with status: Or
    //   Edges in loop:
    //   Edge: (0.5, 1, 1) -> (0, 1, 1)
    //   Edge: (0, 1, 1) -> (0, 1, 0)
    //   Edge: (0, 1, 0) -> (1, 1, 0)
    //   Edge: (1, 1, 0) -> (1, 1, 1)
    //   Edge: (1, 1, 1) -> (0.5, 1, 1)

    // 長さ2のAndループを削除して、
    // 残りが複数なら何もしない
    let mut removed_indexes = vec![];
    geom_loops_store[face_index]
        .iter()
        .enumerate()
        .for_each(|(idx, loop_)| {
            // TODO 曲面の場合loop_.len() == 2とは限らない
            // loop_の面積が0の場合削除のようにするのが正しいか？
            if loop_.status() == ShapesOpStatus::And && loop_.len() == 2 {
                removed_indexes.push(idx);
            }
        });
    removed_indexes.iter().for_each(|&idx| {
        geom_loops_store[face_index].swap_remove(idx);
    });
}

pub fn create_loops_stores<C, S>(
    geom_shell0: &Shell<Point3, C, S>,
    poly_shell0: &Shell<Point3, PolylineCurve, Option<PolygonMesh>>,
    geom_shell1: &Shell<Point3, C, S>,
    poly_shell1: &Shell<Point3, PolylineCurve, Option<PolygonMesh>>,
    tol: f64,
) -> Option<LoopsStoreQuadruple<C>>
where
    C: SearchNearestParameter<D1, Point = Point3>
        + SearchParameter<D1, Point = Point3>
        + Cut<Point = Point3, Vector = Vector3>
        + From<IntersectionCurve<PolylineCurve, S, S>>
        + Clone
        + Debug,
    S: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + Clone + Debug,
{
    let mut geom_loops_store0: LoopsStore<_, _> = geom_shell0.face_iter().collect();
    let mut poly_loops_store0: LoopsStore<_, _> = poly_shell0.face_iter().collect();
    let mut geom_loops_store1: LoopsStore<_, _> = geom_shell1.face_iter().collect();
    let mut poly_loops_store1: LoopsStore<_, _> = poly_shell1.face_iter().collect();
    let store0_len = geom_loops_store0.len();
    let store1_len = geom_loops_store1.len();

    // Track coplanar face pairs to avoid processing them in the main loop
    let mut coplanar_faces_index = Vec::new();

    {
        for face_index0 in 0..store0_len {
            for face_index1 in 0..store1_len {
                let surface0 = geom_shell0[face_index0].surface();
                let surface1 = geom_shell1[face_index1].surface();

                if are_surfaces_coplanar(&surface0, &surface1, tol) {
                    coplanar_faces_index.push((face_index0, face_index1));
                }
            }
        }
    }

    let mut adjacent_to_coplanar_faces_index_0 = Vec::new();
    let mut adjacent_to_coplanar_faces_index_1 = Vec::new();

    println!("Coplanar faces:{:?}", coplanar_faces_index);

    // Main processing for non-coplanar faces
    (0..store0_len)
        .flat_map(move |i| (0..store1_len).map(move |j| (i, j)))
        .try_for_each(|(face_index0, face_index1)| {
            let ori0 = geom_shell0[face_index0].orientation();
            let ori1 = geom_shell1[face_index1].orientation();
            let surface0 = geom_shell0[face_index0].surface();
            let surface1 = geom_shell1[face_index1].surface();
            let polygon0 = poly_shell0[face_index0].surface()?;
            let polygon1 = poly_shell1[face_index1].surface()?;
            intersection_curve::intersection_curves(
                surface0.clone(),
                &polygon0,
                surface1.clone(),
                &polygon1,
            )?
            .into_iter()
            .try_for_each(|(polyline, intersection_curve)| {
                let mut intersection_curve = intersection_curve.into();
                let status = ShapesOpStatus::from_is_curve(&intersection_curve)?;
                let (status0, status1) = match (ori0, ori1) {
                    (true, true) => (status, status.not()),
                    (true, false) => (status.not(), status.not()),
                    (false, true) => (status, status),
                    (false, false) => (status.not(), status),
                };
                if polyline.front().near(&polyline.back()) {
                    let poly_wire = create_independent_loop(polyline);
                    poly_loops_store0[face_index0]
                        .add_independent_loop(BoundaryWire::new(poly_wire.clone(), status0));
                    poly_loops_store1[face_index1]
                        .add_independent_loop(BoundaryWire::new(poly_wire, status1));
                    let geom_wire = create_independent_loop(intersection_curve);
                    geom_loops_store0[face_index0]
                        .add_independent_loop(BoundaryWire::new(geom_wire.clone(), status0));
                    geom_loops_store1[face_index1]
                        .add_independent_loop(BoundaryWire::new(geom_wire, status1));
                } else {
                    let pv0 = Vertex::new(polyline.front());
                    let pv1 = Vertex::new(polyline.back());
                    let gv0 = Vertex::new(polyline.front());
                    let gv1 = Vertex::new(polyline.back());
                    let mut pemap0 = HashMap::default();
                    let mut pemap1 = HashMap::default();
                    let mut gemap0 = HashMap::default();
                    let mut gemap1 = HashMap::default();
                    let (first_is_coplanar, second_is_coplanar) = (
                        first_is_coplanar_face(face_index0, face_index1, &coplanar_faces_index),
                        second_is_coplanar_face(face_index0, face_index1, &coplanar_faces_index),
                    );

                    if first_is_coplanar {
                        adjacent_to_coplanar_faces_index_1.push(face_index1);
                    }
                    if second_is_coplanar {
                        adjacent_to_coplanar_faces_index_0.push(face_index0);
                    }

                    let idx00 =
                        poly_loops_store0.add_polygon_vertex(face_index0, &pv0, &mut pemap0);
                    if let Some((wire_index, edge_index, kind)) = idx00 {
                        geom_loops_store0.add_geom_vertex(
                            (face_index0, wire_index, edge_index),
                            &gv0,
                            kind,
                            &surface1,
                            &mut gemap0,
                        )?;
                        let polyline = intersection_curve.leader_mut();
                        *polyline.first_mut().unwrap() = gv0.point();
                    }
                    let idx01 =
                        poly_loops_store0.add_polygon_vertex(face_index0, &pv1, &mut pemap1);
                    if let Some((wire_index, edge_index, kind)) = idx01 {
                        geom_loops_store0.add_geom_vertex(
                            (face_index0, wire_index, edge_index),
                            &gv1,
                            kind,
                            &surface1,
                            &mut gemap1,
                        )?;
                        let polyline = intersection_curve.leader_mut();
                        *polyline.last_mut().unwrap() = gv1.point();
                    }
                    let idx10 =
                        poly_loops_store1.add_polygon_vertex(face_index1, &pv0, &mut pemap0);
                    if let Some((wire_index, edge_index, kind)) = idx10 {
                        geom_loops_store1.add_geom_vertex(
                            (face_index1, wire_index, edge_index),
                            &gv0,
                            kind,
                            &surface0,
                            &mut gemap0,
                        )?;
                        let polyline = intersection_curve.leader_mut();
                        *polyline.first_mut().unwrap() = gv0.point();
                    }
                    let idx11 =
                        poly_loops_store1.add_polygon_vertex(face_index1, &pv1, &mut pemap1);
                    if let Some((wire_index, edge_index, kind)) = idx11 {
                        geom_loops_store1.add_geom_vertex(
                            (face_index1, wire_index, edge_index),
                            &gv1,
                            kind,
                            &surface0,
                            &mut gemap1,
                        )?;
                        let polyline = intersection_curve.leader_mut();
                        *polyline.last_mut().unwrap() = gv1.point();
                    }
                    let pedge = Edge::new(&pv0, &pv1, polyline);
                    let gedge = Edge::new(&gv0, &gv1, intersection_curve.into());
                    poly_loops_store0[face_index0].add_edge(pedge.clone(), status0);
                    geom_loops_store0[face_index0].add_edge(gedge.clone(), status0);
                    poly_loops_store1[face_index1].add_edge(pedge, status1);
                    geom_loops_store1[face_index1].add_edge(gedge, status1);
                }
                Some(())
            })
        })?;

    // coplanar面の隣の面で辺に沿って発生するAndループを削除
    for face_index0 in adjacent_to_coplanar_faces_index_0 {
        finalize_adjacent_to_coplanar_faces::<C, S>(&mut geom_loops_store0, face_index0);
    }
    for face_index1 in adjacent_to_coplanar_faces_index_1 {
        finalize_adjacent_to_coplanar_faces::<C, S>(&mut geom_loops_store1, face_index1);
    }

    // 両方がcoplanar面なら通常処理のあとfinalize_coplanar_faces
    for &(face_index0, face_index1) in &coplanar_faces_index {
        let ori0 = geom_shell0[face_index0].orientation();
        let ori1 = geom_shell1[face_index1].orientation();
        finalize_coplanar_faces::<C, S>(
            &mut geom_loops_store0,
            &mut geom_loops_store1,
            ori0,
            ori1,
            face_index0,
            face_index1,
        );
    }

    Some(LoopsStoreQuadruple {
        geom_loops_store0,
        poly_loops_store0,
        geom_loops_store1,
        poly_loops_store1,
    })
}

fn second_is_coplanar_face(
    _face_index0: usize,
    face_index1: usize,
    coplanar_faces_index: &Vec<(usize, usize)>,
) -> bool {
    coplanar_faces_index.iter().any(|&(_i, j)| j == face_index1)
}
fn first_is_coplanar_face(
    face_index0: usize,
    _face_index1: usize,
    coplanar_faces_index: &Vec<(usize, usize)>,
) -> bool {
    coplanar_faces_index.iter().any(|&(i, _j)| i == face_index0)
}

#[cfg(test)]
mod tests;
