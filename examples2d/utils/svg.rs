use vectorg_engine_2d::prelude::*;

use lyon::math::Point;
use lyon::path::PathEvent;
use lyon::tessellation::geometry_builder::*;
use lyon::tessellation::{self, FillOptions, FillTessellator};
use usvg::prelude::*;
use vectorg_engine_2d::na::{Point2, Rotation2};

const VECTORG_ENGINE_SVG_STR: &str = r##"
<svg width="220" height="100" viewBox="0 0 220 100" xmlns="http://www.w3.org/2000/svg">
    <path d="M10 10 L42 90 L74 10 L58 10 L42 62 L26 10 Z" fill="#4fd1c5"/>
    <path d="M105 15 H190 V30 H120 V70 H175 V58 H150 V44 H205 V85 H105 Z" fill="#f7fafc"/>
</svg>
"##;

pub fn vectorg_engine_logo() -> Vec<(Vec<Point2<f32>>, Vec<[u32; 3]>)> {
    tessellate_svg_str(VECTORG_ENGINE_SVG_STR)
}

pub fn tessellate_svg_str(svg_str: &str) -> Vec<(Vec<Point2<f32>>, Vec<[u32; 3]>)> {
    let mut result = vec![];
    let mut fill_tess = FillTessellator::new();
    let opt = usvg::Options::default();
    let rtree = usvg::Tree::from_str(svg_str, &opt).unwrap();

    for node in rtree.root().descendants() {
        if let usvg::NodeKind::Path(ref p) = *node.borrow() {
            let transform = node.transform();
            if p.fill.is_some() {
                let path = PathConvIter {
                    iter: p.data.iter(),
                    first: Point::new(0.0, 0.0),
                    prev: Point::new(0.0, 0.0),
                    deferred: None,
                    needs_end: false,
                };

                let mut mesh: VertexBuffers<_, u32> = VertexBuffers::new();
                fill_tess
                    .tessellate(
                        path,
                        &FillOptions::tolerance(0.01),
                        &mut BuffersBuilder::new(&mut mesh, VertexCtor { prim_id: 0 }),
                    )
                    .expect("Tessellation failed.");

                let angle = transform.get_rotate() as f32;

                let (sx, sy) = (
                    transform.get_scale().0 as f32 * 0.2,
                    transform.get_scale().1 as f32 * 0.2,
                );

                let indices: Vec<_> = mesh.indices.chunks(3).map(|v| [v[0], v[1], v[2]]).collect();
                let vertices: Vec<_> = mesh
                    .vertices
                    .iter()
                    .map(|v| {
                        Rotation2::new(angle) * point![v.position[0] * sx, v.position[1] * -sy]
                    })
                    .collect();

                result.push((vertices, indices));
            }
        }
    }

    result
}

struct PathConvIter<'a> {
    iter: std::slice::Iter<'a, usvg::PathSegment>,
    prev: Point,
    first: Point,
    needs_end: bool,
    deferred: Option<PathEvent>,
}

impl Iterator for PathConvIter<'_> {
    type Item = PathEvent;
    fn next(&mut self) -> Option<PathEvent> {
        if self.deferred.is_some() {
            return self.deferred.take();
        }

        let next = self.iter.next();
        match next {
            Some(usvg::PathSegment::MoveTo { x, y }) => {
                if self.needs_end {
                    let last = self.prev;
                    let first = self.first;
                    self.needs_end = false;
                    self.prev = Point::new(*x as f32, *y as f32);
                    self.deferred = Some(PathEvent::Begin { at: self.prev });
                    self.first = self.prev;
                    Some(PathEvent::End {
                        last,
                        first,
                        close: false,
                    })
                } else {
                    self.first = Point::new(*x as f32, *y as f32);
                    Some(PathEvent::Begin { at: self.first })
                }
            }
            Some(usvg::PathSegment::LineTo { x, y }) => {
                self.needs_end = true;
                let from = self.prev;
                self.prev = Point::new(*x as f32, *y as f32);
                Some(PathEvent::Line {
                    from,
                    to: self.prev,
                })
            }
            Some(usvg::PathSegment::CurveTo {
                x1,
                y1,
                x2,
                y2,
                x,
                y,
            }) => {
                self.needs_end = true;
                let from = self.prev;
                self.prev = Point::new(*x as f32, *y as f32);
                Some(PathEvent::Cubic {
                    from,
                    ctrl1: Point::new(*x1 as f32, *y1 as f32),
                    ctrl2: Point::new(*x2 as f32, *y2 as f32),
                    to: self.prev,
                })
            }
            Some(usvg::PathSegment::ClosePath) => {
                self.needs_end = false;
                self.prev = self.first;
                Some(PathEvent::End {
                    last: self.prev,
                    first: self.first,
                    close: true,
                })
            }
            None => {
                if self.needs_end {
                    self.needs_end = false;
                    let last = self.prev;
                    let first = self.first;
                    Some(PathEvent::End {
                        last,
                        first,
                        close: false,
                    })
                } else {
                    None
                }
            }
        }
    }
}

struct VertexCtor {
    prim_id: u32,
}

impl FillVertexConstructor<GpuVertex> for VertexCtor {
    fn new_vertex(&mut self, vertex: tessellation::FillVertex) -> GpuVertex {
        GpuVertex {
            position: vertex.position().to_array(),
            prim_id: self.prim_id,
        }
    }
}

impl StrokeVertexConstructor<GpuVertex> for VertexCtor {
    fn new_vertex(&mut self, vertex: tessellation::StrokeVertex) -> GpuVertex {
        GpuVertex {
            position: vertex.position().to_array(),
            prim_id: self.prim_id,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
struct GpuVertex {
    position: [f32; 2],
    prim_id: u32,
}
