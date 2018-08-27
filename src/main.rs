extern crate nalgebra;
extern crate ncollide2d;
extern crate nphysics2d;

use nalgebra::{Isometry2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::object::{BodyHandle, ColliderHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;

fn create_world() -> World<f32> {
    let mut world = World::new();
    world.set_timestep(0.01);
    world.set_gravity(Vector2::new(0.0, 0.0));
    world
}

fn create_rigidbody(world: &mut World<f32>, pos: &Vector2<f32>) -> (BodyHandle, ColliderHandle) {
    let shape = ShapeHandle::new(Cuboid::new(Vector2::new(0.5, 0.5)));

    let body_handle = world.add_rigid_body(
        Isometry2::new(Vector2::new(pos.x, pos.y), 0.0),
        shape.inertia(0.5),
        shape.center_of_mass(),
    );
    let collider_handle = world.add_collider(
        0.01,
        shape,
        body_handle,
        Isometry2::identity(),
        Material {
            restitution: 0.5,
            friction: 0.5,
        },
    );
    (body_handle, collider_handle)
}
fn main() {
    let mut world = create_world();
    let mut body_handles = vec![];

    loop {
        for i in 0..50 {
            body_handles.push(create_rigidbody(
                &mut world,
                &Vector2::new(i as f32, i as f32),
            ));
        }
        world.step();
        for _ in 0..2 {
            for handle in &body_handles {
                if let Some(rb) = world.rigid_body_mut(handle.0) {
                    rb.set_linear_velocity(Vector2::new(-1.0, -1.0));
                }
            }
            world.step();
        }
        
        world.remove_colliders(
            &body_handles
                .iter()
                .map(|x| x.1)
                .collect::<Vec<ColliderHandle>>(),
        );
        
        world.remove_bodies(
            &body_handles
                .iter()
                .map(|x| x.0)
                .collect::<Vec<BodyHandle>>(),
        );

        body_handles.clear();
        world.step();
    }
}
