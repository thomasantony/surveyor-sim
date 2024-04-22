use bevy::{math::DVec3, prelude::*};

use bevy::math::EulerRot::XYZ;
// use bevy_mod_paramap::*;
use big_space::{reference_frame::RootReferenceFrame, GridCell};
use std::str::FromStr;
use surveyor_physics::universe::Universe;
use surveyor_types::CelestialBodyType;

pub const MOON_RADIUS: f64 = 1737.1e3; // meters
pub const EARTH_RADIUS: f64 = 6378.14e3; // meters

use std::f32::consts::TAU;

use crate::GridCellType;

const EARTH_ALBEDO_MAP: &str = "textures/earth/base_color.jpg";
// const EARTH_NORMAL_MAP: &str = "textures/earth/normal_map.jpg";
// const EARTH_HEIGHT_MAP: &str = "textures/earth/elevation_surface.jpg";
// const EARTH_ROUGH_MAP: &str = "textures/earth/metallic_roughness.png";
// const EARTH_EMI_MAP: &str = "textures/earth/emissive.jpg";

const MOON_ALBEDO_MAP: &str = "textures/moon/base_color.jpg";
// const MOON_NORMAL_MAP: &str = "textures/moon/normal_map.jpg";
// const MOON_HEIGHT_MAP: &str = "textures/moon/elevation_surface.jpg";

/// Store handle of the earth normal to later modify its format
/// in [`update_normal`].
#[derive(Resource)]
pub struct Normal(pub Option<Handle<Image>>);

#[derive(Component, PartialEq, Eq)]
pub struct CelBody(pub CelestialBodyType);

/// Spawns a planet at given position while accounting for floating origin effects due to
/// large distances.
pub fn spawn_celestial_body(
    name: &'static str,
    position: DVec3,
    mesh_handle: Handle<Mesh>,
    matl_handle: Handle<StandardMaterial>,
    settings: &RootReferenceFrame<GridCellType>,
) -> impl Bundle {
    let (planet_grid_cell, translation) = settings.translation_to_grid(position);
    let planet_material: MaterialMeshBundle<StandardMaterial> = PbrBundle {
        mesh: mesh_handle,
        material: matl_handle,
        transform: Transform::from_rotation(Quat::from_euler(XYZ, -TAU / 4.0, 0.0, TAU / 2.0))
            .with_translation(translation),
        ..default()
    };
    (
        planet_material,
        planet_grid_cell,
        Name::new(name),
        CelBody(CelestialBodyType::from_str(&name).unwrap()),
    )
}

pub fn setup_planet(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    // mut normal: ResMut<Normal>,
    settings: Res<RootReferenceFrame<GridCellType>>,
    assets: Res<AssetServer>,
) {
    println!("Setting up planets");
    // let normal_handle = assets.load(MOON_NORMAL_MAP);
    // normal.0 = Some(normal_handle.clone());

    let mut sphere: Mesh = Mesh::from(shape::UVSphere {
        radius: MOON_RADIUS as f32,
        sectors: 360,
        stacks: 180,
        ..Default::default()
    });
    sphere.generate_tangents().unwrap();
    let moon_mesh = meshes.add(sphere);
    let moon_material = materials.add(StandardMaterial {
        base_color_texture: Some(assets.load(MOON_ALBEDO_MAP)),
        // normal_map: Some(normal_handle),
        ..default()
    });
    // Moon-centered inertial frame
    let moon_pos = DVec3::new(0.0, 0.0, 0.0);
    commands.spawn(spawn_celestial_body(
        "Moon",
        moon_pos,
        moon_mesh,
        moon_material,
        &settings,
    ));

    let mut sphere: Mesh = Mesh::from(shape::UVSphere {
        radius: EARTH_RADIUS as f32,
        sectors: 360,
        stacks: 180,
        ..Default::default()
    });
    sphere.generate_tangents().unwrap();
    let earth_mesh = meshes.add(sphere);
    let earth_material = materials.add(StandardMaterial {
        base_color_texture: Some(assets.load(EARTH_ALBEDO_MAP)),
        emissive: Color::rgb_u8(30, 30, 30),
        ..default()
    });
    let earth_pos = DVec3::new(386000e3, 0.0, 0.0);
    commands.spawn(spawn_celestial_body(
        "Earth",
        earth_pos,
        earth_mesh,
        earth_material,
        &settings,
    ));

    // // let sun_matl_handle = materials.add(StandardMaterial {
    // //     base_color: Color::rgb_u8(255, 255, 255),
    // //     emissive: Color::rgb_u8(255, 255, 255),
    // //     ..default()
    // // });

    // *Really bad* approximate position of the sun
    let sun_pos = DVec3::new(-100.0 * EARTH_RADIUS, 0.0, -100.0 * EARTH_RADIUS);
    let (sun_grid_cell, translation) = settings.translation_to_grid(sun_pos);
    commands
        .spawn(DirectionalLightBundle {
            transform: Transform::from_translation(translation).looking_at(Vec3::ZERO, Vec3::Y),
            directional_light: DirectionalLight {
                illuminance: 100_000.0,
                ..default()
            },
            ..default()
        })
        .insert((
            sun_grid_cell,
            Name::new("Sun"),
            CelBody(CelestialBodyType::Sun),
        ));
}

// /// Work around the fact that the default bevy image loader sets the
// /// normal's format to something incompatible with normal shaders.
// /// The format must be one of the `TextureFormat` ending in `*Unorm`.
// ///
// /// In this function, we wait until the image is loaded, immediately
// /// change its format and never run the core logic afterward.
// ///
// /// Without proper format, it looks like the light source moves as the
// /// earth move, and there is major glitchy artifacts on the poles.
// pub fn update_normal(
//     mut already_ran: Local<bool>,
//     mut images: ResMut<Assets<Image>>,
//     normal: Res<Normal>,
// ) {
//     if *already_ran {
//         return;
//     }
//     if let Some(normal) = normal.0.as_ref() {
//         if let Some(mut image) = images.get_mut(normal) {
//             image.texture_descriptor.format = TextureFormat::Rgba8Unorm;
//             *already_ran = true;
//         }
//     }
// }

// Update mesh transforms from universe
pub fn render_celbody_position(
    mut celbody: Query<(&CelBody, &mut Transform, &mut GridCell<GridCellType>)>,
    mut universe: Query<&Universe>,
    settings: Res<RootReferenceFrame<GridCellType>>,
) {
    let universe = universe.single_mut();
    for (celbody, mut transform, mut grid_cell) in celbody.iter_mut() {
        let pos = universe.celestial_bodies.get(&celbody.0).unwrap().position;
        let pos = DVec3::new(pos.x, pos.y, pos.z);
        let (new_grid_cell, translation) = settings.translation_to_grid(pos);
        transform.translation = translation;
        if new_grid_cell != *grid_cell {
            *grid_cell = new_grid_cell;
        }
    }
}
