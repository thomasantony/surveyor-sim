let lander_pos = DVec3::new(MOON_RADIUS + 100e3, 0.0, 0.0);
    let (grid_cell, lander_translation) = settings.translation_to_grid::<i128>(lander_pos);
    // in the SceneBundle
    commands.spawn((SceneBundle {
        scene: asset_server.load("Surveyor/Surveyor-Lander.gltf#Scene0"),
        transform: Transform::from_translation(lander_translation.clone()),
        ..default()
    }, Aabb::from(Sphere {
        center: Vec3A::ZERO,
        radius: 0.5
    }), grid_cell, Name::new("Lander")));

    let camera_pos = DVec3::new(MOON_RADIUS + 100e3 +10.0, 0.0, 0.0);
    let (grid_cell, camera_translation) = settings.translation_to_grid::<i128>(camera_pos.clone());
    let camera_transform = Transform::from_translation(camera_translation)
                                            .looking_at(lander_translation, Vec3::Y);
