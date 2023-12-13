//! A model for the system clock (this is controlled by events/data from the simulator)

use bevy_ecs::prelude::*;
use hifitime::prelude::*;

#[derive(Debug, Clone, Resource, Default)]
pub struct SystemClock {
    pub time: Epoch,
}

#[derive(Debug, Clone, Event)]
pub struct TimeTickEvent {
    pub time: Epoch,
}

pub fn update_sys_clock(mut sys_clock: ResMut<SystemClock>, mut time_tick_event: EventReader<TimeTickEvent>) {
    if let Some(time) = time_tick_event.read().last() {
        // screen_print!("System clock: {}", time.time);
        sys_clock.time = time.time;
    }
}
