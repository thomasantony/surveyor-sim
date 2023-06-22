FSW architecture
----------------

IMU Sensor    ->
                   \    (all measurements)                      (q and omega)                              (torque req)
(Some other sensor)  --> SensorAggregator --> Attitude Estimator    -->    Guidance   --> Attitude Controller  -->    RCS Controller
                                     \                                      \  (attitude/vel)     \-------------->    Engine and TVC Controller
                   /                      --> ?                              \                             /
Star Tracker    ->                                                            \  --> Velocity Controller  /   (thrust req)
                /
Radar       ->                              Ephemeris Estimator ->

- Commands would be processed how?
    - Can be represented by an enum that gets routed to various systems/resources/components based on variant

- Add command to set Guidance mode
- Add command to enable/disable sensor in SensorAggregator
- Add command to directly enable/disable actuator in ControlAllocator
