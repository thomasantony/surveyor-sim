- `surveyor-gnc` will be the main flight-software module
- `surveyor-sim` will contain all the simulation code
    - this includes the "universe" as well as spacecraft components
    - possibly move the universe/physics/integrator stuff into a separate crate later


- New design for simulator
    -


Could have a new flow:

1. update_discrete on all subsystems
2. integrate
...
