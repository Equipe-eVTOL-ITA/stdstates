# stdstates

Standard reusable FSM states for eVTOL ITA drone projects.

## Contents

Pre-built finite state machine states:
- `TakeoffState` — autonomous takeoff sequence
- `LandingState` — autonomous landing sequence
- `NextWaypoints` — navigate through waypoints

## Dependencies

- `fsm` — finite state machine framework
- `drone_lib` — PX4 drone abstraction (includes PID, movement, transformations)

## Usage

```cpp
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"
#include "stdstates/next_waypoints.hpp"
```

## License

MIT
