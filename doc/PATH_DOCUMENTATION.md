# PathPlanner Path Documentation

## Active Paths

| Path Name | Starting Position | Goal Position | Purpose | Status |
|-----------|------------------|---------------|---------|---------|
| AmpToCenter | X: 1.83, Y: 7.5 | X: 8.3, Y: 7.5 | Move from Amp to field center | Active |
| SourceToSpeaker | X: 15.2, Y: 0.5 | X: 15.7, Y: 5.5 | Position for speaker shot | Active |

## Path Constraints
- Max Velocity: 4.5 m/s
- Max Acceleration: 3.0 m/s²
- Holonomic Rotation: Enabled

## Associated Autonomous Routines
- The codebase is currently using the "Default Drop F.auto" path
- Other auto routines in ./pathplanner/autos/


## Field References
- Field Image: `src/main/deploy/pathplanner/2025_Field.png`
- Coordinate System: Origin at blue alliance bottom-right corner
- Units: Meters

## Testing Status
⚠️ Paths should be verified on practice field before competition use
- [ ] Test on practice field
- [ ] Validate with vision alignment
- [ ] Verify with game pieces