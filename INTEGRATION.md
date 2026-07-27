# SO-ARM101 Motion SDK Integration

Robo Director is the primary source of Agentic Forge cross-repository integration requirements. This SDK is a motion library, not a Director service: Robo Studio and Robo Puppeteer wrap it and expose Director actions, lifecycle events, health, and resource claims.

## Contract metadata

- Contract family: `agenticforge.robotics`
- Supported integration line: `0.2.x`
- Library manifest: `.agenticforge/integration-manifest.json`
- Python metadata: `soarm101_motion.integration`
- Default physical resource: `motion-platform:soarm101`

## Adapter responsibilities

Consumer adapters must:

1. Connect and enable the arm explicitly.
2. Use nonblocking motion handles when Director cancellation is required.
3. Wait for measured completion before emitting a success terminal event.
4. Translate SDK exceptions into stable service-level failures.
5. Claim the same `motion-platform:<id>` resource in Studio and Puppeteer.
6. Treat `stop()` as a controlled software stop, not a certified emergency stop.
7. Require an explicit named frame before converting an external pose into the SDK's frame-free `Pose` type.

## Units and orientation

The SDK uses meters, radians, and seconds. `Pose.rotation` is a 3×3 rotation matrix. Studio converts its normalized `x,y,z,w` quaternion to this matrix without changing units.

## TCP ownership

The SDK owns flange-to-active-TCP transforms used by FK, IK, and motion planning. A camera consumer may add active-TCP-to-optical-frame calibration and stage extrinsics, but it must not configure the flange transform a second time.

## Puppeteer boundary

Puppeteer exposes only named semantic gestures. Director scripts never send raw joints, motor IDs, register writes, calibration changes, or unrestricted Cartesian targets through the character interface.

## Validation gate

Simulation validates API integration and cancellation. Physical hardware validation remains required for calibration direction, low-speed movement, feedback completion, stop behavior, TCP accuracy, and workspace safety.
