# Puppeteer SO-ARM101 session boundary

Forge Puppeteer's native ensemble owns the SO-ARM101 SDK executor and its
watchdog for the lifetime of the room. This is a persistent process/session,
not a second service to launch beside the room. Puppeteer submits only
operator-pinned `JointChoreography`/`JointTarget` assets through the SDK's
trusted-host API; the model cannot submit coordinates, revisions, or policy.

The SDK's authenticated HTTP broker is for bounded agent capabilities. Its
physical autonomous agent actions retain the human-issued, calibration-bound
lease. A separately configured physical daemon can select
`operator_choreography_only: true`: it advertises health, capabilities, state,
and STOP only, and the SDK executor rejects state-changing agent actions even
if the profile is misconfigured. Exact-pinned choreography remains on the
authenticated trusted-host route. Operator-only supervision validates the
active SDK session and bound robot/calibration without an agent lease; startup
still requires identity and stationary torque-HOLD preflight by default.

An explicitly operator-authorized daemon configuration can set
`establish_hold_on_startup: true` alongside `operator_choreography_only: true`.
After read-only identity/calibration/health checks, the daemon starts its
existing watchdog and uses the SDK's measured-position latch to enable torque
once, then observes encoder stability before advertising readiness. Existing
HOLD is verified without relatching. No saved pose or stale goal is executed by
this latch; the pinned wake target remains a subsequent operation. Failed or
uncertain startup HOLD faults the session and leaves
`startup-hold-failure.json` in the private state directory. Startup refuses to
reconnect while that marker exists; operator reconciliation is required.
The agent action interface cannot select this option, arm itself or retry it.
Pinned targets and programs in this operator-only mode validate the bound
SDK session/watchdog rather than an agent lease. STOP remains independently
available through both the profile-granted STOP route and private trusted-host
route. An unconfirmed shutdown also leaves
`operator-reconciliation-required.json`; neither marker may be removed as an
automatic reconnect/retry step.

The existing lease-gated autonomous mode is unchanged. The physical-daemon
operator mode is selected only in its private daemon config and does not create
or require an agent lease. Neither mode silently mints or extends authority.

For the operator-side nonconnecting config check and exact launch commands,
see Puppeteer's [SO-ARM101 operator run guide](https://github.com/AgenticForge-Labs/forge-puppeteer/blob/main/docs/SOARM101_OPERATOR_RUN.md).
Neither repository's software-only checks establish physical clearance,
settling, or safety-rated stopping.
