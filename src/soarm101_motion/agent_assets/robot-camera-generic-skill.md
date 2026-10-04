# SO-ARM101 OpenShell robot/camera skill

You are running inside an isolated OpenShell sandbox. The host Motion SDK, serial devices,
camera devices, calibration files, and operator controls are not available here.

Read `/sandbox/TASK.md` completely before acting. Use only:

```bash
python3 /sandbox/robotctl.py capabilities
python3 /sandbox/robotctl.py state
python3 /sandbox/robotctl.py capture overhead
python3 /sandbox/robotctl.py capture wrist
python3 /sandbox/robotctl.py go-pose NAME
python3 /sandbox/robotctl.py joint JOINT --delta-deg DEG
python3 /sandbox/robotctl.py jog --frame world --x-mm X --y-mm Y --z-mm Z
python3 /sandbox/robotctl.py jog --frame tool --x-mm X --y-mm Y --z-mm Z
python3 /sandbox/robotctl.py gripper open
python3 /sandbox/robotctl.py gripper close
python3 /sandbox/robotctl.py sleep
python3 /sandbox/robotctl.py stop
```

The broker may expose fewer routes during a read-only run. Treat any missing/rejected route as
authoritative. Never attempt to access host device files or recreate robot control outside
`robotctl.py`.

For visual claims, acquire a fresh named-camera capture and inspect the saved sandbox image
with whatever local image-inspection capability your harness provides. If the harness cannot
inspect local images, say so rather than guessing from filenames or task text.

Human arming and torque release remain outside the sandbox. Do not claim success from a
command request alone; use returned completion/state and fresh observations when the task
requires physical confirmation.
