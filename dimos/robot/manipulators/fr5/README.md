# FAIRINO FR5

For an FR5 v6 whose controller runs WebAPP 3.9.5; any other model or version is refused at start-up, because the driver depends on that version of FAIRINO's SDK.

## Network

The controller's default address is `192.168.58.2`. To reach it, give your computer's wired (LAN) interface an address on the same subnet, then check the arm answers:

```bash
# <your LAN interface> is the wired port the arm is plugged into, e.g. enp12s0;
# `ip -br link` lists the interfaces.
sudo ip addr add 192.168.58.10/24 dev <your LAN interface>
ping 192.168.58.2
```

## Run

On the pendant, or in the WebAPP at `http://192.168.58.2`, enable the arm first, and leave it in manual mode. It takes commands in manual mode as in automatic mode, and in manual mode it can be dragged by hand, which is how a joint past its limit is brought back (see Faults).

```bash
# Jog the wrist from the keyboard; plan and run trajectories from the Viser UI.
# Loads KeyboardTeleopModule, FR5TwistCoordinator, FR5Connection and ManipulationModule.
uv run dimos run keyboard-teleop-fr5
```

An arm at another address takes `--fr5connection.ip <IP>` after the blueprint name.

Add `--simulation` to try it without the arm.

The first run downloads FAIRINO's SDK and the arm's model, so it needs internet access once. A computer that only reaches the arm has to run it once with internet access beforehand; the SDK is then kept in `~/.cache/dimos/fairino/`.

The address set with `ip addr add` is lost when the controller restarts or the cable is unplugged: add it again.

## Gripper

`keyboard-teleop-fr5` expects a DH-Robotics PGI-140-80 on the flange, driven by the controller over the tool-end RS-485; the keyboard's `[` and `]` open and close it. In Python, `gripper=False` leaves it out: `fr5_model`, `make_fr5_model_config` and `FR5Connection.blueprint` each take it.

Configure the gripper once in the WebAPP, under Initial Setup > Peripherals > Gripper, as manufacturer DH (大寰), type PGI-140, device ID 1. To change a configuration, select it and clear it first.

A gripper that is not activated is reset and activated when the blueprint starts, and again by `clear_errors()`: **its fingers then run through their travel to find its ends, so nothing may be between them.** It is then opened, as is a gripper that was activated on the WebAPP and not moved since: until its first move the controller reports it closed wherever its fingers are. If it does not activate, the log says so and the gripper takes no target; the arm works as without it, and `clear_errors()` tries again.

The gripper is the joint `arm/gripper`: how far each finger is from closed, 0 m to 0.04 m (80 mm between the fingers). It moves at 30 % speed and force. It still opens and closes while a fault holds the arm.

The arm stands still while the gripper moves and for 2 seconds after, about 6 seconds in all, and a trajectory or a jog under way is dropped and has to be given again. This controller stops answering for over a second as a gripper move ends, which a moving arm's command stream does not survive, so the stream is ended first and started again afterwards.

Its model, `pgi140.urdf`, is boxes around the parts of DH's meshes, on a 7 mm adapter plate (`FR5_GRIPPER_MOUNT_M` in `config.py`). The fingers open along the flange's x, with the gripper's cable socket on the +x side.

Set the gripper's weight and centre of mass as the tool load on the pendant, or the controller's collision detection goes by the bare flange.

## Limits

Every joint is told to turn at 0.3 rad/s (17°/s) at most, whether by a keyboard jog, a Viser joint slider or a planned trajectory.

Commands stop 2° short of the controller's soft limits, because the controller faults when a joint reaches one:

| Joint | Soft limit | Commands stop at |
|-------|------------|------------------|
| j1, j5, j6 | ±175° | ±173° |
| j2, j4 | −265° to 85° | −263° to 83° |
| j3 | ±160° | ±158° |

The soft limits are the ones in `config.py` (`FR5_SOFT_LIMITS_DEG`), not read from the controller: change them there if they are changed on the pendant. A joint dragged past where commands stop takes no command until it is dragged back.

Nothing stops a jog at the edge of the arm's reach or before the arm meets itself: keep jogs short and watch the arm.

## Faults

After a fault, a collision or a stop the arm takes no commands but still reports its joints; the log says why. Deal with the cause (drag the arm back inside its limits in the pendant's drag mode and leave that mode, release the emergency stop), then clear the errors and enable the arm from `uv run dimos shell`, without restarting:

```python
app.FR5Connection.clear_errors()
```

Whatever the arm was doing or was told to do while stopped is dropped: a trajectory or a jog has to be given again. It raises an error, and the arm stays stopped, if the cause is still there.
