# FAIRINO FR5

For an FR5 v6 whose controller runs WebAPP 3.9.5; any other model or version is refused at start-up, because the driver depends on that version of FAIRINO's SDK.

## Network

The controller's default address is `192.168.58.2`. Put the PC's wired port on the same subnet, then check the arm answers:

```bash
# The first wired interface; set IFACE by hand if the arm is on another one.
IFACE=$(ip -o link show | awk -F': ' '$2 ~ /^(en|eth)/ {print $2; exit}')
sudo ip addr add 192.168.58.10/24 dev "$IFACE"
ping 192.168.58.2
```

## Run

On the pendant, or in the WebAPP at `http://192.168.58.2`, switch the arm to automatic mode and enable it first. In manual mode the arm still reports its joints but takes no commands.

```bash
# Jog the wrist from the keyboard; plan and run trajectories from the Viser UI.
# Loads KeyboardTeleopModule, FR5TwistCoordinator, FR5Connection and ManipulationModule.
uv run dimos run keyboard-teleop-fr5
```

An arm at another address takes `--fr5connection.ip <IP>` after the blueprint name.

Add `--simulation` to try it without the arm.

The first run downloads FAIRINO's SDK and the arm's model, so it needs internet access once. A computer that only reaches the arm has to run it once with internet access beforehand; the SDK is then kept in `~/.cache/dimos/fairino/`.

The address set with `ip addr add` is lost when the controller restarts or the cable is unplugged: add it again.

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

After a fault, a collision or a stop the arm takes no commands but still reports its joints; the log says why. Deal with the cause (drag the arm back inside its limits in the pendant's drag mode and leave that mode, switch back to automatic mode, release the emergency stop), then clear the errors and enable the arm from `uv run dimos shell`, without restarting:

```python
app.FR5Connection.clear_errors()
```

Whatever the arm was doing or was told to do while stopped is dropped: a trajectory or a jog has to be given again. It raises an error, and the arm stays stopped, if the cause is still there.
