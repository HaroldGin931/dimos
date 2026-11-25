# FAIRINO FR5

## Network

The controller's default address is `192.168.58.2`. Put the PC's wired port on the same subnet, then check the arm answers:

```bash
# The first wired interface; set IFACE by hand if the arm is on another one.
IFACE=$(ip -o link show | awk -F': ' '$2 ~ /^(en|eth)/ {print $2; exit}')
sudo ip addr add 192.168.58.10/24 dev "$IFACE"
ping 192.168.58.2
```

## Run

Enable the arm on its pendant first.

```bash
# Basic: runs joint trajectories sent to it. No UI.
# Loads FR5Coordinator and FR5Connection.
uv run dimos run coordinator-fr5 --fr5connection.ip 192.168.58.2

# Planner: basic, plus planning and running trajectories from the Viser UI.
# Loads FR5Coordinator, FR5Connection and ManipulationModule.
uv run dimos run fr5-planner-coordinator --fr5connection.ip 192.168.58.2

# Keyboard teleop: the planner, plus jogging the wrist from the keyboard.
# Loads KeyboardTeleopModule, FR5TwistCoordinator, FR5Connection and ManipulationModule.
uv run dimos run keyboard-teleop-fr5 --fr5connection.ip 192.168.58.2
```
