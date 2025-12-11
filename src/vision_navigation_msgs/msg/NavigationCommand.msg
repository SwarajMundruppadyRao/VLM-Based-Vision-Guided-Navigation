# NavigationCommand.msg - Message for navigation commands from VLM

std_msgs/Header header
string command_type         # Type of command (STOP, FORWARD, TURN_LEFT, etc.)
geometry_msgs/Twist velocity # Desired velocity
float32 duration            # Duration to execute command (seconds)
string description          # Human-readable description
