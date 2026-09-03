from .RemoteControllerClient import RemoteControllerClient

__all__ = ["RemoteControllerClient", "RobotModel"]


def __getattr__(name):
    """Load Pinocchio only for callers that actually need kinematics."""
    if name == "RobotModel":
        from .robot_kinematics import RobotModel

        return RobotModel
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
