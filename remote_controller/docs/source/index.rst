remote_controller
=================

``remote_controller`` is a remote-control framework for a Franka Panda setup.
It includes a C++ XML-RPC server, a Python client package, UDP robot-state
streaming, gripper helpers, and Pinocchio-based kinematics utilities using the
packaged Panda URDF/assets.

.. warning::

   This project sends commands to a real robot. Start with small motions, low
   velocity limits, and a clear workspace. Keep the Franka desk/user stop
   reachable at all times.

What This Project Provides
--------------------------

* C++ XML-RPC server for arm and gripper commands.
* Python package named ``remote_controller``.
* UDP state stream containing ``q``, ``dq``, ``K_F_ext_hat``, ``arm_state``, and ``gripper_state``.
* Unified Python helpers for direct and queued motion commands.
* Panda URDF/assets packaged for Pinocchio kinematics.

Common Units
------------

* Joint position: radians.
* Joint velocity: radians/second.
* Cartesian translation: meters.
* Cartesian linear velocity: meters/second.
* Cartesian rotation: radians.
* Gripper width and tolerances: meters.
* Gripper force: newtons.

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   installation
   quickstart
   safety
   examples
   udp
   kinematics
   rpc_api

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   python_api

.. toctree::
   :maxdepth: 1
   :caption: Development

   development
