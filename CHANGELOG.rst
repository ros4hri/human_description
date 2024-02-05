^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package human_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2024-02-05)
------------------
* fix joint limits for the arms
  the joint limits for the arms were wrong, as they did not really
  reflect the range of movements a human is able to perform. This
  commit introduces realistic joint limits for human arms.
* Contributors: lorenzoferrini

2.0.1 (2023-11-13)
------------------
* port to ROS2 humble
* Contributors: Séverin Lemaignan

1.0.0 (2022-01-13)
------------------
* v1 of the human URDF model
* Initial commit
* Contributors: Séverin Lemaignan
