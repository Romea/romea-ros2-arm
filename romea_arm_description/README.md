# 1 Overview #

This package contains the description of gps sensors used in romea projects

# 2 Package organization #

This package is organized into subdirectories as follows:

  - config/ contains characteristic description of following gps receivers:

    - astech proflex 800
    - thales zmax
    - ublox evk m8 receivers
    - drotek f9p

  - python/ contains romea_gps_description python module able to create receiver URDF description according their xacro representations and required parameters given by user

  - urdf/ contains (xacro representations of) urdf descriptions of supported gps sensors.
