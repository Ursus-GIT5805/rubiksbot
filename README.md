# rubiksbot

Rubik's Cube robot application running on a Raspberry PI.
It uses OpenCV to detect the cubes configuration.

## Cross compilation

This project depends on OpenCV. To compile it:

- Get a `aarch64-unknown-linux-gnu` port of OpenCV
- Set the variable `OPENCV_PATH`. The value must be a comma
  seperated list of paths where the OpenCV port is located.
