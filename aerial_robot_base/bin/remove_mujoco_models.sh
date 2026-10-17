#!/usr/bin/env bash

for package in dragon hydrus hydrus_xi mini_quadrotor gimbalrotor; do
  if model_package_path=$(rospack find "$package"); then
    rm -rf -- "$model_package_path/mujoco"
  fi
done
