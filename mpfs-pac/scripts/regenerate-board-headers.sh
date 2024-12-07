#! /bin/bash

set -e

# This directory
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
cd $SCRIPT_DIR/..
python mpfs-platform/platform/soc_config_generator/mpfs_configuration_generator.py mpfs-platform/boards/beaglev-fire/fpga_design/design_description/ mpfs-platform/boards/beaglev-fire
