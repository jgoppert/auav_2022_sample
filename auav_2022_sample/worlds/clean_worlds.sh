#!/bin/bash
# note since model generation is embedded via jinja, worlds won't know when models are updated
# so clean them if you need to update the models
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
rm ${SCRIPT_DIR}/*.world

