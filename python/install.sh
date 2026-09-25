#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
python3 -m venv .venv
source .venv/bin/activate
pip install . 
pip install ".[ros]"
pip install ".[tools]"
pip install ".[map]"
pip install ".[net]"
./build_binary.sh

