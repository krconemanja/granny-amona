#!/usr/bin/env bash
# Install the DDBoat GUI on a Steam Deck (desktop mode) or any Linux laptop.
# Everything goes into a venv next to this script; the read-only SteamOS
# system partition is not touched and no sudo is needed.
set -euo pipefail
cd "$(dirname "$0")"
DIR="$(pwd)"

python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt

mkdir -p "$HOME/.local/share/applications"
cat > "$HOME/.local/share/applications/ddboat-gui.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=DDBoat Control
Comment=Steam Deck remote control for the DDBoat
Exec=$DIR/.venv/bin/python $DIR/boat_gui.py --fullscreen
Path=$DIR
Icon=input-gaming
Terminal=false
Categories=Utility;
EOF
chmod +x "$DIR/boat_gui.py"

echo
echo "Installed. Start it with:"
echo "  $DIR/.venv/bin/python $DIR/boat_gui.py"
echo "or from the application menu: DDBoat Control."
echo "For Game Mode: Steam > Add a Non-Steam Game > DDBoat Control."
