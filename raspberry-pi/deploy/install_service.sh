#!/usr/bin/env bash
# Install mrs-krabs.service on the Raspberry Pi so mrs_krabs.py runs at boot
# inside the repo venv. Run on the Pi:  bash install_service.sh
set -euo pipefail

# Resolve the repo from this script's location: <repo>/raspberry-pi/deploy/
DEPLOY_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$DEPLOY_DIR/../.." && pwd)
VENV=$REPO/.venv
SRC=$REPO/raspberry-pi/src
MODEL=$REPO/raspberry-pi/best_ncnn_model
SERVICE=mrs-krabs.service

# The service runs as the human user, not root — it needs their venv and groups.
RUN_USER=${SUDO_USER:-$(id -un)}
[[ $RUN_USER != root ]] || { echo "run as the pi user (sudo is called per-command), not as root"; exit 1; }

echo "repo:    $REPO"
echo "venv:    $VENV"
echo "user:    $RUN_USER"
echo

fail=0
for p in "$VENV/bin/python" "$SRC/mrs_krabs.py" "$MODEL"; do
  if [[ -e $p ]]; then echo "ok       $p"; else echo "MISSING  $p"; fail=1; fi
done

# ncnn lives only in the repo venv, not in ~/robot_env — catch the wrong-venv case now.
if "$VENV/bin/python" -c 'import cv2, ncnn, numpy, serial, google.protobuf' 2>/dev/null; then
  echo "ok       imports (cv2, ncnn, numpy, serial, protobuf)"
else
  echo "MISSING  imports —"
  "$VENV/bin/python" -c 'import cv2, ncnn, numpy, serial, google.protobuf' || true
  fail=1
fi

[[ $fail -eq 0 ]] || { echo; echo "preflight failed, not installing"; exit 1; }

# Warnings only — these do not block install, but they are the usual causes of a restart loop.
echo
grep -q 'CAMERA_INDEX *= *0' "$SRC/config.py" \
  || echo "WARNING  CAMERA_INDEX is not 0 in $SRC/config.py — on the robot it should be 0, or the camera won't open"
ls /dev/ttyACM* >/dev/null 2>&1 \
  || echo "WARNING  no /dev/ttyACM* right now — ESP32 unplugged; the service will retry every 5s until it appears"

# MODEL_PATH in config.py is the relative "../best_ncnn_model", so WorkingDirectory
# must be src/ for it to resolve. Running $VENV/bin/python directly is equivalent to
# activating the venv; VIRTUAL_ENV and PATH are set so subprocesses agree.
echo
echo "writing /etc/systemd/system/$SERVICE"
sudo tee "/etc/systemd/system/$SERVICE" >/dev/null <<EOF
[Unit]
Description=Mrs Krabs - robot CV + UART main loop
After=multi-user.target

[Service]
Type=simple
User=$RUN_USER
SupplementaryGroups=dialout video
WorkingDirectory=$SRC
Environment=VIRTUAL_ENV=$VENV
Environment=PATH=$VENV/bin:/usr/local/bin:/usr/bin:/bin
Environment=PYTHONUNBUFFERED=1
ExecStart=$VENV/bin/python $SRC/mrs_krabs.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "$SERVICE"

sleep 3
echo
systemctl status "$SERVICE" --no-pager || true
echo
journalctl -u "$SERVICE" -n 30 --no-pager || true

cat <<EOF

follow logs:  journalctl -u ${SERVICE%.service} -f
restart:      sudo systemctl restart ${SERVICE%.service}
stop autostart while developing:
              sudo systemctl disable --now ${SERVICE%.service}
EOF
