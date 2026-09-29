#!/usr/bin/env bash
# Descarga los modelos BlazePose (.task) de MediaPipe a models/ (ADR-018).
# Los binarios no se commitean al repo (ver models/.gitignore).
# Uso: scripts/download_models.sh [directorio_destino]
set -euo pipefail

DEST="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/models}"
mkdir -p "$DEST"
BASE="https://storage.googleapis.com/mediapipe-models/pose_landmarker"

for variant in lite full heavy; do
  url="$BASE/pose_landmarker_${variant}/float16/latest/pose_landmarker_${variant}.task"
  out="$DEST/pose_landmarker_${variant}.task"
  if [ -f "$out" ]; then
    echo "[download_models] Ya existe: $out"
    continue
  fi
  echo "[download_models] Descargando $variant -> $out"
  wget -q -O "$out" "$url"
done
echo "[download_models] Listo."
