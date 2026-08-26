#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_parent="$(dirname -- "$workspace_dir")"
workspace_name="$(basename -- "$workspace_dir")"
release_date="${OPENARM_RELEASE_DATE:-$(date +%Y%m%d)}"
output_dir="${1:-$workspace_dir/dist}"
archive_name="openarm_skeleton_v1.2_ws-source-${release_date}.tar.gz"
archive_path="$output_dir/$archive_name"
checksum_path="$archive_path.sha256"

mkdir -p "$output_dir"

tar \
  --exclude="$workspace_name/build" \
  --exclude="$workspace_name/install" \
  --exclude="$workspace_name/log" \
  --exclude="$workspace_name/dist" \
  --exclude="$workspace_name/.git" \
  --exclude="$workspace_name/.pytest_cache" \
  --exclude="*/__pycache__" \
  --exclude="*.pyc" \
  --exclude="*.pyo" \
  -C "$workspace_parent" \
  -czf "$archive_path" \
  "$workspace_name"

(
  cd "$output_dir"
  sha256sum "$archive_name" > "$archive_name.sha256"
)

tar -tzf "$archive_path" > /dev/null

printf 'Created: %s\n' "$archive_path"
printf 'Checksum: %s\n' "$checksum_path"
printf 'Size: '
du -h "$archive_path" | cut -f1
