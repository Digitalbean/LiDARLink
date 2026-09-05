#!/bin/bash
# Build the Mac app (Release, Apple-silicon only) and launch it.
#
# The x86_64 excludes are load-bearing: xcodebuild otherwise compiles the SPM
# dependency for x86_64, where Float16 (the wire codec) is unavailable. They
# can't live in project.yml — SPM package targets don't inherit them — so they
# go here.
set -euo pipefail
cd "$(dirname "$0")/.."

CONFIG="${1:-Release}"
DD=/tmp/lidarmac-dd

pkill -x LiDARLinkMac 2>/dev/null || true
sleep 1
rm -rf "$DD"

xcodebuild -project LiDARLinkMac/LiDARLinkMac.xcodeproj \
  -scheme LiDARLinkMac -configuration "$CONFIG" \
  -destination 'platform=macOS' -derivedDataPath "$DD" \
  ONLY_ACTIVE_ARCH=YES EXCLUDED_ARCHS=x86_64 \
  build

open "$DD/Build/Products/$CONFIG/LiDARLinkMac.app"
echo "launched $CONFIG build"
