# LiDARLink

Stream LiDAR depth, color, and scene mesh from an iPhone Pro to a Mac in real
time, fuse it into a world-space point cloud, and export it.

## Layout

| Path | What it is |
|------|------------|
| `LiDARLinkShared/` | Swift package: wire protocol, depth/pose math, fusion, exporters. Shared by both apps. |
| `LiDARLinkPhone/`  | iOS capture app — ARKit scene depth + mesh, streams to the Mac. |
| `LiDARLinkMac/`    | macOS receiver/viewer — assembles frames, renders the cloud, records and exports. |
| `LiDARLinkTests/`  | XCTest suite (runs on macOS). |
| `LiDARLink.xcworkspace` | Opens all three projects together. |

The two apps reference `../LiDARLinkShared` by relative path, so the four
folders must stay siblings.

## Requirements

- A Mac with Xcode carrying the macOS 26 and iOS 17 SDKs (matches this
  project's deployment targets).
- A LiDAR-equipped iPhone (iPhone 12 Pro/Pro Max or later Pro/Pro Max model)
  running iOS 17+. The simulator has no LiDAR — the phone app needs a real
  device.
- An Apple ID signed into Xcode to build and install the phone app. A free
  account works; its self-signed builds just expire after 7 days and need
  reinstalling (`Settings → General → VPN & Device Management` to trust the
  developer certificate the first time).
- Both devices on the same local network (Wi-Fi), or a USB-C cable between
  them — see [Connecting](#connecting) below.

Project files (`.xcodeproj`) are checked in and ready to open directly; you
only need [XcodeGen](https://github.com/yonaskolb/XcodeGen) if you edit a
`project.yml` and want to regenerate them (`xcodegen generate` in that
project's folder).

## Build

```
xcodebuild -project LiDARLinkTests/LiDARLinkTests.xcodeproj -scheme LiDARLinkTests \
  -configuration Debug -destination 'platform=macOS' test

xcodebuild -project LiDARLinkMac/LiDARLinkMac.xcodeproj -scheme LiDARLinkMac \
  -configuration Debug build

xcodebuild -project LiDARLinkPhone/LiDARLinkPhone.xcodeproj -scheme LiDARLinkPhone \
  -configuration Debug -destination 'generic/platform=iOS' CODE_SIGNING_ALLOWED=NO build
```

Release builds of the Mac app must exclude x86_64 on the `xcodebuild` command
line (not in `project.yml` — package targets don't inherit it): the shared
package's wire codec uses `Float16`, unavailable when the SPM package gets
compiled for Intel.

```
xcodebuild -project LiDARLinkMac/LiDARLinkMac.xcodeproj -scheme LiDARLinkMac \
  -configuration Release -destination 'platform=macOS' \
  ONLY_ACTIVE_ARCH=YES EXCLUDED_ARCHS=x86_64 build
```

## Install

**Mac app** — build it (above) or open `LiDARLink.xcworkspace` in Xcode,
select the `LiDARLinkMac` scheme, and Run. The built `.app` runs standalone;
no install step beyond that.

**Phone app** — needs your own signing identity, since none is checked in:

1. Open `LiDARLink.xcworkspace`, select the `LiDARLinkPhone` scheme.
2. In the target's *Signing & Capabilities* tab, pick your Team (Xcode
   generates a personal provisioning profile automatically).
3. Plug in your iPhone, select it as the run destination, and Run — or from
   the CLI:

   ```
   xcodebuild -project LiDARLinkPhone/LiDARLinkPhone.xcodeproj -scheme LiDARLinkPhone \
     -destination 'id=<your-device-id>' -allowProvisioningUpdates \
     DEVELOPMENT_TEAM=<your-team-id> build

   xcrun devicectl device install app --device <your-device-id> \
     <path-to-built-LiDARLinkPhone.app>
   ```

   Find your device id with `xcrun devicectl list devices`.
4. On first launch, trust the developer certificate: **Settings → General →
   VPN & Device Management** on the phone.

## Connecting

Launch the Mac app first — it advertises itself over Bonjour
(`_lidarlink._tcp`) and starts listening. On the phone, either:

- **Wi-Fi** — pick the Mac from the discovered list once both are on the
  same network, or
- **USB** — connect the cable, then tap *Connect via USB* in the Mac app.
  Lower latency, no Wi-Fi contention, works over `usbmuxd` the same way
  Xcode's own device connection does.

Approve the incoming connection on the Mac (or turn on auto-approve), then
start scanning from either side — the Mac's Scan panel can drive the phone's
scan lifecycle and a few capture settings remotely, or just use the phone's
own controls.
