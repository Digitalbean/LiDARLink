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

## Build

```
xcodebuild -project LiDARLinkTests/LiDARLinkTests.xcodeproj -scheme LiDARLinkTests \
  -configuration Debug -destination 'platform=macOS' test

xcodebuild -project LiDARLinkMac/LiDARLinkMac.xcodeproj -scheme LiDARLinkMac \
  -configuration Debug build

xcodebuild -project LiDARLinkPhone/LiDARLinkPhone.xcodeproj -scheme LiDARLinkPhone \
  -configuration Debug -destination 'generic/platform=iOS' CODE_SIGNING_ALLOWED=NO build
```
