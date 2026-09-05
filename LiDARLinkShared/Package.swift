// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "LiDARLinkShared",
    platforms: [
        .iOS(.v17),
        .macOS(.v14)
    ],
    products: [
        .library(name: "LiDARLinkShared", targets: ["LiDARLinkShared"]),
        .executable(name: "reconstruct", targets: ["reconstruct"])
    ],
    targets: [
        .target(
            name: "LiDARLinkShared",
            path: "Sources/LiDARLinkShared"
        ),
        .executableTarget(
            name: "reconstruct",
            dependencies: ["LiDARLinkShared"],
            path: "Sources/reconstruct"
        )
    ]
)
