import Foundation
import simd

/// Builds the sequence of wire messages that represent one captured frame.
public enum WireFrameBuilder {
    public static func frameMessages(sequence: UInt32,
                                     captureTimestampMs: UInt64,
                                     pose: simd_float4x4,
                                     intrinsics: CameraIntrinsics,
                                     depth: DepthPayload?,
                                     color: ColorPayload?,
                                     depthIsSmoothed: Bool,
                                     depthScale: Float,
                                     downsampleFactor: Int,
                                     meshChunks: [MeshChunk],
                                     removedMeshAnchorIDs: [UUID] = []) -> [Message] {
        var messages: [Message] = []

        // Depth pixels are unprojected on the receiver, so the intrinsics must be
        // expressed in depth-map coordinates, not camera-image coordinates.
        let metadataIntrinsics: CameraIntrinsics
        if let depth {
            metadataIntrinsics = intrinsics.scaled(to: depth.width, height: depth.height)
        } else {
            metadataIntrinsics = intrinsics
        }

        let metadata = FrameMetadata(sequence: sequence,
                                     captureTimestampMs: captureTimestampMs,
                                     depthWidth: depth?.width ?? 0,
                                     depthHeight: depth?.height ?? 0,
                                     depthScale: depthScale,
                                     hasConfidence: depth?.confidenceData != nil,
                                     depthIsSmoothed: depthIsSmoothed,
                                     intrinsics: metadataIntrinsics,
                                     pose: pose,
                                     colorWidth: color?.width ?? 0,
                                     colorHeight: color?.height ?? 0,
                                     colorJpegSize: color?.jpegData.count ?? 0,
                                     downsampleFactor: downsampleFactor)
        messages.append(.frameMeta(metadata))

        if let depth { messages.append(.frameDepth(depth)) }
        if let color { messages.append(.frameColor(color)) }
        if !removedMeshAnchorIDs.isEmpty {
            messages.append(.meshRemove(MeshRemoveMessage(anchorIDs: removedMeshAnchorIDs)))
        }
        for chunk in meshChunks {
            messages.append(.meshMeta(chunk.metadata))
            messages.append(.meshData(chunk))
        }
        return messages
    }
}
