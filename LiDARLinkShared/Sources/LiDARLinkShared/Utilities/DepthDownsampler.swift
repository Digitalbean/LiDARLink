import Foundation
import simd

/// CPU depth downsampling (box average) with confidence nearest sampling.
public enum DepthDownsampler {
    /// Box-averages a Float32 depth buffer into Float16 values (meters).
    /// Invalid samples (NaN or <= 0) are ignored; a box with no valid samples
    /// yields 0 (filtered out later by point-cloud construction).
    public static func downsample(depthPointer: UnsafePointer<Float>,
                                  width: Int,
                                  height: Int,
                                  bytesPerRow: Int,
                                  targetWidth: Int,
                                  targetHeight: Int) -> [Float16] {
        precondition(targetWidth > 0 && targetHeight > 0)
        let rowStride = max(bytesPerRow / MemoryLayout<Float>.stride, width)

        // Boxes that straddle a depth edge (large spread) get the nearest valid
        // sample instead of an average, so an object silhouette does not spawn a
        // phantom surface floating halfway to the background.
        let edgeSpreadMeters: Float = 0.1
        var result = [Float16](repeating: 0, count: targetWidth * targetHeight)
        for ty in 0..<targetHeight {
            let srcY0 = ty * height / targetHeight
            let srcY1 = max((ty + 1) * height / targetHeight, srcY0 + 1)
            for tx in 0..<targetWidth {
                let srcX0 = tx * width / targetWidth
                let srcX1 = max((tx + 1) * width / targetWidth, srcX0 + 1)
                var sum: Float = 0
                var count = 0
                var minValue = Float.greatestFiniteMagnitude
                var maxValue: Float = 0
                var sy = srcY0
                while sy < srcY1 {
                    let row = depthPointer.advanced(by: sy * rowStride)
                    var sx = srcX0
                    while sx < srcX1 {
                        let value = row[sx]
                        if value.isFinite && value > 0 {
                            sum += value
                            count += 1
                            minValue = min(minValue, value)
                            maxValue = max(maxValue, value)
                        }
                        sx += 1
                    }
                    sy += 1
                }
                if count > 0 {
                    let value = (maxValue - minValue) > edgeSpreadMeters ? minValue : sum / Float(count)
                    result[ty * targetWidth + tx] = Float16(value)
                }
            }
        }
        return result
    }

    /// Nearest-samples a UInt8 confidence map to the target resolution.
    public static func downsampleConfidence(pointer: UnsafePointer<UInt8>,
                                            width: Int,
                                            height: Int,
                                            bytesPerRow: Int,
                                            targetWidth: Int,
                                            targetHeight: Int) -> [UInt8] {
        precondition(targetWidth > 0 && targetHeight > 0)
        let rowStride = max(bytesPerRow, width)
        var result = [UInt8](repeating: 0, count: targetWidth * targetHeight)
        for ty in 0..<targetHeight {
            let sy = min(ty * height / targetHeight, height - 1)
            let row = pointer.advanced(by: sy * rowStride)
            for tx in 0..<targetWidth {
                let sx = min(tx * width / targetWidth, width - 1)
                result[ty * targetWidth + tx] = row[sx]
            }
        }
        return result
    }

    /// Array convenience for tests and CPU-side buffers (dense, no row padding).
    public static func downsample(depth: [Float], width: Int, height: Int, targetWidth: Int, targetHeight: Int) -> [Float16] {
        precondition(depth.count >= width * height)
        return depth.withUnsafeBufferPointer { buffer in
            downsample(depthPointer: buffer.baseAddress!,
                       width: width,
                       height: height,
                       bytesPerRow: width * MemoryLayout<Float>.stride,
                       targetWidth: targetWidth,
                       targetHeight: targetHeight)
        }
    }

    public static func downsampleConfidence(confidence: [UInt8], width: Int, height: Int, targetWidth: Int, targetHeight: Int) -> [UInt8] {
        precondition(confidence.count >= width * height)
        return confidence.withUnsafeBufferPointer { buffer in
            downsampleConfidence(pointer: buffer.baseAddress!,
                                 width: width,
                                 height: height,
                                 bytesPerRow: width,
                                 targetWidth: targetWidth,
                                 targetHeight: targetHeight)
        }
    }
}
