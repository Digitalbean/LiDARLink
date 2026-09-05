import AppKit
import SceneKit
import simd
import LiDARLinkShared

/// SceneKit scene that renders the live point cloud and reconstructed mesh with a
/// first-person walk camera (WASD + free mouse-look, optionally floor-locked).
///
/// The point cloud is rendered in fixed-size chunks: each frame's new points are
/// packed into new chunk nodes that are appended once and never rebuilt, so the
/// render cost stays bounded and nothing flickers as the cloud grows.
final class PointCloudSceneController {
    let scene = SCNScene()
    let pointMaterial: SCNMaterial
    /// Holds the point cloud and mesh so a single node scale can mirror the
    /// whole reconstruction (the grid/axes stay put as a world reference).
    private let contentRoot = SCNNode()
    private let pointRoot = SCNNode()
    private let surfaceNode = SCNNode()   // TSDF marching-cubes surface
    private let measureRoot = SCNNode()
    private let cameraNode = SCNNode()
    private let overlayRoot = SCNNode()
    private let roomRoot = SCNNode()      // RoomPlan parametric overlay
    private let objectRoot = SCNNode()    // photogrammetry USDZ result
    /// One node per ARMeshAnchor so each mesh piece keeps its own geometry and
    /// transform instead of replacing the whole display (which made the mesh
    /// jump between anchors).
    private var meshNodes: [UUID: (node: SCNNode, insertedAt: Date)] = [:]

    /// Maximum points per display chunk. New points are appended as new chunks.
    static let chunkSize = 50_000

    private(set) var pointColorMode: PointColorMode = .cameraColor
    private(set) var appendedPointCount = 0
    private var latestPoints: [PointCloudPoint] = []

    /// Screen-space eye-dome lighting. `.technique` can only live on the actual
    /// `SCNView`, not the scene, and (unlike everything else this controller
    /// touches) SwiftUI does not reliably re-invoke `updateNSView` just because
    /// an unrelated `@Published` flipped — so this needs the same imperative
    /// callback pattern as `onFollowDeviceChanged` rather than a passively-read
    /// property. Off by default — first custom render pass in the app, see
    /// `EyeDomeLighting.metal`.
    private(set) var eyeDomeLightingEnabled = false
    var onEyeDomeLightingChanged: ((SCNTechnique?) -> Void)?

    func setEyeDomeLightingEnabled(_ enabled: Bool) {
        eyeDomeLightingEnabled = enabled
        onEyeDomeLightingChanged?(enabled ? Self.eyeDomeLightingTechnique : nil)
    }

    /// Built once; parses `EyeDomeLighting.metal`'s two functions into a
    /// two-stage-free single quad pass reading SceneKit's own COLOR/DEPTH.
    static let eyeDomeLightingTechnique: SCNTechnique? = {
        let dictionary: [String: Any] = [
            "sequence": ["edl"],
            "passes": [
                "edl": [
                    "draw": "DRAW_QUAD",
                    "metalVertexShader": "edl_vertex",
                    "metalFragmentShader": "edl_fragment",
                    "inputs": ["colorSampler": "COLOR", "depthSampler": "DEPTH"],
                    "outputs": ["color": "COLOR"]
                ]
            ]
        ]
        guard let technique = SCNTechnique(dictionary: dictionary) else {
            Log.warning("Eye-dome lighting technique failed to parse", category: "render")
            return nil
        }
        return technique
    }()

    // MARK: Walk (first-person) camera
    private var walkPosition = SIMD3<Float>(0, 1.6, 2.5)
    private var walkYaw: Float = .pi          // face toward -Z (the scan sits ahead)
    private var walkPitch: Float = 0
    private var walkInput = SIMD3<Float>(0, 0, 0)   // x = strafe, y = lift, z = forward
    private var walkSprint = false
    private var floorLocked = true
    private var eyeHeight: Float = 1.6
    private var floorY: Float?

    /// When set, the view camera rides the live iPhone pose instead of the
    /// walker — you see the reconstruction from exactly where the phone is.
    private(set) var followsDevice = false
    private var lastDevicePose: simd_float4x4?
    var onFollowDeviceChanged: ((Bool) -> Void)?
    /// Base walk speed in metres per second (Shift multiplies it).
    var walkSpeed: Float = 1.6
    private var meshWireframe = false

    init() {
        scene.background.contents = NSColor(calibratedWhite: 0.07, alpha: 1)

        let material = SCNMaterial()
        // Unlit: points render their exact vertex color (lit shading washes
        // point colors out to gray).
        material.lightingModel = .constant
        material.shaderModifiers = [
            .geometry: """
            #pragma arguments
            float u_pointSize;
            #pragma transparent
            #pragma body
            // Fade out points within ~0.35 m of the eye: when you walk into the
            // cloud, the specks right in front of your face stop forming an
            // opaque near wall you can't see past.
            float4 _vp = scn_node.modelViewTransform * _geometry.position;
            float _eyeDist = length(_vp.xyz);
            _geometry.pointSize = u_pointSize * smoothstep(0.08, 0.35, _eyeDist);
            """
        ]
        material.setValue(2.0, forKey: "u_pointSize")
        // #pragma transparent above (for the near-eye fade) puts this material
        // in SceneKit's transparent render category, which by default skips
        // depth *writes* (it still depth-tests) — so nothing about the point
        // cloud was ever reaching the depth buffer. Harmless for normal
        // rendering (points still occlude correctly against opaque geometry
        // either way) but it meant eye-dome lighting's depth read was always
        // empty. Force writes back on.
        material.writesToDepthBuffer = true
        pointMaterial = material

        cameraNode.camera = SCNCamera()
        // zNear at 0.001 with zFar 1000 crushes depth precision and z-fights the
        // grid/mesh; 0.02 m is closer than anyone walks into a surface.
        cameraNode.camera?.zNear = 0.02
        cameraNode.camera?.zFar = 1000
        cameraNode.camera?.fieldOfView = 60
        contentRoot.addChildNode(pointRoot)
        contentRoot.addChildNode(surfaceNode)
        contentRoot.addChildNode(measureRoot)
        contentRoot.addChildNode(roomRoot)
        contentRoot.addChildNode(objectRoot)
        scene.rootNode.addChildNode(contentRoot)
        scene.rootNode.addChildNode(overlayRoot)
        scene.rootNode.addChildNode(cameraNode)
        installWorldOverlay()
        applyWalkCamera()
    }

    /// Current camera position in world space (for view-dependent shading).
    var cameraWorldPosition: SIMD3<Float> { cameraNode.simdPosition }

    // MARK: Measurement

    private var measuring = false
    private var measurePoints: [SIMD3<Float>] = []
    /// Reports the distance between the two placed points, or nil when cleared.
    var onMeasurementDistance: ((Float?) -> Void)?

    var isMeasuring: Bool { measuring }

    func setMeasuring(_ on: Bool) {
        measuring = on
        if !on { clearMeasurement() }
    }

    func clearMeasurement() {
        measurePoints.removeAll()
        measureRoot.childNodes.forEach { $0.removeFromParentNode() }
        onMeasurementDistance?(nil)
    }

    /// Places a measurement point under the given viewport click (origin
    /// top-left). After two points, draws the segment and reports its length.
    func placeMeasurePoint(viewportPoint: CGPoint, viewportSize: CGSize) {
        guard measuring, !latestPoints.isEmpty else { return }
        let forward = ViewNavigationMath.walkLookDirection(yaw: walkYaw, pitch: walkPitch)
        let ray = ViewNavigationMath.viewRay(cameraPosition: cameraNode.simdPosition,
                                             cameraForward: forward,
                                             cameraUp: SIMD3<Float>(0, 1, 0),
                                             viewportPoint: viewportPoint,
                                             viewportSize: viewportSize,
                                             fieldOfViewY: Float(cameraNode.camera?.fieldOfView ?? 60))
        guard let hit = ViewNavigationMath.nearestPoint(to: ray.origin,
                                                        direction: ray.direction,
                                                        in: latestPoints,
                                                        maxDistance: 0.2) else { return }
        if measurePoints.count >= 2 { clearMeasurement() }
        measurePoints.append(hit.position)
        addMeasureMarker(at: hit.position)
        if measurePoints.count == 2 {
            addMeasureSegment(from: measurePoints[0], to: measurePoints[1])
            onMeasurementDistance?(simd_length(measurePoints[1] - measurePoints[0]))
        }
    }

    private func addMeasureMarker(at position: SIMD3<Float>) {
        let sphere = SCNSphere(radius: 0.015)
        sphere.firstMaterial?.lightingModel = .constant
        sphere.firstMaterial?.diffuse.contents = NSColor.systemYellow
        let node = SCNNode(geometry: sphere)
        node.simdPosition = position
        measureRoot.addChildNode(node)
    }

    private func addMeasureSegment(from a: SIMD3<Float>, to b: SIMD3<Float>) {
        let length = simd_length(b - a)
        guard length > 0.0001 else { return }
        let cylinder = SCNCylinder(radius: 0.004, height: CGFloat(length))
        cylinder.firstMaterial?.lightingModel = .constant
        cylinder.firstMaterial?.diffuse.contents = NSColor.systemYellow
        let node = SCNNode(geometry: cylinder)
        node.simdPosition = (a + b) * 0.5
        // SCNCylinder runs along local +Y; rotate +Y onto the segment direction.
        let dir = simd_normalize(b - a)
        let up = SIMD3<Float>(0, 1, 0)
        let dot = simd_clamp(simd_dot(up, dir), -1, 1)
        if dot < 0.99999 {
            let axis = simd_normalize(simd_cross(up, dir))
            node.simdOrientation = simd_quatf(angle: acos(dot), axis: axis)
        }
        measureRoot.addChildNode(node)
    }

    // MARK: Content

    /// Appends display geometry for the NEW points. Call with chunks produced by
    /// `makeChunkGeometries` (built off the main thread).
    func appendPointChunks(_ newChunks: [(geometry: SCNGeometry, count: Int)]) {
        for chunk in newChunks {
            let node = SCNNode(geometry: chunk.geometry)
            pointRoot.addChildNode(node)
            appendedPointCount += chunk.count
        }
    }

    /// Replaces all point chunks (used when the color mode changes).
    func installPointChunks(_ chunks: [(geometry: SCNGeometry, count: Int)]) {
        pointRoot.childNodes.forEach { $0.removeFromParentNode() }
        appendedPointCount = 0
        appendPointChunks(chunks)
    }

    // MARK: Reconstructed surface (TSDF)

    // MARK: RoomPlan overlay

    func setRoomStructureVisible(_ visible: Bool) { roomRoot.isHidden = !visible }

    /// Loads a photogrammetry USDZ result into the scene.
    func installObjectModel(_ url: URL) {
        objectRoot.childNodes.forEach { $0.removeFromParentNode() }
        guard let loaded = try? SCNScene(url: url, options: [.checkConsistency: false]) else { return }
        for child in loaded.rootNode.childNodes { objectRoot.addChildNode(child) }
        let (minB, maxB) = objectRoot.boundingBox
        let c = SIMD3<Float>(Float(minB.x + maxB.x) / 2, Float(minB.y + maxB.y) / 2, Float(minB.z + maxB.z) / 2)
        recenter(to: [PointCloudPoint(position: c, color: .zero)])
    }

    func clearObjectModel() { objectRoot.childNodes.forEach { $0.removeFromParentNode() } }

    /// Centroid-fan triangulation of a world-space surface outline.
    private static func makePolygonGeometry(_ points: [SIMD3<Float>], color: NSColor) -> SCNGeometry {
        guard points.count >= 3 else { return SCNGeometry() }
        var centroid = SIMD3<Float>.zero
        for p in points { centroid += p }
        centroid /= Float(points.count)
        let n = simd_normalize(simd_cross(points[1] - points[0], points[2] - points[0]))

        var verts: [SCNVector3] = [SCNVector3(centroid.x, centroid.y, centroid.z)]
        verts += points.map { SCNVector3($0.x, $0.y, $0.z) }
        let normals = [SCNVector3](repeating: SCNVector3(n.x, n.y, n.z), count: verts.count)
        var idx: [Int32] = []
        let count = Int32(points.count)
        for i in 0..<count {
            idx += [0, i + 1, (i + 1) % count + 1]
        }
        let src = SCNGeometrySource(vertices: verts)
        let nrm = SCNGeometrySource(normals: normals)
        let elem = SCNGeometryElement(indices: idx, primitiveType: .triangles)
        let geo = SCNGeometry(sources: [src, nrm], elements: [elem])
        let m = SCNMaterial()
        m.diffuse.contents = color
        m.isDoubleSided = true
        m.lightingModel = .constant
        m.writesToDepthBuffer = false
        m.blendMode = .alpha
        geo.materials = [m]
        return geo
    }

    private static func makePolylineGeometry(_ points: [SIMD3<Float>], color: NSColor) -> SCNGeometry {
        guard points.count >= 2 else { return SCNGeometry() }
        let verts = points.map { SCNVector3($0.x, $0.y, $0.z) }
        var idx: [Int32] = []
        for i in 0..<Int32(points.count) { idx += [i, (i + 1) % Int32(points.count)] }
        let src = SCNGeometrySource(vertices: verts)
        let elem = SCNGeometryElement(indices: idx, primitiveType: .line)
        let geo = SCNGeometry(sources: [src], elements: [elem])
        let m = SCNMaterial()
        m.diffuse.contents = color
        m.lightingModel = .constant
        geo.materials = [m]
        return geo
    }

    /// Draws the parametric room: walls as translucent panels, openings as
    /// bright frames, objects as wireframe boxes. `correction` registers
    /// RoomPlan's ARKit-frame geometry with the pose-graph-corrected surfels.
    func installRoomStructure(_ room: RoomStructure, correction: simd_float4x4 = matrix_identity_float4x4) {
        roomRoot.childNodes.forEach { $0.removeFromParentNode() }
        roomRoot.simdTransform = correction

        for surface in room.surfaces {
            let d = surface.dimensions
            let node = SCNNode()
            switch surface.category {
            case .wall, .floor, .ceiling:
                // The real outline — RoomPlan's polygon for non-rectangular /
                // curved walls, else the four rectangle corners.
                let fill = NSColor(calibratedRed: 0.35, green: 0.65, blue: 1.0, alpha: 0.14)
                node.geometry = Self.makePolygonGeometry(surface.outline, color: fill)
                // Outline the edges so a bay window / alcove reads clearly.
                let edge = SCNNode(geometry: Self.makePolylineGeometry(surface.outline,
                                   color: NSColor(calibratedRed: 0.4, green: 0.7, blue: 1.0, alpha: 0.7)))
                node.addChildNode(edge)
            case .door, .window, .opening:
                let frame = SCNBox(width: CGFloat(max(d.x, 0.02)), height: CGFloat(max(d.y, 0.02)),
                                   length: 0.03, chamferRadius: 0)
                let m = SCNMaterial()
                m.diffuse.contents = surface.category == .window
                    ? NSColor.systemTeal : NSColor.systemOrange
                m.fillMode = .lines
                m.lightingModel = .constant
                frame.materials = [m]
                node.geometry = frame
                node.simdTransform = surface.transform
            }
            roomRoot.addChildNode(node)
        }

        for object in room.objects {
            let d = object.dimensions
            let box = SCNBox(width: CGFloat(max(d.x, 0.02)), height: CGFloat(max(d.y, 0.02)),
                             length: CGFloat(max(d.z, 0.02)), chamferRadius: 0)
            let m = SCNMaterial()
            m.diffuse.contents = NSColor(calibratedWhite: 0.9, alpha: 0.9)
            m.fillMode = .lines
            m.lightingModel = .constant
            box.materials = [m]
            let node = SCNNode(geometry: box)
            node.simdTransform = object.transform
            roomRoot.addChildNode(node)
        }
    }

    func clearReconstructedSurface() {
        surfaceNode.geometry = nil
    }

    /// Installs the marching-cubes surface as a shaded, per-vertex-coloured mesh.
    func installReconstructedSurface(vertices: [SIMD3<Float>],
                                     normals: [SIMD3<Float>],
                                     colours: [SIMD3<Float>],
                                     faces: [UInt32],
                                     wireframe: Bool) {
        guard vertices.count == normals.count, faces.count >= 3 else {
            surfaceNode.geometry = nil
            return
        }
        let positionSource = SCNGeometrySource(vertices: vertices.map { SCNVector3($0.x, $0.y, $0.z) })
        let normalSource = SCNGeometrySource(normals: normals.map { SCNVector3($0.x, $0.y, $0.z) })

        var colourComponents = [Float]()
        colourComponents.reserveCapacity(vertices.count * 3)
        for c in colours {
            colourComponents.append(c.x / 255); colourComponents.append(c.y / 255); colourComponents.append(c.z / 255)
        }
        if colours.count != vertices.count {
            colourComponents = [Float](repeating: 0.8, count: vertices.count * 3)
        }
        let colourData = colourComponents.withUnsafeBytes { Data($0) }
        let colourSource = SCNGeometrySource(data: colourData, semantic: .color,
                                             vectorCount: vertices.count, usesFloatComponents: true,
                                             componentsPerVector: 3, bytesPerComponent: MemoryLayout<Float>.size,
                                             dataOffset: 0, dataStride: MemoryLayout<Float>.size * 3)

        var indexData = Data(capacity: faces.count * 4)
        for f in faces { withUnsafeBytes(of: f.littleEndian) { indexData.append(contentsOf: $0) } }
        let element = SCNGeometryElement(data: indexData, primitiveType: .triangles,
                                         primitiveCount: faces.count / 3, bytesPerIndex: 4)

        let geometry = SCNGeometry(sources: [positionSource, normalSource, colourSource], elements: [element])
        let material = SCNMaterial()
        material.lightingModel = .blinn
        material.diffuse.contents = NSColor.white   // vertex colours modulate this
        material.isDoubleSided = true
        material.fillMode = wireframe ? .lines : .fill
        geometry.materials = [material]
        surfaceNode.geometry = geometry
    }

    func setLatestPoints(_ points: [PointCloudPoint]) {
        latestPoints = points
    }

    /// Builds point geometry off the main thread. `points` should be the new
    /// points since the last append (or the full cloud for a full rebuild).
    static func makeChunkGeometries(points: [PointCloudPoint],
                                    material: SCNMaterial,
                                    colorFor: (PointCloudPoint) -> SIMD3<UInt8>) -> [(geometry: SCNGeometry, count: Int)] {
        guard !points.isEmpty else { return [] }
        var chunks: [(SCNGeometry, Int)] = []
        var index = 0
        while index < points.count {
            let end = min(index + chunkSize, points.count)
            let slice = Array(points[index..<end])
            chunks.append((makePointGeometry(points: slice, material: material, colorFor: colorFor), slice.count))
            index = end
        }
        return chunks
    }

    static func makePointGeometry(points: [PointCloudPoint],
                                  material: SCNMaterial,
                                  colorFor: (PointCloudPoint) -> SIMD3<UInt8>) -> SCNGeometry {
        var positions: [SCNVector3] = []
        var colorComponents: [Float] = []
        positions.reserveCapacity(points.count)
        colorComponents.reserveCapacity(points.count * 3)
        for point in points {
            positions.append(SCNVector3(point.position.x, point.position.y, point.position.z))
            let color = colorFor(point)
            colorComponents.append(Float(color.x) / 255)
            colorComponents.append(Float(color.y) / 255)
            colorComponents.append(Float(color.z) / 255)
        }
        let positionSource = SCNGeometrySource(vertices: positions)
        let colorData = colorComponents.withUnsafeBytes { Data($0) }
        let colorSource = SCNGeometrySource(data: colorData,
                                            semantic: .color,
                                            vectorCount: points.count,
                                            usesFloatComponents: true,
                                            componentsPerVector: 3,
                                            bytesPerComponent: MemoryLayout<Float>.size,
                                            dataOffset: 0,
                                            dataStride: MemoryLayout<Float>.size * 3)
        let element = SCNGeometryElement(data: nil, primitiveType: .point, primitiveCount: points.count, bytesPerIndex: 0)
        // Without an explicit screen-space radius range SceneKit clamps points to
        // ~1 px regardless of the shader's pointSize.
        element.pointSize = pointSpriteSize
        element.minimumPointScreenSpaceRadius = 1
        element.maximumPointScreenSpaceRadius = CGFloat(max(1, pointSpriteSize))
        let geometry = SCNGeometry(sources: [positionSource, colorSource], elements: [element])
        geometry.materials = [material]
        return geometry
    }

    func updateMesh(_ mesh: MeshData) {
        let entry: (node: SCNNode, insertedAt: Date)
        if let existing = meshNodes[mesh.anchorID] {
            entry = existing
        } else {
            let node = SCNNode()
            contentRoot.addChildNode(node)
            entry = (node, Date())
            meshNodes[mesh.anchorID] = entry
            pruneMeshNodes()
        }
        entry.node.geometry = Self.makeMeshGeometry(mesh: mesh, wireframe: meshWireframe)
        entry.node.simdTransform = mesh.transform
        entry.node.isHidden = !meshVisible
    }

    private var meshVisible = true

    func setMeshVisible(_ visible: Bool) {
        meshVisible = visible
        for entry in meshNodes.values {
            entry.node.isHidden = !visible
        }
    }

    /// Toggles the reconstructed mesh between solid shading and a wireframe.
    func setMeshWireframe(_ wireframe: Bool) {
        meshWireframe = wireframe
        for entry in meshNodes.values {
            entry.node.geometry?.materials.forEach { $0.fillMode = wireframe ? .lines : .fill }
        }
        surfaceNode.geometry?.materials.forEach { $0.fillMode = wireframe ? .lines : .fill }
    }

    /// Drops the geometry for an anchor ARKit removed.
    func removeMesh(_ anchorID: UUID) {
        meshNodes[anchorID]?.node.removeFromParentNode()
        meshNodes.removeValue(forKey: anchorID)
    }

    /// Safety cap only — anchor removal is now explicit, so this rarely fires.
    private static let meshNodeCap = 1024

    private func pruneMeshNodes() {
        guard meshNodes.count > Self.meshNodeCap else { return }
        let sorted = meshNodes.sorted { $0.value.insertedAt < $1.value.insertedAt }
        let excess = sorted.prefix(meshNodes.count - Self.meshNodeCap)
        for (id, entry) in excess {
            entry.node.removeFromParentNode()
            meshNodes.removeValue(forKey: id)
        }
    }

    /// Screen-space point size, shared with the static geometry builders so the
    /// element's screen-space radius cap matches the shader.
    static var pointSpriteSize: CGFloat = 2

    func setPointSize(_ size: Float) {
        pointMaterial.setValue(size, forKey: "u_pointSize")
        Self.pointSpriteSize = CGFloat(size)
    }

    func setBackground(_ color: NSColor) {
        scene.background.contents = color
    }

    func setWorldOverlayVisible(_ visible: Bool) {
        overlayRoot.isHidden = !visible
    }

    func clear() {
        installPointChunks([])
        latestPoints.removeAll(keepingCapacity: true)
        clearMeasurement()
        clearReconstructedSurface()
        roomRoot.childNodes.forEach { $0.removeFromParentNode() }
        for entry in meshNodes.values {
            entry.node.removeFromParentNode()
        }
        meshNodes.removeAll()
    }

    // MARK: Walk camera

    /// Re-drops the walker so it looks into the scan from one end. Bound to the
    /// "Recenter" button and used when the first points arrive.
    func recenter(to points: [PointCloudPoint]) {
        guard !points.isEmpty else { return }
        leaveFollowMode()
        let stand = ViewNavigationMath.walkStandpoint(for: points,
                                                      eyeHeight: eyeHeight,
                                                      floorY: floorY)
        walkPosition = stand.position
        walkYaw = stand.yaw
        walkPitch = 0
        applyWalkCamera()
    }

    /// Re-drops the walker using the most recent display points.
    func recenterToLatest() {
        recenter(to: latestPoints)
    }

    /// Resets the walker: into the scan if there is one, else a default outside.
    func resetView() {
        leaveFollowMode()
        if !latestPoints.isEmpty {
            recenter(to: latestPoints)
        } else {
            walkPosition = SIMD3<Float>(0, (floorY ?? 0) + eyeHeight, 2.5)
            walkYaw = .pi
            walkPitch = 0
            applyWalkCamera()
        }
    }

    func setFloorLocked(_ locked: Bool) {
        floorLocked = locked
        if locked, let floorY {
            walkPosition.y = floorY + eyeHeight
            applyWalkCamera()
        }
    }

    func setEyeHeight(_ metres: Float) {
        eyeHeight = max(0.1, metres)
        if floorLocked, let floorY {
            walkPosition.y = floorY + eyeHeight
            applyWalkCamera()
        }
    }

    /// Free mouse-look: raw pointer deltas turn the head in place.
    func walkLook(deltaX: Float, deltaY: Float) {
        leaveFollowMode()
        walkYaw -= deltaX * 0.004
        walkPitch = min(max(walkPitch - deltaY * 0.004, -ViewNavigationMath.walkPitchLimit), ViewNavigationMath.walkPitchLimit)
        applyWalkCamera()
    }

    /// Held-key movement state (each component in [-1, 1]); `sprint` doubles speed.
    func setWalkInput(strafe: Float, forward: Float, lift: Float, sprint: Bool) {
        if followsDevice, (strafe != 0 || forward != 0 || lift != 0) { leaveFollowMode() }
        walkInput = SIMD3<Float>(strafe, lift, forward)
        walkSprint = sprint
    }

    /// Called once per rendered frame by the view; advances the walker.
    func stepWalk(deltaSeconds: Float) {
        guard walkInput != .zero else { return }
        let speed = walkSpeed * (walkSprint ? 3 : 1)
        let distance = speed * min(max(deltaSeconds, 0), 0.1)
        walkPosition = ViewNavigationMath.walkStep(position: walkPosition,
                                                   yaw: walkYaw,
                                                   forward: walkInput.z,
                                                   strafe: walkInput.x,
                                                   lift: walkInput.y,
                                                   distance: distance,
                                                   floorY: floorLocked ? floorY : nil,
                                                   eyeHeight: eyeHeight)
        applyWalkCamera()
    }

    /// Sets the estimated floor height (computed off the main thread by the
    /// display rebuild).
    func setFloorY(_ y: Float) {
        floorY = y
        if floorLocked {
            walkPosition.y = y + eyeHeight
            applyWalkCamera()
        }
    }

    // MARK: Follow-iPhone camera

    /// Ride the live device pose — the phone's position and look direction, but
    /// with a roll-free basis so the horizon stays level regardless of how the
    /// phone is held (the wire pose carries no interface orientation).
    func setDevicePose(_ pose: simd_float4x4, verticalFOV: Float = 60) {
        lastDevicePose = pose
        guard followsDevice else { return }
        cameraNode.simdTransform = Self.levelCameraTransform(from: pose)
        cameraNode.camera?.fieldOfView = CGFloat(max(20, min(verticalFOV, 120)))
        cameraNode.camera?.projectionDirection = .vertical
    }

    static func levelCameraTransform(from pose: simd_float4x4) -> simd_float4x4 {
        let position = SIMD3<Float>(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
        var forward = -SIMD3<Float>(pose.columns.2.x, pose.columns.2.y, pose.columns.2.z)
        if simd_length(forward) < 1e-5 { forward = SIMD3<Float>(0, 0, -1) }
        forward = simd_normalize(forward)
        let worldUp = SIMD3<Float>(0, 1, 0)
        var right = simd_cross(forward, worldUp)
        if simd_length(right) < 1e-3 {                    // looking straight up/down
            right = SIMD3<Float>(pose.columns.0.x, pose.columns.0.y, pose.columns.0.z)
        }
        right = simd_normalize(right)
        let up = simd_normalize(simd_cross(right, forward))
        var t = matrix_identity_float4x4
        t.columns.0 = SIMD4<Float>(right, 0)
        t.columns.1 = SIMD4<Float>(up, 0)
        t.columns.2 = SIMD4<Float>(-forward, 0)
        t.columns.3 = SIMD4<Float>(position, 1)
        return t
    }

    func setFollowsDevice(_ on: Bool) {
        guard followsDevice != on else { return }
        followsDevice = on
        if on {
            if let pose = lastDevicePose { cameraNode.simdTransform = pose }
        } else {
            cameraNode.camera?.fieldOfView = 60
            cameraNode.camera?.projectionDirection = .horizontal
            // Hand control back to the walker at the current viewpoint.
            let t = cameraNode.simdTransform
            walkPosition = SIMD3<Float>(t.columns.3.x, t.columns.3.y, t.columns.3.z)
            let fwd = -SIMD3<Float>(t.columns.2.x, t.columns.2.y, t.columns.2.z)
            walkYaw = atan2(-fwd.x, -fwd.z)
            walkPitch = asin(simd_clamp(fwd.y, -1, 1))
            applyWalkCamera()
        }
        onFollowDeviceChanged?(on)
    }

    private func leaveFollowMode() {
        if followsDevice { setFollowsDevice(false) }
    }

    private func applyWalkCamera() {
        let forward = ViewNavigationMath.walkLookDirection(yaw: walkYaw, pitch: walkPitch)
        // Roll-free camera basis: right is horizontal (forward x world-up), up is
        // recomputed orthogonal. SceneKit camera space is +X right, +Y up, +Z
        // back, so the local +Z column is -forward.
        let worldUp = SIMD3<Float>(0, 1, 0)
        let right = simd_normalize(simd_cross(forward, worldUp))
        let up = simd_normalize(simd_cross(right, forward))
        var transform = matrix_identity_float4x4
        transform.columns.0 = SIMD4<Float>(right, 0)
        transform.columns.1 = SIMD4<Float>(up, 0)
        transform.columns.2 = SIMD4<Float>(-forward, 0)
        transform.columns.3 = SIMD4<Float>(walkPosition, 1)
        cameraNode.simdTransform = transform
    }

    // MARK: World overlay (axes + grid)

    private func installWorldOverlay() {
        let extent: Float = 1.0
        let axes: [(axis: SIMD3<Float>, color: NSColor)] = [
            (SIMD3<Float>(1, 0, 0), NSColor(calibratedRed: 1.0, green: 0.25, blue: 0.25, alpha: 1)),
            (SIMD3<Float>(0, 1, 0), NSColor(calibratedRed: 0.25, green: 1.0, blue: 0.35, alpha: 1)),
            (SIMD3<Float>(0, 0, 1), NSColor(calibratedRed: 0.3, green: 0.6, blue: 1.0, alpha: 1))
        ]
        for item in axes {
            let axisNode = SCNNode(geometry: Self.thinBox(length: CGFloat(extent), color: item.color))
            // `thinBox` runs along its local Z; rotate that onto the target axis
            // so each bar actually lies along X / Y / Z from the origin.
            if item.axis.x == 1 {
                axisNode.eulerAngles = SCNVector3(0, Float.pi / 2, 0)
            } else if item.axis.y == 1 {
                axisNode.eulerAngles = SCNVector3(-Float.pi / 2, 0, 0)
            }
            axisNode.position = SCNVector3(item.axis.x * extent / 2, item.axis.y * extent / 2, item.axis.z * extent / 2)
            overlayRoot.addChildNode(axisNode)
        }
        let gridColor = NSColor(calibratedWhite: 1, alpha: 0.12)
        let step: Float = 0.25
        var value: Float = -1
        while value <= 1.001 {
            // Line parallel to X at z = value (rotate the local-Z box onto X).
            let lineX = SCNNode(geometry: Self.thinBox(length: CGFloat(extent * 2), color: gridColor))
            lineX.eulerAngles = SCNVector3(0, Float.pi / 2, 0)
            lineX.position = SCNVector3(0, 0, value)
            // Line parallel to Z at x = value (box already runs along Z).
            let lineZ = SCNNode(geometry: Self.thinBox(length: CGFloat(extent * 2), color: gridColor))
            lineZ.position = SCNVector3(value, 0, 0)
            overlayRoot.addChildNode(lineX)
            overlayRoot.addChildNode(lineZ)
            value += step
        }
    }

    private static func thinBox(length: CGFloat, color: NSColor) -> SCNGeometry {
        let geometry = SCNBox(width: 0.008, height: 0.008, length: length, chamferRadius: 0)
        let material = SCNMaterial()
        material.lightingModel = .constant
        material.diffuse.contents = color
        geometry.materials = [material]
        return geometry
    }

    // MARK: Mesh geometry

    /// Builds mesh geometry defensively: out-of-range/degenerate faces are
    /// dropped, normals are padded, and oversized meshes are skipped so corrupt
    /// streams cannot crash SceneKit. Returns nil when there is nothing to draw.
    static func makeMeshGeometry(mesh: MeshData, wireframe: Bool = false) -> SCNGeometry? {
        let sanitized = MeshSanitizer.sanitize(vertices: mesh.vertices, normals: mesh.normals, faces: mesh.faces)
        guard !sanitized.vertices.isEmpty, sanitized.faces.count >= 3 else { return nil }
        if sanitized.vertices.count > 500_000 {
            Log.error("Skipping oversized mesh (\(sanitized.vertices.count) vertices)", category: "renderer")
            return nil
        }
        let vertices = sanitized.vertices.map { SCNVector3($0.x, $0.y, $0.z) }
        let normals = sanitized.normals.map { SCNVector3($0.x, $0.y, $0.z) }
        let positionSource = SCNGeometrySource(vertices: vertices)
        let normalSource = SCNGeometrySource(normals: normals)
        var indexData = Data(capacity: sanitized.faces.count * 4)
        for face in sanitized.faces {
            Binary.append(face, to: &indexData)
        }
        let element = SCNGeometryElement(data: indexData, primitiveType: .triangles, primitiveCount: sanitized.faces.count / 3, bytesPerIndex: 4)
        let geometry = SCNGeometry(sources: [positionSource, normalSource], elements: [element])
        let material = SCNMaterial()
        material.diffuse.contents = NSColor(calibratedRed: 0.75, green: 0.82, blue: 0.97, alpha: 1)
        if wireframe {
            // Constant shading so the wireframe reads at full contrast over the
            // point cloud regardless of scene lighting.
            material.lightingModel = .constant
            material.fillMode = .lines
            material.isDoubleSided = true
        } else {
            material.lightingModel = .physicallyBased
        }
        geometry.materials = [material]
        return geometry
    }
}
