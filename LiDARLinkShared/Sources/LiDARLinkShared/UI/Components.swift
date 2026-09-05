import SwiftUI

// MARK: - StatusChip

/// Small capsule badge used for status indicators on both apps (e.g. the phone's
/// connection / scan / fps chips, and Mac badges). The capsule is filled with
/// the given `color`; text is always white for contrast.
public struct StatusChip: View {
    public let text: String
    public let color: Color

    public init(text: String, color: Color) {
        self.text = text
        self.color = color
    }

    public var body: some View {
        Text(text)
            .font(.caption2.weight(.medium))
            .foregroundStyle(.white)
            .lineLimit(1)
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(color, in: Capsule())
    }
}

// MARK: - PanelCard

/// Rounded, softly-filled panel used to group content on both apps.
public struct PanelCard: ViewModifier {
    public var padding: CGFloat

    public init(padding: CGFloat = Design.spaceM) {
        self.padding = padding
    }

    public func body(content: Content) -> some View {
        content
            .padding(padding)
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(Design.panelFill, in: RoundedRectangle(cornerRadius: Design.radiusM, style: .continuous))
            .overlay(
                RoundedRectangle(cornerRadius: Design.radiusM, style: .continuous)
                    .strokeBorder(Design.panelStroke, lineWidth: 1)
            )
    }
}

public extension View {
    /// Wraps the view in a rounded, softly-filled panel.
    func panelCard(padding: CGFloat = Design.spaceM) -> some View {
        modifier(PanelCard(padding: padding))
    }
}

// MARK: - EmptyStateView

/// Icon + caption placeholder for empty lists and panels.
public struct EmptyStateView: View {
    public let icon: String
    public let message: String
    public var tint: Color

    public init(icon: String, message: String, tint: Color = .secondary) {
        self.icon = icon
        self.message = message
        self.tint = tint
    }

    public var body: some View {
        VStack(spacing: Design.spaceS) {
            Image(systemName: icon)
                .font(.title3)
                .foregroundStyle(tint)
            Text(message)
                .font(.caption)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, Design.spaceM)
    }
}
