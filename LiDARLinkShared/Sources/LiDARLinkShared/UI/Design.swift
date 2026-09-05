import SwiftUI

/// Shared visual language for the LiDAR Link apps.
///
/// Both the macOS and iOS apps import this from the shared package so the
/// accent color, semantic palette, spacing, radii, and type sizes stay
/// consistent across the two UIs by construction.
public enum Design {
    // MARK: Accent

    /// Brand accent — a "scanning light" cyan shared by both apps.
    public static let accent = Color(red: 0.18, green: 0.70, blue: 0.82)
    /// Deeper accent for pressed/strong states.
    public static let accentStrong = Color(red: 0.10, green: 0.56, blue: 0.66)
    /// Translucent accent wash for tinted fills and selection hints.
    public static let accentSoft = Color(red: 0.18, green: 0.70, blue: 0.82).opacity(0.16)

    // MARK: Semantic colors

    /// Success / connected / live state (adaptive system green).
    public static let success = Color.green
    /// Warning / pending / paused state (adaptive system orange).
    public static let warning = Color.orange
    /// Error / failure state (adaptive system red).
    public static let error = Color.red
    /// Informational state (adaptive system blue).
    public static let info = Color.blue

    // MARK: Neutrals (adaptive)

    /// Subtle panel fill used behind cards and grids.
    public static let panelFill = Color.primary.opacity(0.05)
    /// Hairline stroke for cards and panels.
    public static let panelStroke = Color.primary.opacity(0.08)
    /// Slightly stronger fill for tappable rows and inline cards.
    public static let subtleFill = Color.secondary.opacity(0.14)

    // MARK: Spacing scale

    public static let spaceXS: CGFloat = 4
    public static let spaceS: CGFloat = 8
    public static let spaceM: CGFloat = 12
    public static let spaceL: CGFloat = 16
    public static let spaceXL: CGFloat = 24

    // MARK: Corner radii

    public static let radiusS: CGFloat = 8
    public static let radiusM: CGFloat = 12
    public static let radiusL: CGFloat = 16

    // MARK: Type roles

    /// Numeric stat values (rounded digits keep numbers from jumping).
    public static let statValue = Font.system(.callout, design: .rounded).monospacedDigit()
    /// Small stat captions and labels.
    public static let statLabel = Font.caption
    /// Section titles in panels and sidebars.
    public static let sectionHeader = Font.subheadline.weight(.semibold)
    /// Primary action labels.
    public static let actionLabel = Font.callout.weight(.medium)

    // MARK: Camera overlay tokens

    /// Colors for content drawn on top of the live camera preview on the phone.
    /// These are intentionally fixed (dark scrim + white text) so overlays stay
    /// legible regardless of the system appearance.
    public enum Camera {
        /// Light scrim for subtle backgrounds over the preview.
        public static let scrim = Color.black.opacity(0.35)
        /// Stronger scrim for panels and cards.
        public static let panel = Color.black.opacity(0.55)
        /// Primary overlay text.
        public static let text = Color.white
        /// Secondary overlay text.
        public static let textSecondary = Color.white.opacity(0.8)
        /// Fill for neutral status chips.
        public static let chipFill = Color.black.opacity(0.5)
    }
}
